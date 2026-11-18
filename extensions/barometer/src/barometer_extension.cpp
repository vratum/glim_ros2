#include "glim_barometer_extension/barometer_model.hpp"
#include "glim_barometer_extension/relative_height_factor.hpp"
#include "glim_barometer_extension/pressure_replay.hpp"
#include <gtsam/base/serialization.h>

#include <glim/odometry/callbacks.hpp>
#include <glim/mapping/callbacks.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/util/extension_module_ros2.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>

#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

namespace glim_barometer_extension
{
namespace
{

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

struct AnchoredPressureSample
{
  PressureSample pressure;
  double world_z{0.0};
};
struct PendingResidual { std::uint64_t current_id; double expected_world_z; };
struct PendingGlobalHeight { PressureSample pressure; Eigen::Vector3d vertical_axis; };

class BarometerExtension final : public glim::ExtensionModuleROS2
{
public:
  BarometerExtension()
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;
    glim::OdometryEstimationCallbacks::on_new_frame.add(
      std::bind(&BarometerExtension::onNewFrame, this, _1));
    glim::OdometryEstimationCallbacks::on_smoother_update.add(
      std::bind(&BarometerExtension::onSmootherUpdate, this, _1, _2, _3, _4));
    glim::OdometryEstimationCallbacks::on_update_new_frame.add(
      std::bind(&BarometerExtension::onUpdatedFrame, this, _1));
    glim::GlobalMappingCallbacks::on_insert_submap.add(
      std::bind(&BarometerExtension::onInsertSubmap, this, _1));
    glim::SubMappingCallbacks::on_new_submap.add([this](const auto& submap) {
      if(!submap || submap->frames.empty()) return;
      const auto stamp=submap->origin_frame()->stamp;
      const auto pressure=newestPressureAtOrBefore(stamp);
      if(pressure) {
        std::lock_guard<std::mutex> lock(mutex_);
        deferred_submap_pressures_.insert_or_assign(stamp,*pressure);
      }
    });
    glim::GlobalMappingCallbacks::on_independent_session_submap.add(
      [this](const auto& submap,auto& factors,auto& values) {
        onInsertSubmap(submap);
        addGlobalHeightFactors(factors,values);
      });
    glim::GlobalMappingCallbacks::on_smoother_update.add(
      std::bind(&BarometerExtension::onGlobalSmootherUpdate, this, _1, _2, _3));
  }

  std::vector<glim::GenericTopicSubscription::Ptr> create_subscriptions(
    rclcpp::Node & node) override
  {
    node_ = &node;
    residual_publisher_ = node.create_publisher<std_msgs::msg::Float64>("/barometer/z_residual", 10);
    topic_ = node.declare_parameter<std::string>("barometer_topic", "/barometer/pressure");
    default_height_stddev_m_ = node.declare_parameter<double>(
      "barometer_default_height_stddev_m", 1.0);
    // GLIM configuration is JSON-based.  This extension is constructed by
    // GlimROS after that JSON has been loaded; it is not given the JSON keys
    // as ROS parameters.  Reading a ROS parameter here therefore silently
    // selected the default (1.0) even when a session requested 0.0.
    const glim::Config config_ros(glim::GlobalConfig::get_config_path("config_ros"));
    influence_ = config_ros.param<double>("glim_ros", "barometer_influence", 1.0);
    const auto replay_path=config_ros.param<std::string>("glim_ros","barometer_replay_metrics_path","");
    if (!replay_path.empty()) {
      pressure_replay_=std::make_unique<PressureReplay>(replay_path);
      RCLCPP_INFO(node.get_logger(),"Using original pressure observations from %s",replay_path.c_str());
    }
    queue_capacity_ = static_cast<std::size_t>(node.declare_parameter<int>(
      "barometer_queue_capacity", 512));

    if (!std::isfinite(default_height_stddev_m_) || default_height_stddev_m_ <= 0.0) {
      throw std::invalid_argument("barometer_default_height_stddev_m must be finite and positive");
    }
    if (!std::isfinite(influence_) || influence_ < 0.0) {
      throw std::invalid_argument("barometer_influence must be finite and non-negative");
    }
    if (queue_capacity_ == 0) {
      throw std::invalid_argument("barometer_queue_capacity must be positive");
    }

    RCLCPP_INFO(
      node.get_logger(),
      "barometer extension: topic=%s relative_reference=session_start default_height_stddev_m=%.3f influence=%.3f global_backends=enabled",
      topic_.c_str(), default_height_stddev_m_, influence_);

    using Message = sensor_msgs::msg::FluidPressure;
    return {std::make_shared<glim::TopicSubscription<Message>>(
      topic_, std::bind(&BarometerExtension::onPressure, this, std::placeholders::_1))};
  }

private:
  void onPressure(const std::shared_ptr<const sensor_msgs::msg::FluidPressure> & message)
  {
    if (!message || !BarometerModel::validPressure(message->fluid_pressure)) {
      ++rejected_pressure_count_;
      return;
    }

    double stamp_sec = stampSeconds(message->header.stamp);
    // The planned USB bridge has no sensor clock.  A zero header stamp therefore
    // means "stamp on receipt" in the ROS clock, never an invented sensor time.
    if (!(stamp_sec > 0.0) && node_ != nullptr) {
      stamp_sec = node_->get_clock()->now().seconds();
    }
    if (!std::isfinite(stamp_sec) || stamp_sec <= 0.0) {
      ++rejected_pressure_count_;
      return;
    }

    PressureSample sample{stamp_sec, message->fluid_pressure, message->variance};
    if (pressure_replay_) sample=pressure_replay_->at(stamp_sec);
    std::lock_guard<std::mutex> lock(mutex_);
    if (!session_reference_pressure_.has_value()) {
      session_reference_pressure_ = sample;
    }
    const auto insertion = std::upper_bound(
      pending_pressures_.begin(), pending_pressures_.end(), sample.stamp_sec,
      [](double stamp, const PressureSample & candidate) { return stamp < candidate.stamp_sec; });
    pending_pressures_.insert(insertion, sample);
    while (pending_pressures_.size() > queue_capacity_) {
      pending_pressures_.pop_front();
      ++dropped_pressure_count_;
    }
    pressure_history_.insert(
      std::upper_bound(
        pressure_history_.begin(), pressure_history_.end(), sample.stamp_sec,
        [](double stamp, const PressureSample & candidate) { return stamp < candidate.stamp_sec; }),
      sample);
    while (pressure_history_.size() > queue_capacity_) {
      pressure_history_.pop_front();
    }
  }

  void onNewFrame(const glim::EstimationFrame::ConstPtr & frame)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_frame_ = frame;
  }

  std::optional<PressureSample> takeNewestPressureAtOrBefore(double frame_stamp_sec)
  {
    std::optional<PressureSample> selected;
    while (!pending_pressures_.empty() && pending_pressures_.front().stamp_sec <= frame_stamp_sec) {
      selected = pending_pressures_.front();
      pending_pressures_.pop_front();
    }
    if (selected && !BarometerModel::timely(*selected,frame_stamp_sec)) return std::nullopt;
    return selected;
  }

  std::optional<PressureSample> newestPressureAtOrBefore(double stamp_sec)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = std::upper_bound(
      pressure_history_.begin(), pressure_history_.end(), stamp_sec,
      [](double stamp, const PressureSample & candidate) { return stamp < candidate.stamp_sec; });
    if (found == pressure_history_.begin()) {
      return std::nullopt;
    }
    const auto sample=*std::prev(found);
    return BarometerModel::timely(sample,stamp_sec) ? std::optional<PressureSample>(sample) : std::nullopt;
  }

  std::optional<PressureSample> sessionReferencePressure()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_reference_pressure_;
  }

  void onSmootherUpdate(
    gtsam_points::IncrementalFixedLagSmootherExtWithFallback & smoother,
    gtsam::NonlinearFactorGraph & new_factors,
    gtsam::Values & new_values,
    std::map<std::uint64_t, double> &)
  {
    glim::EstimationFrame::ConstPtr frame;
    std::optional<PressureSample> pressure;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frame = latest_frame_;
      if (!frame) {
        return;
      }
      pressure = takeNewestPressureAtOrBefore(frame->stamp);
    }
    if (!pressure.has_value()) {
      return;
    }

    using gtsam::symbol_shorthand::X;
    const gtsam::Key current_key = X(frame->id);
    if (!new_values.exists(current_key)) {
      ++rejected_pressure_count_;
      return;
    }

    if (!anchor_.has_value()) {
      const auto session_reference = sessionReferencePressure();
      if (!session_reference.has_value()) {
        ++rejected_pressure_count_;
        return;
      }
      anchor_ = AnchoredPressureSample{
        *session_reference,
        new_values.at<gtsam::Pose3>(current_key).translation().z()};
      return;
    }

    const auto measurement = BarometerModel::relativeHeight(
      anchor_->pressure, *pressure, default_height_stddev_m_);
    if (!measurement.has_value()) {
      ++rejected_pressure_count_;
      return;
    }

    if (influence_ == 0.0) {
      return;
    }
    const auto noise = heightNoise(measurement->variance_m2,influence_);
    const double expected_world_z = anchor_->world_z + measurement->delta_height_m;
    new_factors.emplace_shared<ReferenceHeightFactor>(current_key, expected_world_z, noise);
    pending_residual_ = PendingResidual{static_cast<std::uint64_t>(frame->id), expected_world_z};
    ++inserted_factor_count_;
  }

  void onUpdatedFrame(const glim::EstimationFrame::ConstPtr & frame)
  {
    if (!pending_residual_.has_value() || !residual_publisher_ || frame->id != pending_residual_->current_id) return;
    std_msgs::msg::Float64 message;
    message.data = frame->T_world_imu.translation().z() - pending_residual_->expected_world_z;
    residual_publisher_->publish(message);
    pending_residual_.reset();
  }

  void onInsertSubmap(const glim::SubMap::ConstPtr & submap)
  {
    if (!submap || submap->frames.empty()) {
      return;
    }
    const auto stamp=submap->origin_frame()->stamp;
    auto pressure = newestPressureAtOrBefore(stamp);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto cached=deferred_submap_pressures_.find(stamp);
      if(cached!=deferred_submap_pressures_.end()) {
        pressure=cached->second;
        deferred_submap_pressures_.erase(cached);
      }
    }
    if (pressure.has_value()) {
      const auto axis=submap->get_custom_data<Eigen::Vector3d>("session_gravity_axis");
      pending_global_heights_.insert_or_assign(
        static_cast<std::uint64_t>(submap->id), PendingGlobalHeight{*pressure,axis ? *axis : Eigen::Vector3d::UnitZ()});
    }
  }

  void onGlobalSmootherUpdate(
    gtsam_points::ISAM2Ext &,
    gtsam::NonlinearFactorGraph & new_factors,
    gtsam::Values & new_values)
  {
    addGlobalHeightFactors(new_factors,new_values);
  }

  void addGlobalHeightFactors(gtsam::NonlinearFactorGraph & new_factors,gtsam::Values & new_values)
  {
    if (influence_ == 0.0) {
      return;
    }

    using gtsam::symbol_shorthand::X;
    for (auto it = pending_global_heights_.begin(); it != pending_global_heights_.end();) {
      const gtsam::Key key = X(it->first);
      if (!new_values.exists(key)) {
        ++it;
        continue;
      }

      const auto pose = new_values.at<gtsam::Pose3>(key);
      if (!global_anchor_.has_value()) {
        // Pressure and anchor pose describe the same submap-origin time.
        // A physical restart gets its own weather/height offset.
        global_anchor_ = AnchoredPressureSample{
          it->second.pressure, it->second.vertical_axis.dot(pose.translation())};
      } else {
        const auto measurement = BarometerModel::relativeHeight(
          global_anchor_->pressure, it->second.pressure, default_height_stddev_m_);
        if (measurement.has_value()) {
          const auto noise = heightNoise(measurement->variance_m2,influence_);
          new_factors.emplace_shared<ReferenceHeightFactor>(
            key, global_anchor_->world_z + measurement->delta_height_m, noise,it->second.vertical_axis);
          ++inserted_global_factor_count_;
        }
      }
      it = pending_global_heights_.erase(it);
    }
  }

  rclcpp::Node * node_{nullptr};
  std::unique_ptr<PressureReplay> pressure_replay_;
  std::string topic_{"/barometer/pressure"};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr residual_publisher_;
  double default_height_stddev_m_{1.0};
  double influence_{1.0};
  std::size_t queue_capacity_{512};

  std::mutex mutex_;
  std::deque<PressureSample> pending_pressures_;
  std::deque<PressureSample> pressure_history_;
  std::optional<PressureSample> session_reference_pressure_;
  glim::EstimationFrame::ConstPtr latest_frame_;
  std::optional<AnchoredPressureSample> anchor_;
  std::optional<PendingResidual> pending_residual_;
  std::optional<AnchoredPressureSample> global_anchor_;
  std::map<std::uint64_t, PendingGlobalHeight> pending_global_heights_;
  std::map<double,PressureSample> deferred_submap_pressures_;
  std::size_t inserted_factor_count_{0};
  std::size_t inserted_global_factor_count_{0};
  std::size_t rejected_pressure_count_{0};
  std::size_t dropped_pressure_count_{0};
};

}  // namespace
}  // namespace glim_barometer_extension

extern "C" glim::ExtensionModule * create_extension_module()
{
  return new glim_barometer_extension::BarometerExtension();
}
