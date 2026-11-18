#include "glim_barometer_extension/barometer_model.hpp"
#include "glim_barometer_extension/relative_height_factor.hpp"
#include "glim_barometer_extension/pressure_replay.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include <cassert>
#include <cmath>
#include <gtsam/base/serialization.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <sstream>
#include <glim/util/serialization.hpp>
#include <unistd.h>
#include <cstdio>
#include <fstream>

namespace be = glim_barometer_extension;

int main()
{
  const be::PressureSample start{1.0, 100000.0, 0.0};
  const be::PressureSample higher_pressure{2.0, 100129.0, 0.0};

  const auto start_height = be::BarometerModel::heightMeters(
    start.pressure_pa, start.pressure_pa);
  assert(start_height.has_value());
  assert(std::abs(*start_height) < 1e-9);

  const auto relative = be::BarometerModel::relativeHeight(
    start, higher_pressure, 1.0);
  assert(relative.has_value());
  // Rising pressure means descending.  129 Pa near 100 kPa is about 10.88 m,
  // not a ninety-metre climb (the old fixture had both sign and scale wrong).
  assert(std::abs(relative->delta_height_m - (-10.8767597043)) < 1e-6);
  const auto climb = be::BarometerModel::relativeHeight(
    start, be::PressureSample{2.0, 99871.0, 0.0}, 1.0);
  assert(climb && climb->delta_height_m > 10.0 && climb->delta_height_m < 12.0);
  // Two independent fallback one-metre samples produce two square metres.
  assert(std::abs(relative->variance_m2 - 2.0) < 1e-9);
  assert(be::BarometerModel::timely(start,3.5));
  assert(!be::BarometerModel::timely(start,3.501));
  assert(!be::BarometerModel::timely(start,.99));
  assert(!be::BarometerModel::timely(start,std::numeric_limits<double>::quiet_NaN()));
  const auto robust=be::heightNoise(2.0,2.0);
  const auto robust_model=std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(robust);
  assert(robust_model);
  assert(std::abs(robust_model->robust()->weight(.5)-1)<1e-9);
  assert(robust_model->robust()->weight(20)<.1);

  const std::string metrics="/tmp/lidar_lab_pressure_replay_"+std::to_string(getpid())+".ndjson";
  std::ofstream(metrics) << "{\"stamp_sec\":10,\"barometer_raw_pressure_pa\":100000}\n"
                       << "{\"stamp_sec\":10,\"barometer_pressure_pa\":99900}\n"
                       << "{\"stamp_sec\":12,\"barometer_raw_pressure_pa\":100100}\n";
  const be::PressureReplay replay(metrics);
  assert(replay.at(10.001).pressure_pa==100000);
  assert(replay.at(11.999).pressure_pa==100100);
  assert(replay.at(12.001).stamp_sec==12.001);
  bool mismatch=false;
  try { replay.at(11); } catch(const std::runtime_error&) { mismatch=true; }
  assert(mismatch);
  std::ofstream(metrics) << "{\"stamp_sec\":12,\"barometer_raw_pressure_pa\":100000}\n"
                       << "{\"stamp_sec\":10,\"barometer_raw_pressure_pa\":100100}\n";
  bool unordered=false;
  try { const be::PressureReplay invalid(metrics); } catch(const std::invalid_argument&) { unordered=true; }
  assert(unordered);
  std::remove(metrics.c_str());

  // All later samples are measured against the session start, rather than
  // accumulating a chain of pressure-to-pressure deltas.
  const be::PressureSample later_pressure{3.0, 100258.0, 0.0};
  const auto later_relative = be::BarometerModel::relativeHeight(
    start, later_pressure, 1.0);
  assert(later_relative.has_value());
  assert(later_relative->delta_height_m < relative->delta_height_m);
  assert(std::abs(later_relative->variance_m2 - 2.0) < 1e-9);

  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const be::ReferenceHeightFactor factor(2, 3.0, noise);
  const gtsam::Pose3 current(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 3.0));
  gtsam::Matrix H_current;
  const auto error = factor.evaluateError(current, H_current);
  assert(std::abs(error(0)) < 1e-9);
  assert(H_current.rows() == 1 && H_current.cols() == 6);
  assert(std::abs(H_current(0, 5) - 1.0) < 1e-9);
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<be::ReferenceHeightFactor>(2,3.0,robust);
  const std::string path="/tmp/lidar_lab_barometer_roundtrip_"+std::to_string(getpid())+".bin";
  glim::serializeToBinaryFile(graph,path,false);
  gtsam::NonlinearFactorGraph restored;
  gtsam::deserializeFromBinaryFile(path,restored);
  std::remove(path.c_str());
  assert(restored.size()==1);
  auto height=dynamic_cast<be::ReferenceHeightFactor*>(restored[0].get());
  assert(height && std::abs(height->evaluateError(current)(0))<1e-9);
  const auto rekeyed=restored.rekey(std::map<gtsam::Key,gtsam::Key>{{2,42}});
  assert(rekeyed[0]->keys()[0]==42);
  const be::ReferenceHeightFactor tilted(3,3.0,noise,gtsam::Vector3::UnitX());
  const gtsam::Pose3 translated(gtsam::Rot3(),gtsam::Point3(3.0,0.0,100.0));
  assert(std::abs(tilted.evaluateError(translated)(0))<1e-9);
  graph.resize(0);
  graph.emplace_shared<be::ReferenceHeightFactor>(tilted);
  glim::serializeToBinaryFile(graph,path,false);
  gtsam::deserializeFromBinaryFile(path,restored);
  std::remove(path.c_str());
  height=dynamic_cast<be::ReferenceHeightFactor*>(restored[0].get());
  assert(height && std::abs(height->evaluateError(translated)(0))<1e-9);
  const auto tilted_rekeyed=restored.rekey(std::map<gtsam::Key,gtsam::Key>{{3,43}});
  assert(tilted_rekeyed[0]->keys()[0]==43);
  assert(std::abs(dynamic_cast<be::ReferenceHeightFactor*>(tilted_rekeyed[0].get())
    ->evaluateError(translated)(0))<1e-9);
  return 0;
}
