#pragma once
#include "barometer_model.hpp"
#include <boost/property_tree/json_parser.hpp>
#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

namespace glim_barometer_extension {
// Replace the already-filtered pressure in an old bag with its original
// observation. Match the recorded topic stamp; never shift clocks or emit a
// second pressure stream. No new bag or converted point cloud is required.
class PressureReplay {
public:
  explicit PressureReplay(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::invalid_argument("Cannot open raw pressure metrics: "+path);
    std::string line;
    while (std::getline(input,line)) {
      if (line.empty()) continue;
      boost::property_tree::ptree row;
      std::istringstream stream(line);
      boost::property_tree::read_json(stream,row);
      const auto pressure=row.get_optional<double>("barometer_raw_pressure_pa");
      if (!pressure) continue;
      PressureSample sample{row.get<double>("stamp_sec"),*pressure,0.0};
      if (!std::isfinite(sample.stamp_sec) || sample.stamp_sec<=0 || !BarometerModel::validPressure(sample.pressure_pa) ||
          (!samples_.empty() && sample.stamp_sec<=samples_.back().stamp_sec))
        throw std::invalid_argument("Invalid or unordered raw pressure metrics: "+path);
      samples_.push_back(sample);
    }
    if (samples_.empty()) throw std::invalid_argument("No raw pressure observations: "+path);
  }
  PressureSample at(double stamp) const {
    auto it=std::lower_bound(samples_.begin(),samples_.end(),stamp,
      [](const auto& sample,double time) { return sample.stamp_sec<time; });
    const PressureSample* best=it==samples_.end() ? &samples_.back() : &*it;
    if (it!=samples_.begin() && std::abs(std::prev(it)->stamp_sec-stamp)<std::abs(best->stamp_sec-stamp))
      best=&*std::prev(it);
    if (!std::isfinite(stamp) || std::abs(best->stamp_sec-stamp)>.25)
      throw std::runtime_error("Recorded pressure has no matching raw observation; refusing mixed replay");
    return {stamp,best->pressure_pa,best->variance_pa2};
  }
private:
  std::vector<PressureSample> samples_;
};
}
