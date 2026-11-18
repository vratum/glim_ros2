#pragma once
// Register the factor in libglim itself, so an archived map can be loaded even
// when the barometer is unplugged and the sensor extension is not instantiated.
#include <glim/factors/reference_height_factor.hpp>
#include <cmath>
#include <stdexcept>
namespace glim_barometer_extension {
using ReferenceHeightFactor = glim::ReferenceHeightFactor;

inline gtsam::SharedNoiseModel heightNoise(double variance_m2, double influence) {
  if (!std::isfinite(variance_m2) || variance_m2<=0 || !std::isfinite(influence) || influence<=0)
    throw std::invalid_argument("Height variance and influence must be finite and positive");
  return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
    gtsam::noiseModel::Isotropic::Sigma(1,std::sqrt(variance_m2/influence)));
}
}
