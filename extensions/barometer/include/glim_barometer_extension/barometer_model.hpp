#pragma once

#include <cmath>
#include <limits>
#include <optional>

namespace glim_barometer_extension
{

struct PressureSample
{
  double stamp_sec{0.0};
  double pressure_pa{0.0};
  double variance_pa2{std::numeric_limits<double>::quiet_NaN()};
};

struct RelativeHeightMeasurement
{
  double delta_height_m{0.0};
  double variance_m2{0.0};
};

// A standard-atmosphere conversion used only to turn a pressure difference into
// a local relative-height observation.  It is intentionally not a claim that
// the pressure is an absolute geodetic altitude.
class BarometerModel
{
public:
  static constexpr double kStandardAtmosphereHeightScaleM = 44330.0;
  static constexpr double kPressureExponent = 0.1903;

  static bool validPressure(double pressure_pa)
  {
    return std::isfinite(pressure_pa) && pressure_pa > 0.0;
  }

  static bool validReferencePressure(double reference_pressure_pa)
  {
    return validPressure(reference_pressure_pa);
  }

  static std::optional<double> heightMeters(
    double pressure_pa, double reference_pressure_pa)
  {
    if (!validPressure(pressure_pa) || !validReferencePressure(reference_pressure_pa)) {
      return std::nullopt;
    }
    return kStandardAtmosphereHeightScaleM *
      (1.0 - std::pow(pressure_pa / reference_pressure_pa, kPressureExponent));
  }

  static std::optional<double> heightDerivativeMetersPerPascal(
    double pressure_pa, double reference_pressure_pa)
  {
    if (!validPressure(pressure_pa) || !validReferencePressure(reference_pressure_pa)) {
      return std::nullopt;
    }
    return -kStandardAtmosphereHeightScaleM * kPressureExponent / reference_pressure_pa *
      std::pow(pressure_pa / reference_pressure_pa, kPressureExponent - 1.0);
  }

  static std::optional<double> heightVarianceM2(
    const PressureSample & sample, double reference_pressure_pa,
    double default_height_stddev_m)
  {
    if (!std::isfinite(default_height_stddev_m) || default_height_stddev_m <= 0.0 ||
      !validPressure(sample.pressure_pa) || !validReferencePressure(reference_pressure_pa))
    {
      return std::nullopt;
    }

    if (!std::isfinite(sample.variance_pa2) || sample.variance_pa2 <= 0.0) {
      return default_height_stddev_m * default_height_stddev_m;
    }

    const auto derivative = heightDerivativeMetersPerPascal(
      sample.pressure_pa, reference_pressure_pa);
    if (!derivative.has_value()) {
      return std::nullopt;
    }
    const double variance_m2 = *derivative * *derivative * sample.variance_pa2;
    return std::isfinite(variance_m2) && variance_m2 > 0.0 ?
      std::optional<double>(variance_m2) : std::nullopt;
  }

  static std::optional<RelativeHeightMeasurement> relativeHeight(
    const PressureSample & reference, const PressureSample & current,
    double default_height_stddev_m)
  {
    // This is a relative observation.  The reference is normally the first
    // pressure sample of the session, so the observation does not accumulate
    // per-sample error as a random walk.
    const double reference_pressure_pa = reference.pressure_pa;
    const auto reference_height = heightMeters(reference.pressure_pa, reference_pressure_pa);
    const auto current_height = heightMeters(current.pressure_pa, reference_pressure_pa);
    const auto reference_variance = heightVarianceM2(
      reference, reference_pressure_pa, default_height_stddev_m);
    const auto current_variance = heightVarianceM2(
      current, reference_pressure_pa, default_height_stddev_m);
    if (!reference_height.has_value() || !current_height.has_value() ||
      !reference_variance.has_value() || !current_variance.has_value())
    {
      return std::nullopt;
    }

    const double variance_m2 = *reference_variance + *current_variance;
    if (!std::isfinite(variance_m2) || variance_m2 <= 0.0) {
      return std::nullopt;
    }
    return RelativeHeightMeasurement{*current_height - *reference_height, variance_m2};
  }
};

}  // namespace glim_barometer_extension
