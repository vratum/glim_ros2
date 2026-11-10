#pragma once
// Register the factor in libglim itself, so an archived map can be loaded even
// when the barometer is unplugged and the sensor extension is not instantiated.
#include <glim/factors/reference_height_factor.hpp>
namespace glim_barometer_extension {
using ReferenceHeightFactor = glim::ReferenceHeightFactor;
}
