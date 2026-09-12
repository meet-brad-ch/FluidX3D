#pragma once

#include "core/types.hpp"

struct FluidProperties {
    float32_t density;              // kg/m³
    float32_t kinematic_viscosity;  // m²/s
    float32_t thermal_diffusivity;  // m²/s (alpha = k/(rho*cp))
    float32_t thermal_expansion;    // 1/K (beta)
};

// fluid properties at 20°C, 1 atm
namespace Fluid {
    constexpr FluidProperties AIR = { 1.225f, 1.48e-5f, 2.08e-5f, 0.00341f };   // Pr = 0.71
    constexpr FluidProperties WATER = { 998.2f, 1.004e-6f, 1.43e-7f, 0.000207f }; // Pr = 7.0
}
