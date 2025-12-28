#pragma once

#include "core/types.hpp"

/**
 * @file fluids.hpp
 * @brief Common fluid property presets for FluidX3D simulations
 *
 * Provides predefined fluid properties at standard conditions (20°C, 1 atm).
 */

/**
 * @struct FluidProperties
 * @brief Physical properties of a fluid
 */
struct FluidProperties {
    float32_t density;              ///< Density in kg/m³
    float32_t kinematic_viscosity;  ///< Kinematic viscosity in m²/s
    float32_t dynamic_viscosity;    ///< Dynamic viscosity in Pa·s (kg/(m·s))
    float32_t thermal_diffusivity;  ///< Thermal diffusivity in m²/s (alpha = k/(rho*cp))
    float32_t thermal_expansion;    ///< Thermal expansion coefficient in 1/K (beta)

    /**
     * @brief Calculate Reynolds number for this fluid
     * @param characteristic_length Reference length in meters
     * @param velocity Flow velocity in m/s
     * @return Reynolds number (dimensionless)
     */
    float32_t reynolds(float32_t characteristic_length, float32_t velocity) const {
        return characteristic_length * velocity / kinematic_viscosity;
    }

    /**
     * @brief Calculate Prandtl number for this fluid
     * @return Prandtl number (dimensionless, Pr = nu/alpha)
     */
    float32_t prandtl() const {
        return kinematic_viscosity / thermal_diffusivity;
    }

    /**
     * @brief Calculate Rayleigh number for this fluid
     * @param delta_T Temperature difference in Kelvin
     * @param length Characteristic length in meters
     * @param gravity Gravitational acceleration in m/s² (default: 9.81)
     * @return Rayleigh number (dimensionless)
     */
    float32_t rayleigh(float32_t delta_T, float32_t length, float32_t gravity = 9.81f) const {
        return gravity * thermal_expansion * delta_T * length * length * length /
               (kinematic_viscosity * thermal_diffusivity);
    }
};

/**
 * @namespace Fluid
 * @brief Predefined fluid properties at standard conditions (20°C, 1 atm)
 *
 * Thermal properties (thermal diffusivity alpha, thermal expansion beta):
 * - alpha = k / (rho * cp), where k is thermal conductivity, cp is specific heat
 * - beta = 1/T for ideal gases, measured values for liquids
 */
namespace Fluid {
    /// Air at 20°C, 1 atm (Pr = 0.71)
    constexpr FluidProperties AIR = {
        1.225f,         // density: kg/m³
        1.48e-5f,       // kinematic viscosity: m²/s
        1.81e-5f,       // dynamic viscosity: Pa·s
        2.08e-5f,       // thermal diffusivity: m²/s
        0.00341f        // thermal expansion: 1/K (≈ 1/293K)
    };

    /// Water at 20°C, 1 atm (Pr = 7.0)
    constexpr FluidProperties WATER = {
        998.2f,         // density: kg/m³
        1.004e-6f,      // kinematic viscosity: m²/s
        1.002e-3f,      // dynamic viscosity: Pa·s
        1.43e-7f,       // thermal diffusivity: m²/s
        0.000207f       // thermal expansion: 1/K
    };

    /// Seawater at 20°C (35 ppt salinity)
    constexpr FluidProperties SEAWATER = {
        1025.0f,        // density: kg/m³
        1.08e-6f,       // kinematic viscosity: m²/s
        1.08e-3f,       // dynamic viscosity: Pa·s
        1.40e-7f,       // thermal diffusivity: m²/s
        0.000200f       // thermal expansion: 1/K
    };

    /// SAE 10W motor oil at 20°C
    constexpr FluidProperties OIL_SAE10W = {
        870.0f,         // density: kg/m³
        6.5e-5f,        // kinematic viscosity: m²/s
        5.7e-2f,        // dynamic viscosity: Pa·s
        8.5e-8f,        // thermal diffusivity: m²/s
        0.00070f        // thermal expansion: 1/K
    };

    /// Glycerin at 20°C
    constexpr FluidProperties GLYCERIN = {
        1261.0f,        // density: kg/m³
        1.18e-3f,       // kinematic viscosity: m²/s
        1.49f,          // dynamic viscosity: Pa·s
        9.8e-8f,        // thermal diffusivity: m²/s
        0.00050f        // thermal expansion: 1/K
    };

    /// Honey at 20°C (approximate, varies widely)
    constexpr FluidProperties HONEY = {
        1420.0f,        // density: kg/m³
        2.0e-3f,        // kinematic viscosity: m²/s
        2.84f,          // dynamic viscosity: Pa·s
        1.0e-7f,        // thermal diffusivity: m²/s
        0.00040f        // thermal expansion: 1/K
    };
}
