#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"

/// Material properties of a fluid.
struct FluidProperties {
    Density density;                        ///< kg/m³
    KinematicViscosity kinematic_viscosity; ///< m²/s
    KinematicViscosity thermal_diffusivity; ///< m²/s, alpha = k/(rho*cp)
    ThermalExpansion thermal_expansion;     ///< 1/K, beta
};

/// Fluids at 20 °C and 1 atm: Fluid::AIR, Fluid::WATER.
struct Fluid {
    /// Air, Prandtl number 0.71.
    static constexpr FluidProperties AIR = { Density::from_si(1.225f), KinematicViscosity::from_si(1.48e-5f),
                                             KinematicViscosity::from_si(2.08e-5f), ThermalExpansion::from_si(0.00341f) };
    /// Water, Prandtl number 7.0.
    static constexpr FluidProperties WATER = { Density::from_si(998.2f), KinematicViscosity::from_si(1.004e-6f),
                                               KinematicViscosity::from_si(1.43e-7f), ThermalExpansion::from_si(0.000207f) };
};
