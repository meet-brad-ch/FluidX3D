#pragma once

#include "setup/core/types.hpp"
#include "setup/units/units.hpp"

/// @file fluids.hpp
/// @brief The material properties of a fluid (FluidProperties) and two common fluids (Fluid::AIR, Fluid::WATER).

/// @brief The material properties of a fluid: what Simulation needs to set the lattice density, the viscosity, the
/// thermal diffusivity and the buoyancy.
///
/// A custom fluid (here a glycerine):
/// @code
/// using namespace si::symbols;
/// const FluidProperties glycerine{ .density = 1260.0 * kg / (m * m * m), .kinematic_viscosity = 1.12E-3 * m * m / s };
/// @endcode
struct FluidProperties {
    si::Density density{ au::ZERO };                        ///< the density, which becomes the lattice density 1
    si::KinematicViscosity kinematic_viscosity{ au::ZERO }; ///< the kinematic viscosity nu
    si::KinematicViscosity thermal_diffusivity{ au::ZERO }; ///< the thermal diffusivity alpha = k / (rho cp)
    si::ThermalExpansion thermal_expansion{ au::ZERO };     ///< the thermal expansion coefficient beta

    /// @brief Gives the same fluid with another viscosity, for example to match a Reynolds number:
    /// Fluid::WATER.with_viscosity(u * L / Re).
    /// @param viscosity the kinematic viscosity of the copy
    /// @return a copy of this fluid with that viscosity
    constexpr FluidProperties with_viscosity(si::KinematicViscosity viscosity) const {
        FluidProperties fluid{ *this };
        fluid.kinematic_viscosity = viscosity;
        return fluid;
    }
};

/// @brief Two fluids at 20 °C and 1 atm: Fluid::AIR and Fluid::WATER.
struct Fluid {
    /// Air: Prandtl number 0.71.
    static constexpr FluidProperties AIR{ .density = (si::kilograms / au::cubed(au::meters))(1.225),
                                          .kinematic_viscosity = (au::squared(au::meters) / au::seconds)(1.48e-5),
                                          .thermal_diffusivity = (au::squared(au::meters) / au::seconds)(2.08e-5),
                                          .thermal_expansion = (au::inverse(au::kelvins))(0.00341) };
    /// Water: Prandtl number 7.0.
    static constexpr FluidProperties WATER{ .density = (si::kilograms / au::cubed(au::meters))(998.2),
                                            .kinematic_viscosity = (au::squared(au::meters) / au::seconds)(1.004e-6),
                                            .thermal_diffusivity = (au::squared(au::meters) / au::seconds)(1.43e-7),
                                            .thermal_expansion = (au::inverse(au::kelvins))(0.000207) };
};
