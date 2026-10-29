#pragma once

#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"

#include <string>
#include <variant>

/// @brief A diffusivity (kinematic viscosity or scalar diffusivity), given either physically (m²/s) or as a lattice
/// value, and which of the two it is.
///
/// A physical diffusivity is the same at every resolution. A lattice value is a numerical choice that scales with
/// the grid, D = value dx²/dt: the small molecular viscosity and scalar diffusivity of a large-eddy simulation, whose
/// subgrid model carries the turbulence, are of this kind. Both convert through the simulation's LatticeConverter,
/// so there is one conversion only.
/// @code
/// using namespace si::symbols;
/// const Diffusivity air = Diffusivity::physical(1.5e-5 * m * m / s);
/// const Diffusivity les = Diffusivity::lattice(si::LatticeDiffusivity(1.0E-5)); // the same lattice value at any cell size
/// @endcode
class Diffusivity {
public:
    /// @brief Makes a diffusivity that is the same physical value at every resolution.
    /// @param d the diffusivity, 0 or more
    /// @return the diffusivity
    /// @throws SetupError when the diffusivity is below 0 or not a number
    static Diffusivity physical(si::KinematicViscosity d);

    /// @brief Makes a diffusivity that is the same lattice value at every resolution (its physical value scales with
    /// dx²/dt).
    /// @param d the diffusivity in cells squared per time step, 0 or more
    /// @return the diffusivity
    /// @throws SetupError when the diffusivity is below 0 or not a number
    static Diffusivity lattice(si::LatticeDiffusivity d);

    /// @brief Gives the diffusivity in the core's lattice units: a lattice value as it was given, a physical value
    /// converted with nu dt / dx².
    /// @param converter the simulation's conversions (its cell and its time step)
    /// @return the diffusivity in cells squared per time step
    float lattice_value(const si::LatticeConverter& converter) const;

    /// @brief Gives the diffusivity in physical units: a physical value as it was given, a lattice value converted
    /// with value dx² / dt.
    /// @param converter the simulation's conversions (its cell and its time step)
    /// @return the diffusivity
    si::KinematicViscosity physical_value(const si::LatticeConverter& converter) const;

    /// @return true when the diffusivity was given as a lattice value, false when it was given as a physical value
    bool is_lattice() const { return std::holds_alternative<si::LatticeDiffusivity>(value_); }

    /// @brief Describes the diffusivity in both forms, for the set-up line of a run's log.
    /// @param converter the simulation's conversions (its cell and its time step)
    /// @return "<lattice value> (<physical value> m^2/s, given as a lattice value)" or "... given as a physical value"
    /// @throws SetupError when the description does not fit its text buffer (a programming error)
    std::string describe(const si::LatticeConverter& converter) const;

private:
    /// @brief Makes the diffusivity from its checked value. Use physical() or lattice(), which check it.
    /// @param value the diffusivity as it was given: physical or lattice
    explicit Diffusivity(std::variant<si::KinematicViscosity, si::LatticeDiffusivity> value) : value_(value) {}

    std::variant<si::KinematicViscosity, si::LatticeDiffusivity> value_{}; ///< the diffusivity as it was given
};
