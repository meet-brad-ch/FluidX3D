#pragma once

#include "setup/atmosphere/cbl_description.hpp"
#include "setup/core/scalar_transport_rules.hpp"
#include "setup/units/units.hpp"

#include <string>

/// @brief The lattice numbers that an atmosphere run gives to the core, as a read-only record: the low layer's (L1)
/// view of a CblDescription.
///
/// Each number comes from the shared conversions (si::LatticeConverter, BoussinesqBuoyancy, ScalarTransportRules);
/// none is computed here on its own, so the record shows exactly what the run uses. The cell, the step and the lattice
/// speed keep their units; the other members are the core's lattice numbers.
struct LatticeParameters {
    si::Length dx{ au::ZERO };      ///< the cell size
    si::Duration dt{ au::ZERO };    ///< the time step (the core's float)
    si::Speed speed{ au::ZERO };    ///< the speed of one cell per time step, dx / dt
    float nu = 0.0f;                ///< the fluid's kinematic viscosity in cells squared per time step
    float diffusivity = 0.0f;       ///< the diffusivity D of heat and water in cells squared per time step
    float alpha = 0.0f;             ///< the alpha that the core gets so that it diffuses with D (ScalarTransportRules::core_alpha())
    float relaxation_time = 0.0f;   ///< the D3Q7 lattice's tauT = 4 D + 1/2; 0 with the finite volumes (SCALAR_FV), which have none
    float inv_prandtl = 0.0f;       ///< 1 / Pr_t, the inverse turbulent Prandtl number (a ratio)
    float fz = 0.0f;                ///< the buoyancy's carrier force fz in lattice units: negative (down z), -carrier
    float beta = 0.0f;              ///< the core's thermal expansion per lattice temperature unit
    float heat_rate = 0.0f;         ///< the lattice temperature that a floor source cell gains per time step
    float moisture_rate = 0.0f;     ///< the lattice moisture that a floor source cell gains per time step (0 in a dry run)
    float nudge_per_step = 0.0f;    ///< dt / tau of the wind's nudging (0 without wind)
    TransportBuild build{};         ///< the build's transports that these numbers are for

    /// @brief Computes the lattice numbers of a description for a build.
    /// @param description the run
    /// @param build the build's transports (current_build() in a header-side caller)
    /// @return the numbers
    /// @throws SetupError when a tabulated sounding has no theta_0, the cell size is not a float exactly, a
    /// diffusivity is below 0, the build has SCALAR_FV without THERMAL_DIFFUSIVITY_FIX, gravity or the carrier is not
    /// more than 0, a moist run's Bowen ratio is not more than 0, or the wind's nudging time scale is not more than 0
    static LatticeParameters of(const CblDescription& description, const TransportBuild& build);

    /// @brief Describes the numbers that decide the run, for the run's log.
    /// @return one line: dx, dt, the lattice speed, nu, D, alpha (with ", finite volumes" for SCALAR_FV), 1/Pr_t,
    /// fz, beta, the heat rate and, when the moisture rate is above 0, ", moisture rate set"
    /// @throws SetupError when the text does not fit its buffer
    std::string describe() const;
};
