#pragma once

#include "setup/units/units.hpp"

#include "moist_air.hpp"

/// @file physical_constants.hpp
/// @brief The physical constants of the moist air, typed. The values come from the core's src/moist_air.hpp, the one
/// source of them; this header only gives them their units.

namespace si {

/// @brief The physical constants of the moist-air model (src/moist_air.hpp), each with its unit.
struct PhysicalConstants {
    static constexpr Acceleration gravity = (au::meters / au::squared(au::second))(moist_air::G);           ///< the gravitational acceleration g, 9.81 m/s²
    static constexpr SpecificHeat dry_air_gas_constant = (au::joules / (kilograms * au::kelvins))(moist_air::Rd); ///< the gas constant of dry air Rd, 287.04 J/(kg K)
    static constexpr SpecificHeat vapour_gas_constant = (au::joules / (kilograms * au::kelvins))(moist_air::Rv);  ///< the gas constant of water vapour Rv, 461.5 J/(kg K)
    static constexpr SpecificHeat dry_air_heat_capacity = (au::joules / (kilograms * au::kelvins))(moist_air::Cp); ///< the specific heat of dry air at constant pressure cp, 1004 J/(kg K)
    static constexpr LatentHeat latent_heat = (au::joules / kilograms_of_water)(moist_air::Lv);             ///< the latent heat of condensation Lv, 2.5e6 J per kg of water
    static constexpr Pressure ground_pressure = au::pascals(moist_air::GroundPressure);                      ///< the pressure at the ground, 101325 Pa
    static constexpr Pressure exner_reference = au::pascals(moist_air::PRef);                                ///< the reference pressure of the Exner function (p / p_ref)^(Rd/cp), 1000 hPa
};

} // namespace si
