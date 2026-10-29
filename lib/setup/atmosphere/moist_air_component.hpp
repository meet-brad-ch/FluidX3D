#pragma once

/// @file moist_air_component.hpp
/// @brief The middle layer's (L2) moist air of an atmosphere (MOISTURE). Header-side: it needs the build's defines
/// and the core (lbm.hpp).

#include "setup/atmosphere/moist_column.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"

#include "lbm.hpp"

#include <cstddef>
#include <cstdio>
#include <string>

#if !defined(MOISTURE) || !defined(LEVEL_FORCING)
#error "setup/atmosphere/moist_air_component.hpp needs MOISTURE and LEVEL_FORCING in the EXTENSIONS"
#endif // MOISTURE && LEVEL_FORCING

/// @brief The moist air's terms in the core: per level the hydrostatic pressure and the start sounding's theta_v (the
/// moist buoyancy's reference), theta_0 (the theta_l that the lattice temperature 1 stands for) and the floor's
/// moisture source.
///
/// Middle layer (L2). AtmosphereSimulation owns it for a moist run's lifetime; apply() writes the core's level
/// buffer (pressure and theta_v), theta_0 and the moisture rate once, before the first run.
class MoistAir {
public:
    /// @brief Makes the moist air of a run.
    /// @param lbm the core; it must outlive this object
    /// @param column the moist column of the start sounding (the pressure and theta_v of each level); taken over
    /// @param flux the floor's kinematic moisture flux
    /// @param converter the run's conversions; it must have theta_0
    /// @throws SetupError when the converter has no temperature scale (LatticeConverter::theta_0())
    MoistAir(LBM& lbm, MoistColumn column, si::MoistureFlux flux, const si::LatticeConverter& converter)
        : lbm_(lbm), column_(std::move(column)), theta_0_(converter.theta_0()), flux_(flux), rate_(converter.moisture_rate(flux)) {}

    /// @brief Gives the terms to the core: LBM::set_level_moist(), set_theta_0() and set_moisture_rate().
    ///
    /// The core ends the program with print_error() when the column does not have one value per level. A build
    /// without MOISTURE and LEVEL_FORCING does not compile this header (LBM::set_level_moist() needs both).
    void apply() {
        const std::vector<float> pressure = column_.core_pressure();
        const std::vector<float> theta_v = column_.core_theta_v();
        lbm_.set_level_moist(pressure, theta_v);
        const double theta_0_kelvin = si::in_kelvin(theta_0_);
        const float lattice_theta_0 = static_cast<float>(theta_0_kelvin);
        lbm_.set_theta_0(lattice_theta_0);
        lbm_.set_moisture_rate(rate_);
    }

    /// @return the moist column of the start sounding
    const MoistColumn& column() const { return column_; }
    /// @return the lattice moisture that a floor source cell gains per time step
    float moisture_rate() const { return rate_; }
    /// @return the floor's kinematic moisture flux
    si::MoistureFlux flux() const { return flux_; }
    /// @brief Describes the moist set-up for the run's log.
    /// @param floor_qt the total water at the first fluid level
    /// @return the MOIST line: the floor's qt in g/kg, the flux, the lattice rate, and the pressure at level 1 (the
    /// first fluid level) and at the lid
    /// @throws std::out_of_range from std::vector::at() when the column has fewer than two levels
    /// @throws SetupError when the MOIST line does not fit its buffer
    std::string describe(si::WaterContent floor_qt) const {
        const double floor_qt_g_per_kg = floor_qt.in(au::milli(si::kilograms_of_water) / si::kilograms);
        const double flux = flux_.in(si::kilograms_of_water / si::kilograms * au::meters / au::second);
        const double rate{ static_cast<double>(rate_) };
        const double floor_pressure = column_.pressure(1u).in(au::pascals);
        const std::uint32_t lid_level = column_.levels() - 1u;
        const double lid_pressure = column_.pressure(lid_level).in(au::pascals);
        constexpr std::size_t text_capacity{ 240u };
        char text[text_capacity]{};
        const int length = std::snprintf(text, sizeof(text), "MOIST qt %.2f g/kg at the floor, the moisture flux %.3e kg/kg m/s, rate %.4e Q per step; %.0f Pa at the floor, %.0f Pa at the lid",
                      floor_qt_g_per_kg, flux, rate, floor_pressure, lid_pressure);
        const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
        if(cut) {
            throw SetupError("MoistAir::describe(): the MOIST line does not fit its buffer");
        }
        const std::string line = text;
        return line;
    }

private:
    LBM& lbm_;                                              ///< the core, whose moist terms apply() writes
    MoistColumn column_;                                    ///< the moist column of the start sounding
    si::Temperature theta_0_{ si::kelvin_point(0.0) };      ///< the potential temperature of the lattice temperature 1
    si::MoistureFlux flux_{ au::ZERO };                     ///< the floor's kinematic moisture flux
    float rate_{};                                          ///< the lattice moisture per time step and source cell
};
