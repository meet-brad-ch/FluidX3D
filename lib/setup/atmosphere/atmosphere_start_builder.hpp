#pragma once

/// @file atmosphere_start_builder.hpp
/// @brief The middle layer's (L2) builder of an atmosphere's start state. Header-side: it needs the build's defines
/// and the core (lbm.hpp).

#include "setup/atmosphere/atmosphere_grid.hpp"
#include "setup/atmosphere/sounding.hpp"
#include "setup/atmosphere/start_noise.hpp"
#include "setup/atmosphere/wind_target.hpp"
#include "setup/core/boussinesq_buoyancy.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"

#include "lbm.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <optional>

#ifndef TEMPERATURE
#error "setup/atmosphere/atmosphere_start_builder.hpp needs TEMPERATURE in the EXTENSIONS"
#endif // TEMPERATURE

/// @brief Writes the start state of an atmosphere's box into the core's host buffers, before the first run.
///
/// Middle layer (L2). A temporary: make it with AtmosphereSimulation::start(), set its parts, call apply() once. It
/// refers to the LBM, which must outlive it. apply() writes the host buffers flags (the floor and the lid TYPE_S), T
/// (the sounding plus the start noise and an optional excess), Q (the sounding's total water, in a moist start), u
/// (the wind target) and, with balanced_density(), rho. The core sends the host buffers to the device at its first
/// run. One serial loop over z, then y, then x: the noise draws are the same for any thread count.
///
/// @code
/// using namespace si::symbols;
/// atmosphere.start().sounding(sounding).noise(StartNoise::per_cell(0.1 * K, 300.0 * m, 2u)).apply();
/// @endcode
class AtmosphereStartBuilder {
public:
    /// @brief Makes a builder with no sounding, no noise, no wind, a dry start and a uniform density.
    /// @param lbm the core, before its first run; it must outlive the builder
    /// @param grid the box's cells and levels (the layout of the host buffers)
    /// @param converter the run's conversions: its temperature unit converts the sounding's excess over the
    /// sounding's own theta_0 (Sounding::theta_excess()), so its theta_0 must be the sounding's (checked by apply())
    AtmosphereStartBuilder(LBM& lbm, const AtmosphereGrid& grid, const si::LatticeConverter& converter) : lbm_(lbm), grid_(grid), converter_(converter) {}

    /// @brief Sets the start profile that every column gets (required).
    /// @param sounding the start profile of theta_l (and of the total water in a moist start)
    /// @return this builder
    AtmosphereStartBuilder& sounding(Sounding sounding) {
        sounding_ = std::move(sounding);
        return *this;
    }
    /// @brief Sets the random perturbation of the start temperature.
    /// @param noise the start noise (default StartNoise::none())
    /// @return this builder
    AtmosphereStartBuilder& noise(StartNoise noise) {
        noise_ = noise;
        return *this;
    }
    /// @brief Sets the start wind: each fluid cell gets the target's horizontal wind at its height.
    /// @param target the wind profile, or nullptr for air at rest (the default)
    /// @return this builder
    AtmosphereStartBuilder& wind(std::shared_ptr<const WindTarget> target) {
        wind_ = std::move(target);
        return *this;
    }
    /// @brief Makes the start moist or dry.
    /// @param moist true: Q gets the sounding's total water, also in the floor and the lid cells (the first stream
    /// step carries their water into the fluid); false: Q is 1 (no water). A moist start needs MOISTURE and a sounding
    /// with total water (checked by apply()).
    /// @return this builder
    AtmosphereStartBuilder& moisture(bool moist) {
        moist_ = moist;
        return *this;
    }
    /// @brief Adds a temperature excess per fluid cell on top of the sounding and the noise (a thermal, a bubble).
    /// @param excess a function of the cell's centre that gives its theta_l excess; it replaces an earlier one
    /// @return this builder
    AtmosphereStartBuilder& add_excess(std::function<si::TemperatureDifference(const CellCentre&)> excess) {
        excess_ = std::move(excess);
        return *this;
    }
    /// @brief Starts the density in hydrostatic balance with the buoyancy's carrier force, in place of a uniform
    /// density 1.
    ///
    /// rho = 1 + 3 fz (z - the middle level), so p = rho / 3 and dp/dz = fz, with the mean 1. A uniform start makes
    /// the column's lowest sound mode ring, with an amplitude of about fz times a quarter of its period.
    /// @param buoyancy the run's buoyancy (its lattice carrier force fz)
    /// @return this builder
    AtmosphereStartBuilder& balanced_density(const BoussinesqBuoyancy& buoyancy) {
        const float force = buoyancy.lattice_force();
        carrier_force_ = force;
        return *this;
    }

    /// @brief Writes the start state into the core's host buffers (see the class description).
    /// @throws SetupError when no sounding is set, when a moist start has no MOISTURE in the build or no total water
    /// in the sounding, when the noise's blocks are not a whole number of cells (StartNoise::sequence()), when a
    /// tabulated sounding has no theta_0 (Sounding::theta_excess()), or when the converter has no theta_0 or another
    /// theta_0 than the sounding (Sounding::require_reference())
    void apply() {
        if(!sounding_.has_value()) {
            throw SetupError("AtmosphereStartBuilder: no sounding");
        }
        const Sounding& sounding = *sounding_;
        check(sounding);
        StartNoise::Sequence draws = noise_.sequence(grid_.cell());
        const std::uint32_t nz{ grid_.levels().levels() };
        for(std::uint32_t z = 0u; z < nz; z++) {
            write_level(sounding, z, draws);
        }
    }

private:
    /// @brief Checks the parts before anything is written.
    /// @param sounding the start profile
    /// @throws SetupError when a moist start has no MOISTURE in the build or no total water in the sounding, or the
    /// converter's theta_0 is missing or not the sounding's
    void check(const Sounding& sounding) const {
#ifndef MOISTURE
        if(moist_) {
            throw SetupError("AtmosphereStartBuilder: a moist start needs MOISTURE in the EXTENSIONS");
        }
#endif // MOISTURE
        const bool dry_sounding = !sounding.has_water();
        if(moist_ && dry_sounding) {
            throw SetupError("AtmosphereStartBuilder: a moist start needs total water in the sounding");
        }
        const si::Temperature lattice_theta_0 = converter_.theta_0();
        sounding.require_reference(lattice_theta_0);
    }

    /// @brief Gives the start density of a level in hydrostatic balance (balanced_density()).
    /// @param z the level, from 0 at the floor cell
    /// @param force the buoyancy's lattice carrier force fz
    /// @return rho = 1 + 3 fz (z - the middle level), in lattice units
    float balanced_rho(std::uint32_t z, double force) const {
        constexpr double half = 0.5;          // the middle level
        constexpr double ramp_per_level = 3.0; // rho = 1 + 3 fz (z - middle): p = rho / 3, dp/dz = fz
        const std::uint32_t top_level{ grid_.levels().levels() - 1u };
        const double top{ static_cast<double>(top_level) };
        const double middle{ half * top };
        const double ramp{ ramp_per_level * force };
        const double level{ static_cast<double>(z) };
        const double offset{ level - middle };
        const double rise{ ramp * offset };
        const double rho{ 1.0 + rise };
        const float rho_lattice{ static_cast<float>(rho) };
        return rho_lattice;
    }

    /// @brief Writes the cells of one level, in the order y, then x (the order of the noise draws).
    /// @param sounding the start profile
    /// @param z the level, from 0 at the floor cell
    /// @param draws the start noise, used in the cells' order
    /// @throws SetupError when a tabulated sounding has no theta_0 (Sounding::theta_excess())
    void write_level(const Sounding& sounding, std::uint32_t z, StartNoise::Sequence& draws) {
        const LevelGrid& levels = grid_.levels();
        const std::uint32_t nx{ grid_.nx() };
        const std::uint32_t ny{ grid_.ny() };
        if(carrier_force_.has_value()) {
            const double force{ static_cast<double>(*carrier_force_) };
            const float rho{ balanced_rho(z, force) };
            write_level_density(z, rho);
        }
        const bool below_fluid{ z < levels.first_fluid() };
        const bool above_fluid{ z > levels.last_fluid() };
        const bool wall_level{ below_fluid || above_fluid };
        const ulong z_wide{ static_cast<ulong>(z) };
        const ulong ny_wide{ static_cast<ulong>(ny) };
        const ulong nx_wide{ static_cast<ulong>(nx) };
        const ulong level_rows{ z_wide * ny_wide };
        for(std::uint32_t y = 0u; y < ny; y++) {
            const ulong y_wide{ static_cast<ulong>(y) };
            const ulong row{ y_wide + level_rows };
            const ulong row_start{ row * nx_wide };
            for(std::uint32_t x = 0u; x < nx; x++) {
                const ulong x_wide{ static_cast<ulong>(x) };
                const ulong n{ x_wide + row_start };
                const CellCentre centre = grid_.centre(x, y, z);
                write_cell(sounding, n, centre, draws, wall_level);
            }
        }
    }

    /// @brief Writes one density into every cell of a level (balanced_density()).
    /// @param z the level, from 0 at the floor cell
    /// @param rho the level's lattice density
    void write_level_density(std::uint32_t z, float rho) {
        const ulong z_wide{ static_cast<ulong>(z) };
        const ulong nx_wide{ static_cast<ulong>(grid_.nx()) };
        const ulong ny_wide{ static_cast<ulong>(grid_.ny()) };
        const ulong level_cells{ nx_wide * ny_wide };
        const ulong first{ z_wide * level_cells };
        const ulong end{ first + level_cells };
        for(ulong n = first; n < end; n++) {
            lbm_.rho[n] = rho;
        }
    }

    /// @brief Writes one cell: a wall cell on the floor and the lid levels, else a fluid cell.
    /// @param sounding the start profile
    /// @param n the cell's linear index
    /// @param centre the cell's centre
    /// @param draws the start noise, used in the cells' order
    /// @param wall true on a floor or lid level
    /// @throws SetupError when a tabulated sounding has no theta_0 (Sounding::theta_excess())
    void write_cell(const Sounding& sounding, ulong n, const CellCentre& centre, StartNoise::Sequence& draws, bool wall) {
        if(wall) {
            wall_cell(sounding, n, centre);
        } else {
            fluid_cell(sounding, n, centre, draws);
        }
    }

    /// @brief Writes a floor or lid cell: solid (TYPE_S); in a moist start it also holds the total water of the
    /// nearest fluid height (its height clamped to the fluid levels, as the example), which the first stream step
    /// carries into the fluid.
    /// @param sounding the start profile (read only in a MOISTURE build)
    /// @param n the cell's linear index
    /// @param centre the cell's centre (read only in a MOISTURE build)
    void wall_cell([[maybe_unused]] const Sounding& sounding, ulong n, [[maybe_unused]] const CellCentre& centre) {
        lbm_.flags[n] = TYPE_S;
#ifdef MOISTURE
        if(moist_) {
            const LevelGrid& levels = grid_.levels();
            const si::Length fluid_top = levels.height(levels.last_fluid());
            const si::Length height = au::clamp(centre.height, au::meters(0.0), fluid_top);
            const si::WaterContent water = sounding.total_water(height);
            const float q = converter_.moisture(water);
            lbm_.Q[n] = q;
        }
#endif // MOISTURE
    }

    /// @brief Writes a fluid cell: T from the sounding, the noise and the excess; Q from the total water (1 when
    /// dry); u from the wind target.
    /// @param sounding the start profile
    /// @param n the cell's linear index
    /// @param centre the cell's centre
    /// @param draws the start noise, used in the cells' order
    /// @throws SetupError when a tabulated sounding has no theta_0 (Sounding::theta_excess())
    void fluid_cell(const Sounding& sounding, ulong n, const CellCentre& centre, StartNoise::Sequence& draws) {
        const si::TemperatureDifference sounding_theta = sounding.theta_excess(centre.height);
        const si::TemperatureDifference draw = draws.at(centre);
        si::TemperatureDifference theta = sounding_theta + draw;
        if(excess_) {
            // The user's function, one indirect call per fluid cell: this loop runs once, before the first run step.
            const si::TemperatureDifference excess = excess_(centre);
            theta += excess;
        }
        const float t = converter_.temperature_excess(theta);
        lbm_.T[n] = t;
#ifdef MOISTURE
        float q{ 1.0f }; // dry: Q 1, no water
        if(moist_) {
            const si::WaterContent water = sounding.total_water(centre.height);
            q = converter_.moisture(water);
        }
        lbm_.Q[n] = q;
#endif // MOISTURE
        if(wind_) {
            const HorizontalWind w = wind_->at(centre.height);
            const float ux = converter_.velocity(w.x);
            const float uy = converter_.velocity(w.y);
            lbm_.u.x[n] = ux;
            lbm_.u.y[n] = uy;
        }
    }

    LBM& lbm_;                                  ///< the core, whose host buffers apply() writes
    AtmosphereGrid grid_;                       ///< the box's cells and levels
    si::LatticeConverter converter_;            ///< the run's conversions
    std::optional<Sounding> sounding_{};        ///< the start profile, or none until sounding() is called
    StartNoise noise_ = StartNoise::none();     ///< the random perturbation of the start temperature
    std::shared_ptr<const WindTarget> wind_{};  ///< the start wind profile, or nullptr for air at rest
    bool moist_ = false;                        ///< true when Q gets the sounding's total water
    std::function<si::TemperatureDifference(const CellCentre&)> excess_{}; ///< the theta_l excess per fluid cell, or empty
    std::optional<float> carrier_force_{};      ///< the buoyancy's lattice carrier force fz (balanced_density()), or none for a uniform density
};
