#pragma once

#include "setup/atmosphere/sounding.hpp"
#include "setup/domain/level_grid.hpp"
#include "setup/units/physical_constants.hpp"
#include "setup/units/units.hpp"

#include <cstdint>
#include <vector>

/// @file moist_column.hpp
/// @brief The moist air's per-level terms of a column: MoistColumn.

/// @brief The moist air's terms of a column, one per level: the hydrostatic pressure and the start sounding's
/// virtual potential temperature theta_v (the buoyancy's reference at each level), and the saturation adjustment at
/// a level's pressure (si::MoistAirPhysics, the host's copy of the kernel's model).
///
/// The level index z counts from 0 at the floor cell; the floor's cells, below the floor's top, get the ground's
/// values. MoistAir hands the pressures and theta_v to the core (LBM::set_level_moist()); the diagnostics use
/// liquid() for the cloud base.
///
/// hydrostatic() integrates the pressure from the ground up, one first-order step per level (the form of the
/// original CBL example): over the step from the level below to level z, T = theta_l (p / 1e5 Pa)^(Rd/cp)
/// (1 + 0.61 qt) is held, with p the lower level's pressure and theta_l and qt the sounding's values at level z (the
/// upper end of the step), and p is multiplied by exp(-g dh / (Rd T)). MicroHH evaluates T at the step's midpoint
/// (thermo_moist_functions.h, calc_base_state). On the moist sounding case the first-order step gives a pressure
/// 3.3 Pa above a fine reference at zi and 6.6 Pa above it at 3765 m (1.0e-4 relative). The cloud base moves about
/// 0.34 m. This is far below the 10 m cell, so the step is kept (owner, 2026-10-28).
class MoistColumn {
public:
    /// @brief Makes the column of a moist start sounding: the pressure integrated level by level from the ground.
    /// @param sounding the start state, with total water and theta_0
    /// @param grid the column's levels
    /// @param ground the pressure at the floor's top (h = 0), more than 0
    /// @return the column
    /// @throws SetupError when the sounding has no total water, the ground pressure is not more than 0, or the
    /// sounding has no theta_0 (Sounding::reference())
    static MoistColumn hydrostatic(const Sounding& sounding, const LevelGrid& grid, si::Pressure ground = si::PhysicalConstants::ground_pressure);

    /// @brief Makes a column of one potential temperature and one total water at every height (a saturated, neutral
    /// environment: Bryan and Fritsch 2002). The pressure is in closed form, p = p0 (1 - g h / (cp theta))^(cp / Rd)
    /// with p0 the standard ground pressure, and theta_v follows from the saturation adjustment at each level. A level
    /// above cp theta / g (about 30 km at 300 K) gets a pressure and a theta_v that are not numbers, without an error.
    /// @param theta the potential temperature theta_l at every height, more than 0 K
    /// @param total_water the total water qt at every height
    /// @param grid the column's levels
    /// @return the column
    /// @throws SetupError when the temperature is not more than 0 K
    static MoistColumn isentropic(si::Temperature theta, si::WaterContent total_water, const LevelGrid& grid);

    /// @brief Gives the pressure of a level.
    /// @param z the level, from 0 at the floor cell
    /// @return the pressure
    /// @throws std::out_of_range when z is not a level of the column
    si::Pressure pressure(std::uint32_t z) const { return pressure_.at(z); }
    /// @brief Gives the start sounding's virtual potential temperature at a level.
    /// @param z the level, from 0 at the floor cell
    /// @return theta_v
    /// @throws std::out_of_range when z is not a level of the column
    si::Temperature theta_v(std::uint32_t z) const { return theta_v_.at(z); }
    /// @brief Gives the pressures for the core, which takes floats in Pa (LBM::set_level_moist()).
    /// @return the pressure of each level (index z) in Pa
    std::vector<float> core_pressure() const;
    /// @brief Gives the virtual potential temperatures for the core, which takes floats in K (LBM::set_level_moist()).
    /// @return theta_v of each level (index z) in K
    std::vector<float> core_theta_v() const;

    /// @brief Gives the liquid water of air at a level after the saturation adjustment at that level's pressure.
    /// @param theta_l the air's liquid water potential temperature
    /// @param total_water the air's total water qt
    /// @param z the level, from 0 at the floor cell (it gives the pressure)
    /// @return the liquid water (0 for unsaturated air)
    /// @throws std::out_of_range when z is not a level of the column
    si::WaterContent liquid(si::Temperature theta_l, si::WaterContent total_water, std::uint32_t z) const;

    /// @brief Checks that the column has a level for each level of a box, so that liquid() does not throw for one.
    /// @param box_levels the box's levels, the floor and the lid included
    /// @throws SetupError that names both counts when the column has fewer levels than the box
    void require_levels(std::uint32_t box_levels) const;

    /// @brief Gives the number of levels of the column.
    /// @return the levels, the floor and the lid included
    std::uint32_t levels() const {
        const std::size_t level_count{ pressure_.size() };
        const std::uint32_t levels_u32{ static_cast<std::uint32_t>(level_count) };
        return levels_u32;
    }

private:
    /// @brief Makes the column from its values. Use hydrostatic() or isentropic().
    /// @param pressure the pressure of each level
    /// @param theta_v the virtual potential temperature of each level
    MoistColumn(std::vector<si::Pressure> pressure, std::vector<si::Temperature> theta_v) : pressure_(std::move(pressure)), theta_v_(std::move(theta_v)) {}
    std::vector<si::Pressure> pressure_{};   ///< the pressure of each level (index z)
    std::vector<si::Temperature> theta_v_{}; ///< the virtual potential temperature of each level (index z)
};
