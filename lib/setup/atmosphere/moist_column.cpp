#include "setup/atmosphere/moist_column.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/units/moist_air_physics.hpp"

#include <algorithm>
#include <string>

namespace {

/// @brief Gives a level's height for the sounding: a floor cell, below the floor's top, gets the ground's values.
/// @param h a level's centre height above the floor's top (negative inside the floor)
/// @return h, or 0 m for a negative h
si::Length above_ground(si::Length h) {
    const si::Length ground{ au::meters(0.0) };
    const si::Length height{ au::max(h, ground) };
    return height;
}

} // namespace

MoistColumn MoistColumn::hydrostatic(const Sounding& sounding, const LevelGrid& grid, si::Pressure ground) {
    if(!sounding.has_water()) {
        throw SetupError("MoistColumn: the sounding has no total water");
    }
    if(!(ground > au::ZERO)) {
        throw SetupError("MoistColumn: the ground pressure must be more than 0");
    }
    const si::Temperature theta_0{ sounding.reference() };
    const std::uint32_t level_count{ grid.levels() };
    std::vector<si::Pressure> pressure(level_count, au::pascals(0.0));
    std::vector<si::Temperature> theta_v(level_count, si::kelvin_point(0.0));
    si::Pressure p{ ground };
    si::Length below{ au::meters(0.0) };
    for(std::uint32_t z = 0u; z < level_count; z++) { // the example's set_up_moist(): one step per level
        const si::Length level_height{ grid.height(z) };
        const si::Length h{ above_ground(level_height) };
        const si::TemperatureDifference excess{ sounding.theta_excess(h) };
        const si::Temperature thl{ theta_0 + excess };
        const si::WaterContent qt{ sounding.total_water(h) };
        if(h > below) {
            const si::Length rise{ h - below };
            p = si::MoistAirPhysics::hydrostatic_step(p, thl, qt, rise);
            below = h;
        }
        pressure[z] = p;
        theta_v[z] = si::MoistAirPhysics::virtual_theta(thl, qt, p);
    }
    MoistColumn column(std::move(pressure), std::move(theta_v));
    return column;
}

MoistColumn MoistColumn::isentropic(si::Temperature theta, si::WaterContent total_water, const LevelGrid& grid) {
    const double theta_k{ si::in_kelvin(theta) };
    if(!(theta_k > 0.0)) {
        throw SetupError("MoistColumn::isentropic(): the temperature must be more than 0 K");
    }
    const std::uint32_t level_count{ grid.levels() };
    std::vector<si::Pressure> pressure(level_count, au::pascals(0.0));
    std::vector<si::Temperature> theta_v(level_count, si::kelvin_point(0.0));
    for(std::uint32_t z = 0u; z < level_count; z++) {
        const si::Length level_height{ grid.height(z) };
        const si::Length h{ above_ground(level_height) };
        const si::Pressure level_pressure{ si::MoistAirPhysics::isentropic_pressure(h, theta, si::PhysicalConstants::ground_pressure) };
        pressure[z] = level_pressure;
        theta_v[z] = si::MoistAirPhysics::virtual_theta(theta, total_water, level_pressure);
    }
    MoistColumn column(std::move(pressure), std::move(theta_v));
    return column;
}

std::vector<float> MoistColumn::core_pressure() const {
    std::vector<float> p(pressure_.size(), 0.0f);
    for(std::size_t z = 0u; z < p.size(); z++) {
        const double pascals{ pressure_[z].in(au::pascals) };
        p[z] = static_cast<float>(pascals);
    }
    return p;
}

std::vector<float> MoistColumn::core_theta_v() const {
    std::vector<float> t(theta_v_.size(), 0.0f);
    for(std::size_t z = 0u; z < t.size(); z++) {
        const double kelvin{ si::in_kelvin(theta_v_[z]) };
        t[z] = static_cast<float>(kelvin);
    }
    return t;
}

void MoistColumn::require_levels(std::uint32_t box_levels) const {
    const std::uint32_t column_levels{ levels() };
    if(column_levels < box_levels) {
        const std::string column_text{ std::to_string(column_levels) };
        const std::string box_text{ std::to_string(box_levels) };
        throw SetupError("MoistColumn: the column has " + column_text + " levels, the box " + box_text);
    }
}

si::WaterContent MoistColumn::liquid(si::Temperature theta_l, si::WaterContent total_water, std::uint32_t z) const {
    const si::Pressure level_pressure{ pressure_.at(z) };
    const si::WaterContent liquid_water{ si::MoistAirPhysics::liquid(theta_l, total_water, level_pressure) };
    return liquid_water;
}
