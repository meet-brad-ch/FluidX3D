#include "setup/domain/level_grid.hpp"
#include "setup/core/setup_error.hpp"

#include <cmath>

LevelGrid::LevelGrid(std::uint32_t levels, si::Length cell, std::uint32_t floor_cells, std::uint32_t lid_cells)
    : levels_(levels), cell_(cell), floor_(floor_cells), lid_(lid_cells) {
    if(!(cell > au::ZERO)) {
        throw SetupError("LevelGrid: the cell size must be more than 0");
    }
    if(floor_cells == 0u || lid_cells == 0u) {
        throw SetupError("LevelGrid: the column needs a floor and a lid cell");
    }
    const std::uint32_t wall_cells{ floor_cells + lid_cells };
    if(levels <= wall_cells) {
        throw SetupError("LevelGrid: no fluid level between the floor and the lid");
    }
}

std::uint32_t LevelGrid::levels_for(si::Length ground_to_lid, si::Length cell) {
    const bool height_positive{ ground_to_lid > au::ZERO };
    const bool cell_positive{ cell > au::ZERO };
    if(!height_positive || !cell_positive) {
        throw SetupError("LevelGrid::levels_for(): the height and the cell must be more than 0");
    }
    const auto ratio = ground_to_lid / cell;
    const double height_cells{ au::as_raw_number(ratio) };
    const long rounded{ std::lround(height_cells) };
    const std::uint32_t fluid_cells{ static_cast<std::uint32_t>(rounded) };
    if(fluid_cells == 0u) {
        throw SetupError("LevelGrid::levels_for(): the column is under half a cell high: no fluid level");
    }
    const std::uint32_t level_count{ fluid_cells + 2u }; // the floor and the lid
    return level_count;
}

std::uint32_t LevelGrid::nearest_level(si::Length h) const {
    const auto ratio = h / cell_;
    const double h_cells{ au::as_raw_number(ratio) };
    const double below_centre{ h_cells - cell_centre };
    const double floor_cells{ static_cast<double>(floor_) };
    const double z_exact{ below_centre + floor_cells };
    const long z{ std::lround(z_exact) };
    const std::uint32_t first{ first_fluid() };
    const std::uint32_t last{ last_fluid() };
    const long first_long{ static_cast<long>(first) };
    const long last_long{ static_cast<long>(last) };
    std::uint32_t level{};
    if(z < first_long) {
        level = first;
    } else if(z > last_long) {
        level = last;
    } else {
        level = static_cast<std::uint32_t>(z);
    }
    return level;
}
