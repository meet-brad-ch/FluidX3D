#include "setup/atmosphere/atmosphere_grid.hpp"
#include "setup/core/setup_error.hpp"

#include <cmath>

AtmosphereGrid AtmosphereGrid::of(si::Length east, si::Length north, si::Length ground_to_lid, si::Length cell) {
    if(!(cell > au::ZERO)) {
        throw SetupError("AtmosphereGrid: the cell size must be more than 0 m");
    }
    const auto east_ratio = east / cell;
    const double east_cells{ au::as_raw_number(east_ratio) };
    const long nx{ std::lround(east_cells) };
    const auto north_ratio = north / cell;
    const double north_cells{ au::as_raw_number(north_ratio) };
    const long ny{ std::lround(north_cells) };
    if(nx < 1 || ny < 1) {
        throw SetupError("AtmosphereGrid: the box must be at least one cell across");
    }
    const std::uint32_t level_count{ LevelGrid::levels_for(ground_to_lid, cell) };
    const LevelGrid levels(level_count, cell);
    const std::uint32_t nx_cells{ static_cast<std::uint32_t>(nx) };
    const std::uint32_t ny_cells{ static_cast<std::uint32_t>(ny) };
    const AtmosphereGrid grid(nx_cells, ny_cells, levels);
    return grid;
}

CellCentre AtmosphereGrid::centre(std::uint32_t x, std::uint32_t y, std::uint32_t z) const {
    constexpr double cell_centre = 0.5; // a cell's centre, in cells from its face
    const si::Length cell_size{ cell() };
    const double x_double{ static_cast<double>(x) };
    const double y_double{ static_cast<double>(y) };
    const double x_cells{ x_double + cell_centre };
    const double y_cells{ y_double + cell_centre };
    const si::Length east{ cell_size * x_cells };
    const si::Length north{ cell_size * y_cells };
    const si::Length height{ levels_.height(z) };
    const CellCentre centre_position{ .east = east, .north = north, .height = height };
    return centre_position;
}
