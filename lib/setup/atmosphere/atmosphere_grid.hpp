#pragma once

#include "setup/atmosphere/start_noise.hpp"
#include "setup/domain/lattice.hpp"
#include "setup/domain/level_grid.hpp"

#include <cstdint>

/// @file atmosphere_grid.hpp
/// @brief The cells of an atmosphere's box: AtmosphereGrid.

/// @brief The cells of an atmosphere's box: cubic cells, nx by ny across, and the column's levels from the solid
/// floor to the solid lid (LevelGrid).
///
/// A value made by of() from the box in metres. x is west to east, y is south to north, z is the level (0 is the
/// floor cell). CblDescription::grid() makes the grid of a description, and the atmosphere classes read their cell
/// counts and heights from it.
/// @code
/// using namespace si::symbols;
/// const AtmosphereGrid grid = AtmosphereGrid::of(1280.0 * m, 1280.0 * m, 2048.0 * m, 10.0 * m);
/// // 128 x 128 cells across; 205 fluid levels plus the floor and the lid
/// @endcode
class AtmosphereGrid {
public:
    /// @brief Makes the grid of a box: each extent and the lid height in whole cells, rounded to the nearest.
    /// @param east the box's extent west to east (it must round to at least one cell)
    /// @param north the box's extent south to north (it must round to at least one cell)
    /// @param ground_to_lid the lid's height above the floor's top, more than 0 (it must round to at least one fluid
    /// level)
    /// @param cell the edge of one cubic cell, more than 0
    /// @return the grid: lround(extent / cell) cells across, and LevelGrid::levels_for(ground_to_lid, cell) levels
    /// (the fluid levels plus the floor and the lid)
    /// @throws SetupError when the cell size is not more than 0, an extent rounds to less than one cell, the height
    /// is not more than 0 (LevelGrid::levels_for()), or the height rounds to no fluid level (LevelGrid's constructor)
    static AtmosphereGrid of(si::Length east, si::Length north, si::Length ground_to_lid, si::Length cell);

    /// @return the number of cells west to east (along x)
    std::uint32_t nx() const { return nx_; }
    /// @return the number of cells south to north (along y)
    std::uint32_t ny() const { return ny_; }
    /// @return the levels of the column, the floor and the lid included
    const LevelGrid& levels() const { return levels_; }
    /// @return the edge of one cubic cell
    si::Length cell() const { return levels_.cell(); }
    /// @brief Gives the cell counts of the whole grid, for the domain planner and the run estimate.
    /// @return nx along x, ny along y, and the levels (the floor and the lid included) along z
    GridSize size() const {
        const std::uint32_t nz{ levels_.levels() };
        const GridSize grid{ .x = nx_, .y = ny_, .z = nz };
        return grid;
    }
    /// @brief Gives the position of a cell's centre. The indices are not checked against the grid.
    /// @param x the cell's index along x, from 0 at the west edge
    /// @param y the cell's index along y, from 0 at the south edge
    /// @param z the cell's level, from 0 at the floor cell
    /// @return the centre: (x + 1/2) cells east and (y + 1/2) cells north of the south-west corner, and the level's
    /// height above the floor's top (LevelGrid::height(): negative inside the floor)
    CellCentre centre(std::uint32_t x, std::uint32_t y, std::uint32_t z) const;

private:
    /// @brief Makes the grid from its counts. Use of(), which checks the values.
    /// @param nx the number of cells west to east, at least 1
    /// @param ny the number of cells south to north, at least 1
    /// @param levels the levels of the column
    AtmosphereGrid(std::uint32_t nx, std::uint32_t ny, LevelGrid levels) : nx_(nx), ny_(ny), levels_(levels) {}
    std::uint32_t nx_{}; ///< the number of cells west to east (along x)
    std::uint32_t ny_{}; ///< the number of cells south to north (along y)
    LevelGrid levels_; ///< the levels of the column, the floor and the lid included
};
