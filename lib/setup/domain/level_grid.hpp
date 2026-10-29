#pragma once

#include "setup/units/units.hpp"

#include <cstdint>

/// @brief The horizontal levels of an atmospheric column: the cells along z, the solid floor below the first fluid
/// level and the solid lid above the last, and the height of each level's centre above the floor's top.
///
/// The floor and the lid are bounce-back walls halfway between a solid and a fluid cell, so with one floor cell the
/// first fluid level's centre is half a cell above the floor's top: height(z) = (z + 1/2 - floor) dx (the convective
/// boundary layer example's (z - 1/2) dx).
class LevelGrid {
public:
    /// @brief Makes the levels of a column from its cell count and its walls.
    /// @param levels the cells along z, the floor and lid cells included
    /// @param cell the cell size, more than 0
    /// @param floor_cells the solid cells at the bottom, at least 1
    /// @param lid_cells the solid cells at the top, at least 1
    /// @throws SetupError when the cell is not more than 0, a wall has no cell, or no fluid level is left between the
    /// walls
    LevelGrid(std::uint32_t levels, si::Length cell, std::uint32_t floor_cells = 1u, std::uint32_t lid_cells = 1u);

    /// @brief Gives the levels of a column whose fluid reaches a height: the height in cells, rounded to the
    /// nearest, plus one floor and one lid cell (the CBL example's lround(H / dx) + 2).
    /// @param ground_to_lid the column's height from the floor's top to the lid, more than 0
    /// @param cell the cell size, more than 0
    /// @return the levels, the floor and lid cells included: at least 3
    /// @throws SetupError when the height or the cell is not more than 0, or the height is under half a cell (no fluid
    /// level)
    static std::uint32_t levels_for(si::Length ground_to_lid, si::Length cell);

    /// @brief Gives the height of a level's centre above the floor's top: (z + 1/2 - floor cells) dx.
    /// @param z a level, counted from 0 at the bottom wall cell
    /// @return the height (negative inside the floor)
    si::Length height(std::uint32_t z) const {
        const double z_cells{ static_cast<double>(z) };
        const double floor_cells{ static_cast<double>(floor_) };
        const double centre_cells{ z_cells + cell_centre };
        const double above_floor{ centre_cells - floor_cells };
        const si::Length h{ cell_ * above_floor };
        return h;
    }

    /// @return the first fluid level (the level just above the floor cells), counted from 0
    std::uint32_t first_fluid() const { return floor_; }
    /// @brief Gives the last fluid level: the level just below the lid cells.
    /// @return the level, counted from 0
    std::uint32_t last_fluid() const {
        const std::uint32_t top{ levels_ - 1u };
        const std::uint32_t last{ top - lid_ };
        return last;
    }
    /// @brief Gives the number of fluid levels between the floor and the lid cells.
    /// @return the levels minus the floor and lid cells, at least 1
    std::uint32_t fluid_levels() const {
        const std::uint32_t above_floor{ levels_ - floor_ };
        const std::uint32_t fluid{ above_floor - lid_ };
        return fluid;
    }
    /// @return the cells along z, the floor and lid cells included
    std::uint32_t levels() const { return levels_; }
    /// @return the cell size
    si::Length cell() const { return cell_; }
    /// @brief Gives the height of the top cell's centre: the lid height that the CBL example's sponge profile uses
    /// (half a cell above the lid wall with one lid cell).
    /// @return the height above the floor's top
    si::Length lid_height() const {
        const std::uint32_t top{ levels_ - 1u };
        const si::Length lid{ height(top) };
        return lid;
    }

    /// @brief Gives the fluid level whose centre is nearest to a height.
    /// @param h a height above the floor's top
    /// @return the level, counted from 0; a height below the first or above the last fluid level gives that level
    std::uint32_t nearest_level(si::Length h) const;

private:
    static constexpr double cell_centre = 0.5; ///< the position of a cell's centre, in cells above its lower face

    std::uint32_t levels_{};       ///< the cells along z, the floor and lid cells included
    si::Length cell_{ au::ZERO }; ///< the cell size
    std::uint32_t floor_{};        ///< the solid cells at the bottom
    std::uint32_t lid_{};          ///< the solid cells at the top
};
