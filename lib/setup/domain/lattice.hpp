#pragma once

#include <compare>
#include <cstdint>

/// @file lattice.hpp
/// @brief Lattice sizing: device memory per cell, grids that fit a memory budget, cell spans (internal).

/// @brief Device memory per lattice cell.
///
/// It depends on the example's configuration (velocity set, FP16 compression, extensions), so the simulation layer
/// passes in the core's bytes_per_cell_device(), and 2 dimensions for D2Q9.
struct LatticeMemory {
    std::uint32_t bytes_per_cell;   ///< bytes of device memory per cell
    std::uint32_t dimensions = 3u;  ///< 3, or 2 for D2Q9
};

/// A grid's cells along each axis.
struct GridSize {
    std::uint32_t x = 1u; ///< cells along X
    std::uint32_t y = 1u; ///< cells along Y
    std::uint32_t z = 1u; ///< cells along Z

    /// @brief Three-way comparison of two grids.
    /// @return the ordering
    constexpr auto operator<=>(const GridSize&) const = default;
};

/// @brief The whole cells [begin, end) along one axis that an interval covers.
///
/// Both bounds are rounded to the nearest cell boundary (cell i spans i to i+1 cell sizes from the origin), as the
/// original examples convert lengths with to_uint(units.x()), and clamped to the axis. So a length equal to the domain
/// size covers every cell even when the grid was rounded from it.
struct CellSpan {
    std::uint32_t begin = 0u; ///< first cell
    std::uint32_t end = 0u;   ///< one past the last cell

    /// @brief The cells an interval covers.
    /// @param from, to  the interval in cells (lattice units), e.g. units.x(metres)
    /// @param cells     the number of cells along the axis
    /// @return the span
    static constexpr CellSpan of(float from, float to, std::uint32_t cells) {
        return { boundary(from, cells), boundary(to, cells) };
    }

    /// @param i a cell
    /// @return whether the cell is in the span
    constexpr bool contains(std::uint32_t i) const { return i >= begin && i < end; }

    /// @brief Three-way comparison of two spans.
    /// @return the ordering
    constexpr auto operator<=>(const CellSpan&) const = default;

private:
    /// @brief The cell boundary nearest to a position.
    /// @param position the position in cells
    /// @param cells the number of cells along the axis
    /// @return the boundary, within 0 to cells
    static constexpr std::uint32_t boundary(float position, std::uint32_t cells) {
        return position <= 0.0f ? 0u : position >= static_cast<float>(cells) ? cells : static_cast<std::uint32_t>(position + 0.5f);
    }
};

/// @brief The lattice density at a height under a force per volume toward lower heights, with the density 1 at the
/// surface height (the core's Units::rho_hydrostatic(): the lattice pressure is (rho-1)/3).
/// @param force_per_volume the force per volume in lattice units, toward lower heights
/// @param height the height in cells
/// @param surface_height the height in cells at which the density is 1
/// @return the lattice density
inline float hydrostatic_density(float force_per_volume, float height, float surface_height) {
    return 3.0f * force_per_volume * (surface_height - height) + 1.0f;
}

/// @brief The grid with a given aspect ratio that fills a memory budget, as the core's resolution() (same float operations).
/// @param aspect_x, aspect_y, aspect_z the sides' proportions
/// @param budget_mb the device memory to fill, in MB
/// @param lattice the memory per cell
/// @return the grid
GridSize grid_for_memory(float aspect_x, float aspect_y, float aspect_z, std::uint32_t budget_mb, LatticeMemory lattice);

/// @brief The device memory a grid needs.
/// @param grid the grid
/// @param lattice the memory per cell
/// @return whole MB
std::uint64_t required_memory_mb(GridSize grid, LatticeMemory lattice);
