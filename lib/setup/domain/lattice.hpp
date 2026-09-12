#pragma once

#include <compare>
#include <cstdint>

// Device memory per lattice cell. It depends on the example's defines.hpp (velocity set, FP16 compression,
// extensions), so the simulation layer passes in the core's bytes_per_cell_device(), and 2 dimensions for D2Q9.
struct LatticeMemory {
    std::uint32_t bytes_per_cell;
    std::uint32_t dimensions = 3u;
};

struct GridSize { // cells
    std::uint32_t x = 1u, y = 1u, z = 1u;
    constexpr auto operator<=>(const GridSize&) const = default;
};

/// @brief The whole cells [begin, end) along one axis that an interval covers.
///
/// Both bounds are truncated to whole cells (cell i spans i to i+1 cell sizes from the origin), as the examples
/// convert lengths with (uint)units.x(). A bound within the last cell reaches the end of the axis, so a length equal
/// to the domain size covers every cell even when the grid was rounded down from it.
struct CellSpan {
    std::uint32_t begin = 0u; ///< first cell
    std::uint32_t end = 0u;   ///< one past the last cell

    /// @param from, to  the interval in cells (lattice units), e.g. units.x(metres)
    /// @param cells     the number of cells along the axis
    static constexpr CellSpan of(float from, float to, std::uint32_t cells) {
        return { from <= 0.0f ? 0u : from >= static_cast<float>(cells) ? cells : static_cast<std::uint32_t>(from),
                 to <= 0.0f ? 0u : to >= static_cast<float>(cells - 1u) ? cells : static_cast<std::uint32_t>(to) };
    }

    /// Whether cell i is in the span.
    constexpr bool contains(std::uint32_t i) const { return i >= begin && i < end; }

    constexpr auto operator<=>(const CellSpan&) const = default;
};

// grid with the given aspect ratio that fills budget_mb, as the core's resolution() (same float operations)
GridSize grid_for_memory(float aspect_x, float aspect_y, float aspect_z, std::uint32_t budget_mb, LatticeMemory lattice);

// whole MB of device memory that a grid needs
std::uint64_t required_memory_mb(GridSize grid, LatticeMemory lattice);
