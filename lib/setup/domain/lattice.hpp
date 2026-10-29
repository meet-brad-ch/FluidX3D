#pragma once

#include "setup/units/units.hpp"

#include <compare>
#include <cstdint>

/// @file lattice.hpp
/// @brief Lattice sizing: device memory per cell, grids that fit a memory budget, cell spans (internal).

/// A grid's number of cells along each axis.
struct GridSize {
    std::uint32_t x = 1u; ///< the cells along x
    std::uint32_t y = 1u; ///< the cells along y
    std::uint32_t z = 1u; ///< the cells along z

    /// @brief Compares two grids, axis by axis in the order x, y, z.
    /// @return the ordering
    constexpr auto operator<=>(const GridSize&) const = default;
};

/// @brief The device memory of one lattice cell, and the device's limit on one buffer.
///
/// It depends on the example's configuration (velocity set, FP16 compression, extensions), so the simulation layer
/// fills it from the core's bytes_per_cell_device() (memory_per_cell), with 2 dimensions for D2Q9. A device accepts each buffer up to a
/// limit (CL_DEVICE_MAX_MEM_ALLOC_SIZE, a quarter of the memory on NVIDIA cards), and the lattice's DDFs are one buffer
/// per domain: the simulation layer gives the DDFs' bytes per cell and the smallest limit of the devices, and the
/// planner splits a lattice whose DDF buffer would be over it (DomainPlanner::plan()). MB here are the core's MB,
/// 2^20 bytes.
struct LatticeMemory {
    si::MemoryAmount memory_per_cell{};        ///< the device memory per cell, all buffers together, whole bytes (more than 0 for a grid)
    std::uint32_t dimensions = 3u;             ///< the lattice's dimensions: 3, or 2 for D2Q9
    si::MemoryAmount buffer_memory_per_cell{}; ///< the memory per cell of the largest single buffer, the DDFs, whole bytes (0: not known)
    si::MemoryAmount max_buffer{};             ///< the largest buffer that a device accepts, whole MiB (0: no limit known)
    std::uint32_t halo_layers = 1u;            ///< the halo layers on each side of a domain cut (the core's halo_width(): 2 with SCALAR_FV)

    /// The bytes of one MB, the core's MB (2^20 bytes).
    static constexpr std::uint64_t bytes_per_mb = 1048576ull;

    /// @brief Gives the grid of a given aspect ratio that fills a memory budget, with the same float operations as
    /// the core's resolution(), so that both give the same grid.
    /// @param aspect_x the proportion of the box's side along x, more than 0
    /// @param aspect_y the proportion of the box's side along y, more than 0
    /// @param aspect_z the proportion of the box's side along z, more than 0 (not used for 2 dimensions)
    /// @param budget the device memory to fill, more than 0 (the core counts it in MB of 2^20 bytes, as a float)
    /// @return the grid: each side the aspect times one scale factor, rounded to the nearest cell (z is 1 for 2
    /// dimensions); dimensions 2 gives a 2D grid, any other value is taken as 3
    /// @throws SetupError when the budget is not more than 0 or the lattice's memory per cell is not more than 0
    GridSize grid_for_memory(float aspect_x, float aspect_y, float aspect_z, si::MemoryAmount budget) const;

    /// @brief Gives a memory amount of whole bytes as the core's integer count, for the core's integer arithmetic.
    /// @param amount a memory amount of whole bytes, 0 or more (the fields of a LatticeMemory)
    /// @return the bytes
    static std::uint64_t whole_bytes(si::MemoryAmount amount) {
        const double bytes{ amount.in(au::bytes) };
        const std::uint64_t count{ static_cast<std::uint64_t>(bytes) };
        return count;
    }

    /// @brief Gives the device memory that a grid needs, as the core counts it.
    /// @param grid the grid's cells along each axis
    /// @return the cells times memory_per_cell, in whole MiB (the core's MB), rounded down
    si::MemoryAmount required_memory(GridSize grid) const;

private:
    /// @brief Rounds a number of cells to the nearest whole cell, half up, as the core's to_uint().
    /// @param cells the number of cells
    /// @return the whole number, 0 or more (a negative number gives 0)
    static std::uint32_t round_to_cells(float cells);
};

/// @brief The whole cells [begin, end) along one axis that an interval covers.
///
/// Both bounds are rounded to the nearest cell boundary (cell i spans i to i+1 cell sizes from the origin), as the
/// original examples convert lengths with to_uint(units.x()), and clamped to the axis. So a length equal to the domain
/// size covers every cell even when the grid was rounded from it.
struct CellSpan {
    std::uint32_t begin = 0u; ///< the first cell in the span, counted from 0
    std::uint32_t end = 0u;   ///< one past the last cell in the span (begin == end: no cell)

    /// @brief Gives the cells that an interval covers along one axis.
    /// @param from the interval's start in cells from the origin (lattice units), not more than to
    /// @param to the interval's end in cells from the origin (lattice units)
    /// @param cells the number of cells along the axis
    /// @return the span, with both bounds within 0 to cells
    static constexpr CellSpan of(float from, float to, std::uint32_t cells) {
        const std::uint32_t first{ boundary(from, cells) };
        const std::uint32_t past_last{ boundary(to, cells) };
        const CellSpan span{ .begin = first, .end = past_last };
        return span;
    }

    /// @brief Tells whether a cell is in the span.
    /// @param i a cell, counted from 0
    /// @return true when begin <= i < end
    constexpr bool contains(std::uint32_t i) const { return i >= begin && i < end; }

    /// @brief Compares two spans, by begin and then by end.
    /// @return the ordering
    constexpr auto operator<=>(const CellSpan&) const = default;

private:
    /// @brief Gives the cell boundary nearest to a position, clamped to the axis.
    /// @param position the position in cells from the origin
    /// @param cells the number of cells along the axis
    /// @return the boundary, from 0 to cells
    static constexpr std::uint32_t boundary(float position, std::uint32_t cells) {
        if(position <= 0.0f) {
            return 0u;
        }
        const float last{ static_cast<float>(cells) };
        if(position >= last) {
            return cells;
        }
        const std::uint32_t nearest{ static_cast<std::uint32_t>(position + 0.5f) }; // NOLINT(bugprone-incorrect-roundings): the core's to_uint() rounding
        return nearest;
    }
};

/// @brief The density of a lattice fluid at rest under a force per volume toward lower heights (the core's
/// Units::rho_hydrostatic(); the lattice pressure is (ρ - 1)/3). ThermalBuilder and SurfaceBuilder start a hydrostatic
/// column with it.
class HydrostaticDensity {
public:
    /// @brief Gives the lattice density at a height, with the density 1 at the surface height.
    /// @param force_per_volume the force per volume in lattice units, toward lower heights
    /// @param height the height in cells
    /// @param surface_height the height in cells at which the density is 1
    /// @return the lattice density: 1 + 3 force (surface_height - height)
    static float at(float force_per_volume, float height, float surface_height) {
        const float depth{ surface_height - height };
        const float excess{ density_per_pressure * force_per_volume * depth };
        const float density{ excess + 1.0f };
        return density;
    }

private:
    static constexpr float density_per_pressure = 3.0f; ///< the inverse of the lattice's squared sound speed
};
