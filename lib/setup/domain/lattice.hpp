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

// grid with the given aspect ratio that fills budget_mb, as the core's resolution() (same float operations)
GridSize grid_for_memory(float aspect_x, float aspect_y, float aspect_z, std::uint32_t budget_mb, LatticeMemory lattice);

// whole MB of device memory that a grid needs
std::uint64_t required_memory_mb(GridSize grid, LatticeMemory lattice);
