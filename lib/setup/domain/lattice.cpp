#include "setup/domain/lattice.hpp"

#include <algorithm>
#include <cmath>

namespace {
std::uint32_t round_to_cells(float cells) { // as the core's to_uint()
    return static_cast<std::uint32_t>(std::max(cells + 0.5f, 0.5f));
}
} // namespace

GridSize grid_for_memory(float aspect_x, float aspect_y, float aspect_z, std::uint32_t budget_mb, LatticeMemory lattice) {
    const float bytes_per_cell = static_cast<float>(lattice.bytes_per_cell);
    if(lattice.dimensions == 2u) {
        const float memory_required = (aspect_x * aspect_y) * bytes_per_cell / 1048576.0f; // MB
        const float scaling = std::sqrt(static_cast<float>(budget_mb) / memory_required);
        return { round_to_cells(scaling * aspect_x), round_to_cells(scaling * aspect_y), 1u };
    }
    const float memory_required = (aspect_x * aspect_y * aspect_z) * bytes_per_cell / 1048576.0f; // MB
    const float scaling = std::cbrt(static_cast<float>(budget_mb) / memory_required);
    return { round_to_cells(scaling * aspect_x), round_to_cells(scaling * aspect_y), round_to_cells(scaling * aspect_z) };
}

std::uint64_t required_memory_mb(GridSize grid, LatticeMemory lattice) {
    const std::uint64_t cells = static_cast<std::uint64_t>(grid.x) * grid.y * grid.z;
    return cells * lattice.bytes_per_cell / 1048576ull;
}
