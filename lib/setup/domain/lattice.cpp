#include "setup/domain/lattice.hpp"
#include "setup/core/setup_error.hpp"

#include <algorithm>
#include <cmath>

std::uint32_t LatticeMemory::round_to_cells(float cells) {
    const float rounded_up{ std::max(cells + 0.5f, 0.5f) };
    const std::uint32_t whole{ static_cast<std::uint32_t>(rounded_up) };
    return whole;
}

GridSize LatticeMemory::grid_for_memory(float aspect_x, float aspect_y, float aspect_z, si::MemoryAmount budget) const {
    const bool positive{ budget > au::ZERO }; // false for NaN too
    if(!positive) {
        throw SetupError("LatticeMemory::grid_for_memory(): the memory budget must be more than 0 MiB");
    }
    const bool has_memory{ memory_per_cell > au::ZERO }; // false for NaN too
    if(!has_memory) {
        throw SetupError("LatticeMemory::grid_for_memory(): the lattice must have more than 0 bytes per cell");
    }
    // the core's resolution(): the same float operations in the same order, so both give the same grid
    const double cell_bytes_double{ memory_per_cell.in(au::bytes) };
    const float cell_bytes{ static_cast<float>(cell_bytes_double) };
    const double budget_mib{ budget.in(si::symbols::MiB) };
    const float budget_mb{ static_cast<float>(budget_mib) }; // the core's float MB
    const float mb{ static_cast<float>(bytes_per_mb) };
    if(dimensions == 2u) {
        const float area{ aspect_x * aspect_y };
        const float memory_required{ area * cell_bytes / mb }; // MB
        const float scaling{ std::sqrt(budget_mb / memory_required) };
        const GridSize grid{ .x = round_to_cells(scaling * aspect_x), .y = round_to_cells(scaling * aspect_y), .z = 1u };
        return grid;
    }
    const float volume{ aspect_x * aspect_y * aspect_z };
    const float memory_required{ volume * cell_bytes / mb }; // MB
    const float scaling{ std::cbrt(budget_mb / memory_required) };
    const GridSize grid{ .x = round_to_cells(scaling * aspect_x), .y = round_to_cells(scaling * aspect_y), .z = round_to_cells(scaling * aspect_z) };
    return grid;
}

si::MemoryAmount LatticeMemory::required_memory(GridSize grid) const {
    const std::uint64_t cells{ static_cast<std::uint64_t>(grid.x) * grid.y * grid.z };
    const std::uint64_t per_cell{ whole_bytes(memory_per_cell) };
    const std::uint64_t bytes{ cells * per_cell };
    const std::uint64_t whole_mb{ bytes / bytes_per_mb }; // the core's whole MB, rounded down
    const double whole_mib{ static_cast<double>(whole_mb) };
    const si::MemoryAmount amount{ si::mebibytes(whole_mib) };
    return amount;
}
