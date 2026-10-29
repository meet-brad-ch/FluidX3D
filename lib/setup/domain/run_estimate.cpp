#include "setup/domain/run_estimate.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/domain_plan.hpp"

#include <cstdio>

void RunEstimate::check(GridSize grid, uint3 domains, std::optional<si::LatticeUpdateRate> speed) {
    if(grid.x == 0u || grid.y == 0u || grid.z == 0u) {
        throw SetupError("RunEstimate: the grid has no cells");
    }
    if(domains.x == 0u || domains.y == 0u || domains.z == 0u) {
        throw SetupError("RunEstimate: at least one domain along each axis");
    }
    if(speed) {
        const si::LatticeUpdateRate measured{ *speed };
        if(!(measured > au::ZERO)) {
            throw SetupError("RunEstimate: a measured speed must be more than 0 MLUPs");
        }
    }
}

RunEstimate RunEstimate::of(GridSize grid, uint3 domains, const LatticeMemory& lattice, std::uint64_t steps, std::optional<si::LatticeUpdateRate> speed) {
    check(grid, domains, speed);
    const std::uint64_t grid_x{ static_cast<std::uint64_t>(grid.x) };
    const std::uint64_t grid_xy{ grid_x * grid.y };
    const std::uint64_t cell_count{ grid_xy * grid.z };
    const std::uint64_t per_domain_cells{ DomainPlanner::domain_cells(grid, domains, lattice.halo_layers) };
    const double per_domain_cells_double{ static_cast<double>(per_domain_cells) };
    const double bytes_per_cell{ lattice.memory_per_cell.in(au::bytes) };
    const double memory_bytes{ per_domain_cells_double * bytes_per_cell };
    const si::MemoryAmount memory{ au::bytes(memory_bytes) };
    const double buffer_bytes_per_cell{ lattice.buffer_memory_per_cell.in(au::bytes) };
    const double ddf_bytes{ per_domain_cells_double * buffer_bytes_per_cell };
    const si::MemoryAmount ddf_memory{ au::bytes(ddf_bytes) };
    std::optional<si::MemoryAmount> buffer_limit{};
    if(lattice.max_buffer > au::ZERO) {
        buffer_limit = lattice.max_buffer;
    }
    std::optional<si::Duration> run_time{};
    if(speed) {
        const double cell_count_double{ static_cast<double>(cell_count) };
        const double steps_double{ static_cast<double>(steps) };
        const double updates{ cell_count_double * steps_double };
        const auto work = si::lattice_updates(updates);
        const auto quotient = work / *speed;
        run_time = si::Duration(quotient);
    }
    const RunEstimate estimate{
        .grid = grid,
        .domains = domains,
        .cells = cell_count,
        .domain_cells = per_domain_cells,
        .domain_memory = memory,
        .domain_ddf_memory = ddf_memory,
        .ddf_limit = buffer_limit,
        .steps = steps,
        .wall_time = run_time,
    };
    return estimate;
}

bool RunEstimate::fits(si::MemoryAmount device) const {
    const bool domain_fits{ domain_memory <= device };
    bool buffer_fits{ true };
    if(ddf_limit) {
        buffer_fits = domain_ddf_memory <= *ddf_limit;
    }
    const bool fit{ domain_fits && buffer_fits };
    return fit;
}

const char* RunEstimate::limit_words() const {
    const char* words{ "" };
    if(ddf_limit) {
        const bool within{ domain_ddf_memory <= *ddf_limit };
        if(within) {
            words = ", within the device's buffer limit";
        } else {
            words = ", OVER the device's buffer limit";
        }
    }
    return words;
}

std::string RunEstimate::describe() const {
    const char* const limit{ limit_words() };
    const double cell_total{ static_cast<double>(cells) };
    const double cells_millions{ cell_total * 1.0E-6 };
    const double domain_mb{ domain_memory.in(si::mebibytes) };
    const double ddf_mb{ domain_ddf_memory.in(si::mebibytes) };
    const unsigned long long step_count{ static_cast<unsigned long long>(steps) };
    constexpr std::size_t text_capacity{ 320 };
    char text[text_capacity]{};
    const int written{ std::snprintf(text, sizeof(text), "%u x %u x %u = %.3g M cells in %u x %u x %u domains: %.1f MB per domain (DDF buffer %.1f MB%s), %llu steps",
                                     grid.x, grid.y, grid.z, cells_millions, domains.x, domains.y, domains.z, domain_mb, ddf_mb, limit, step_count) };
    if(written < 0 || static_cast<std::size_t>(written) >= text_capacity) {
        throw SetupError("RunEstimate::describe(): the description does not fit its text buffer");
    }
    std::string line{ text };
    if(wall_time.has_value()) {
        const double wall_s{ wall_time.value().in(au::seconds) };
        const int time_written{ std::snprintf(text, sizeof(text), ", about %.0f s of wall time", wall_s) };
        if(time_written < 0 || static_cast<std::size_t>(time_written) >= text_capacity) {
            throw SetupError("RunEstimate::describe(): the wall time does not fit its text buffer");
        }
        line += text;
    }
    return line;
}
