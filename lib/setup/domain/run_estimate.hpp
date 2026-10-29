#pragma once

#include "setup/domain/lattice.hpp"
#include "setup/units/units.hpp"
#include "utilities.hpp"

#include <cstdint>
#include <optional>
#include <string>

/// @brief What a run will cost, before it starts: the cells, the device memory per domain (its halo included), the
/// largest buffer (the DDFs) per domain against the device's limit, the time steps and, from a measured speed, the
/// wall time. The cost grows as dx^-4 (cells as dx^-3, steps as dx^-1 at a constant lattice speed).
/// @code
/// using namespace si::symbols;
/// const RunEstimate e = RunEstimate::of({ 512u, 512u, 412u }, uint3(2u, 1u, 1u), lattice, 67500u, 8000.0 * MLUPs);
/// if(!e.fits(24000.0 * MiB)) ... // the memory of each device
/// @endcode
struct RunEstimate {
    GridSize grid{};                               ///< the whole grid's cells along x, y and z
    uint3 domains = uint3(1u, 1u, 1u);             ///< the number of domains along each axis
    std::uint64_t cells = 0ull;                    ///< the whole grid's cells
    std::uint64_t domain_cells = 0ull;             ///< one domain's cells, its halo included
    si::MemoryAmount domain_memory{ au::ZERO };    ///< the device memory of one domain, all buffers together
    si::MemoryAmount domain_ddf_memory{ au::ZERO }; ///< the size of one domain's largest buffer, the DDFs
    std::optional<si::MemoryAmount> ddf_limit{};   ///< the device's limit on one buffer, or none when not known
    std::uint64_t steps = 0ull;                    ///< the run's time steps
    std::optional<si::Duration> wall_time{};       ///< the run's wall time from a measured speed, or none without a speed

    /// @brief Estimates the cost of a run from its grid, its domains, the build's memory per cell and its steps.
    ///
    /// The wall time is the cells times the steps over the speed (the whole grid, without the halo cells).
    /// @param grid the whole grid's cells along x, y and z, at least 1 each
    /// @param domains the number of domains along each axis, at least 1 each
    /// @param lattice the device memory per cell, the DDF buffer's bytes per cell, the halo layers and the buffer limit
    /// @param steps the time steps of the run
    /// @param speed a measured speed of this build on the device, more than 0, or none for no wall time
    /// @return the estimate
    /// @throws SetupError when the grid has no cells along an axis, an axis has no domain, or the speed is not more
    /// than 0
    static RunEstimate of(GridSize grid, uint3 domains, const LatticeMemory& lattice, std::uint64_t steps, std::optional<si::LatticeUpdateRate> speed);

    /// @brief Tells whether the run fits the devices: one domain in one device's memory, and its DDF buffer within
    /// the device's buffer limit (when the limit is known).
    /// @param device the memory of each device
    /// @return true when both fit
    bool fits(si::MemoryAmount device) const;

    /// @brief Describes the estimate in one line for a log: the cells, the domains, the MB per domain, the DDF
    /// buffer against the limit, the steps, and the wall time when it is known.
    /// @return the line (MB are 2^20 bytes)
    /// @throws SetupError when the line does not fit its text buffer (a programming error)
    std::string describe() const;

private:
    /// @brief Checks the inputs of of().
    /// @param grid the cells along x, y and z
    /// @param domains the number of domains along x, y and z
    /// @param speed a measured speed, or none
    /// @throws SetupError when the grid has no cells along an axis, an axis has no domain, or the speed is not more
    /// than 0
    static void check(GridSize grid, uint3 domains, std::optional<si::LatticeUpdateRate> speed);

    /// @brief Gives the words for describe() on the DDF buffer against the device's limit.
    /// @return ", within the device's buffer limit", ", OVER the device's buffer limit", or empty when the limit is
    /// not known
    const char* limit_words() const;
};
