#include "setup/domain/run_estimate.hpp"
#include "setup/core/setup_error.hpp"

#include <gtest/gtest.h>

namespace {

using namespace si::symbols;

/// @return the 201 x 100 x 100 cells of DomainPlanner's own test case
GridSize test_grid() {
    const GridSize grid = GridSize{ .x = 201u, .y = 100u, .z = 100u };
    return grid;
}

/// @return 55 bytes per cell in 3 dimensions, DDFs 38 bytes per cell, a 40 MB buffer limit, one halo layer
LatticeMemory test_lattice() {
    const LatticeMemory lattice = LatticeMemory{ .memory_per_cell = au::bytes(55.0), .dimensions = 3u, .buffer_memory_per_cell = au::bytes(38.0), .max_buffer = 40.0 * MiB, .halo_layers = 1u };
    return lattice;
}

// 201 x 100 x 100 cells in 2 domains along x (DomainPlanner's own test case): 55 bytes per cell, DDFs 38
TEST(RunEstimate, CountsADomainWithItsHalo) {
    const LatticeMemory one = test_lattice();
    const GridSize grid = test_grid();
    const uint3 two_along_x = uint3(2u, 1u, 1u);
    const RunEstimate e = RunEstimate::of(grid, two_along_x, one, 1000u, 2000.0 * MLUPs);
    EXPECT_EQ(e.cells, 2010000ull);
    const unsigned long long expected_domain_cells = 102ull * 100ull * 100ull;
    EXPECT_EQ(e.domain_cells, expected_domain_cells); // 100 cells and one halo layer on each side of the cut
    const double domain_memory = e.domain_memory.in(MiB);
    const double expected_domain_memory = 102.0 * 100.0 * 100.0 * 55.0 / 1048576.0;
    EXPECT_DOUBLE_EQ(domain_memory, expected_domain_memory);
    const double ddf_memory = e.domain_ddf_memory.in(MiB);
    const double expected_ddf_memory = 102.0 * 100.0 * 100.0 * 38.0 / 1048576.0;
    EXPECT_DOUBLE_EQ(ddf_memory, expected_ddf_memory); // 36.97 MB
    ASSERT_TRUE(e.wall_time.has_value());
    const double wall_time = e.wall_time->in(s);
    const double expected_wall_time = 2010000.0 * 1000.0 / 2.0E9;
    EXPECT_DOUBLE_EQ(wall_time, expected_wall_time); // 1.005 s at 2000 MLUPs
    LatticeMemory two = one;
    two.halo_layers = 2u; // SCALAR_FV
    const RunEstimate wide_halo = RunEstimate::of(grid, two_along_x, two, 1000u, std::nullopt);
    const unsigned long long expected_wide_cells = 104ull * 100ull * 100ull;
    EXPECT_EQ(wide_halo.domain_cells, expected_wide_cells);
}

TEST(RunEstimate, FitsTheDeviceAndItsBufferLimit) {
    const LatticeMemory lattice = test_lattice();
    const GridSize grid = test_grid();
    const uint3 two_along_x = uint3(2u, 1u, 1u);
    const RunEstimate e = RunEstimate::of(grid, two_along_x, lattice, 1000u, std::nullopt);
    EXPECT_TRUE(e.fits(100.0 * MiB)); // 53.5 MB per domain, DDFs 37 MB within 40
    EXPECT_FALSE(e.fits(50.0 * MiB)); // the domain over the device's memory
    LatticeMemory tight = lattice;
    tight.max_buffer = 36.0 * MiB;
    const RunEstimate tight_estimate = RunEstimate::of(grid, two_along_x, tight, 1000u, std::nullopt);
    EXPECT_FALSE(tight_estimate.fits(100.0 * MiB)); // DDFs over 36 MB
    EXPECT_FALSE(e.wall_time.has_value());
    const std::string described = e.describe();
    const std::size_t grid_at = described.find("201 x 100 x 100");
    EXPECT_NE(grid_at, std::string::npos);
    const std::string described_again = e.describe();
    const std::size_t limit_at = described_again.find("within the device's buffer limit");
    EXPECT_NE(limit_at, std::string::npos);
}

TEST(RunEstimate, RefusesNoCellsNoDomainAndNoSpeed) {
    const LatticeMemory lattice = LatticeMemory{ .memory_per_cell = au::bytes(55.0) };
    const GridSize no_cells = GridSize{ .x = 0u, .y = 100u, .z = 100u };
    const GridSize small = GridSize{ .x = 10u, .y = 10u, .z = 10u };
    EXPECT_THROW(RunEstimate::of(no_cells, uint3(1u, 1u, 1u), lattice, 1u, std::nullopt), SetupError);
    EXPECT_THROW(RunEstimate::of(small, uint3(0u, 1u, 1u), lattice, 1u, std::nullopt), SetupError);
    EXPECT_THROW(RunEstimate::of(small, uint3(1u, 1u, 1u), lattice, 1u, 0.0 * MLUPs), SetupError);
}

} // namespace
