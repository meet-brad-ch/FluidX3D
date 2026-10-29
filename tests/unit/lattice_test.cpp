#include "setup/domain/lattice.hpp"
#include "setup/core/setup_error.hpp"

#include <gtest/gtest.h>
#include <limits>
#include <ostream>

void PrintTo(const GridSize& grid, std::ostream* os) { *os << grid.x << " x " << grid.y << " x " << grid.z; }

namespace {

using namespace si::symbols;

/// @brief Gives a whole number of MiB as a memory amount.
/// @param mib the budget in whole MiB (2^20 bytes)
/// @return the memory amount
si::MemoryAmount budget_of(std::uint32_t mib) {
    const double mib_double{ static_cast<double>(mib) };
    const si::MemoryAmount budget{ si::mebibytes(mib_double) };
    return budget;
}

// bytes per cell as the core's bytes_per_cell_device(): 19 DDFs (2 bytes with FP16, 4 without) + 17 for rho, u, flags
constexpr LatticeMemory d3q19_fp16 { au::bytes(19.0 * 2.0 + 17.0) };               // 55
constexpr LatticeMemory d3q19_fp16_surface { au::bytes(19.0 * 2.0 + 17.0 + 12.0) }; // 67: phi, mass, flags
constexpr LatticeMemory d3q19_fp16_temperature { au::bytes(19.0 * 2.0 + 17.0 + 7.0 * 2.0 + 4.0) }; // 73: gi, T
constexpr LatticeMemory d3q19_fp32_force { au::bytes(19.0 * 4.0 + 17.0 + 12.0) };   // 105: F

struct ExampleGrid {
    const char* example;
    float aspect_x, aspect_y, aspect_z;
    std::uint32_t budget_mb;
    LatticeMemory lattice;
    GridSize grid; // the example's grid, as the core computed it
};

// Aspect ratios as the planner computes them: the size in m (Domain::size()), or divided by its largest side (Domain::box()).
const ExampleGrid example_grids[] = {
    { "cow",                1.0f, 2.0f, 1.0f,  1000u, d3q19_fp16, { 212u, 424u, 212u } },
    { "xwing",              1.0f, 2.0f, 0.5f,   880u, d3q19_fp16, { 256u, 512u, 128u } },
    { "starship",           1.0f, 2.0f, 2.0f,  1000u, d3q19_fp16, { 168u, 337u, 337u } },
    { "tie_fighter",        1.0f, 2.0f, 1.0f,  1760u, d3q19_fp16, { 256u, 512u, 256u } },
    { "city",               1.0f, 2.0f, 0.5f,  2152u, d3q19_fp16, { 345u, 690u, 172u } },
    { "concorde",           1.0f, 3.0f, 0.5f,  6084u, d3q19_fp16, { 426u, 1278u, 213u } },
    { "edf",                1.0f, 1.5f, 1.0f,  8000u, d3q19_fp16, { 467u, 700u, 467u } },
    // the example splits the grid over 2x4x1 GPUs; the LBM rounds it down to multiples of that (lbm.cpp), 180 x 724 x 145
    { "space_shuttle",      1.0f, 4.0f, 0.8f,  1000u, d3q19_fp16, { 181u, 725u, 145u } },
    { "radial_fan",         3.0f, 3.0f, 1.0f,   181u, d3q19_fp16, { 218u, 218u, 73u } },
    { "breaking_waves",     1.0f / 5.0f, 5.0f / 5.0f, 0.75f / 5.0f, 2000u, d3q19_fp16_surface, { 203u, 1014u, 152u } },
    { "thermal_convection", 0.1f / 0.6f, 0.6f / 0.6f, 0.2f / 0.6f,  2000u, d3q19_fp16_temperature, { 134u, 803u, 268u } },
    { "particle_test",      1.0f, 1.0f, 1.0f,  2000u, d3q19_fp32_force, { 271u, 271u, 271u } },
};

// Regression for defect 11: every member of LatticeMemory has an initializer (a constexpr default-initialized object
// does not compile while one member has none)
constexpr LatticeMemory default_memory;
static_assert(default_memory.memory_per_cell == au::ZERO, "a default LatticeMemory has 0 bytes per cell");

// Regression for defect 18: a budget of 0 or 0 bytes per cell has no grid (0 cells or an infinite scale): refused
TEST(Lattice, RefusesAnEmptyBudgetOrNoBytesPerCell) {
    EXPECT_THROW(d3q19_fp16.grid_for_memory(1.0f, 1.0f, 1.0f, 0.0 * MiB), SetupError);
    const LatticeMemory no_bytes{ au::bytes(0.0) };
    EXPECT_THROW(no_bytes.grid_for_memory(1.0f, 1.0f, 1.0f, 1000.0 * MiB), SetupError);
}

// The typed budget: a negative or not-a-number amount has no grid either
TEST(Lattice, RefusesANegativeOrNotANumberBudget) {
    EXPECT_THROW(d3q19_fp16.grid_for_memory(1.0f, 1.0f, 1.0f, -1.0 * MiB), SetupError);
    const double not_a_number{ std::numeric_limits<double>::quiet_NaN() };
    EXPECT_THROW(d3q19_fp16.grid_for_memory(1.0f, 1.0f, 1.0f, not_a_number * MiB), SetupError);
}

// A budget in GiB gives the same grid as the same budget in MiB (the core counts MB of 2^20 bytes)
TEST(Lattice, BudgetInGibibytesIsTheSameAsInMebibytes) {
    const GridSize in_gib{ d3q19_fp16.grid_for_memory(1.0f, 2.0f, 1.0f, 2.0 * GiB) };
    const GridSize in_mib{ d3q19_fp16.grid_for_memory(1.0f, 2.0f, 1.0f, 2048.0 * MiB) };
    EXPECT_EQ(in_gib, in_mib);
}

TEST(Lattice, GridForMemoryReproducesTheExampleGrids) {
    for(const ExampleGrid& e : example_grids) {
        const GridSize grid{ e.lattice.grid_for_memory(e.aspect_x, e.aspect_y, e.aspect_z, budget_of(e.budget_mb)) };
        EXPECT_EQ(grid, e.grid) << e.example;
    }
}

TEST(Lattice, TwoDimensionalGridsHaveOneLayer) {
    const LatticeMemory d2q9{ au::bytes(9.0 * 2.0 + 17.0), 2u };
    const GridSize grid{ d2q9.grid_for_memory(1.0f, 2.0f, 1.0f, 100.0 * MiB) };
    EXPECT_EQ(grid.z, 1u);
    EXPECT_EQ(grid.y, 2u * grid.x);
}

TEST(Lattice, RequiredMemoryInWholeMegabytes) {
    const LatticeMemory lattice{ au::bytes(93.0) };
    EXPECT_EQ(lattice.required_memory({ 256u, 256u, 256u }), 1488.0 * MiB); // 16.8 M cells x 93 bytes
    EXPECT_EQ(lattice.required_memory({ 1u, 1u, 1u }), 0.0 * MiB);
}

static_assert(CellSpan::of(0.0f, 0.528f / 0.01f, 352u).end == 53u); // hydraulic_jump's socket: to_uint(52.8) in the original

TEST(CellSpan, RoundsBothBoundsToTheNearestCellBoundary) {
    EXPECT_EQ(CellSpan::of(2.9f, 7.49f, 100u), (CellSpan{ 3u, 7u }));
    EXPECT_EQ(CellSpan::of(2.4f, 7.5f, 100u), (CellSpan{ 2u, 8u }));
    EXPECT_EQ(CellSpan::of(-3.0f, 0.0f, 100u), (CellSpan{ 0u, 0u }));
}

TEST(CellSpan, BoundsAreClampedToTheAxis) {
    EXPECT_EQ(CellSpan::of(0.0f, 198.5f, 199u).end, 199u); // 0.5 m of 1/397 m cells on a 199-cell axis
    EXPECT_EQ(CellSpan::of(0.0f, 250.0f, 199u).end, 199u);
    EXPECT_EQ(CellSpan::of(0.0f, 198.4f, 199u).end, 198u);
    EXPECT_EQ(CellSpan::of(250.0f, 300.0f, 199u), (CellSpan{ 199u, 199u })); // beyond the axis: empty
}

TEST(CellSpan, ContainsItsCells) {
    const CellSpan span{ 2u, 5u };
    EXPECT_FALSE(span.contains(1u));
    EXPECT_TRUE(span.contains(2u));
    EXPECT_TRUE(span.contains(4u));
    EXPECT_FALSE(span.contains(5u));
}

TEST(Lattice, GridFitsItsBudget) {
    for(const ExampleGrid& e : example_grids) {
        const GridSize grid{ e.lattice.grid_for_memory(e.aspect_x, e.aspect_y, e.aspect_z, budget_of(e.budget_mb)) };
        const std::uint32_t allowed_mb{ e.budget_mb + e.budget_mb / 100u }; // rounding: within 1 %
        EXPECT_LE(e.lattice.required_memory(grid), budget_of(allowed_mb)) << e.example;
    }
}

// the hydrostatic start: 1 at the surface, 1 + 3 f d at the depth d below it, less than 1 above it (the core's
// Units::rho_hydrostatic(), float for float)
TEST(HydrostaticDensity, IsOneAtTheSurfaceAndGrowsWithTheDepth) {
    const float force{ 1.0E-4f };
    const float surface{ 40.0f };
    EXPECT_EQ(HydrostaticDensity::at(force, surface, surface), 1.0f);
    const float depth{ 10.0f };
    const float expected_below{ 3.0f * force * depth + 1.0f };
    EXPECT_EQ(HydrostaticDensity::at(force, surface - depth, surface), expected_below);
    EXPECT_LT(HydrostaticDensity::at(force, surface + depth, surface), 1.0f);
}

} // namespace
