#include "setup/domain/lattice.hpp"

#include <gtest/gtest.h>
#include <ostream>

void PrintTo(const GridSize& grid, std::ostream* os) { *os << grid.x << " x " << grid.y << " x " << grid.z; }

namespace {

// bytes per cell as the core's bytes_per_cell_device(): 19 DDFs (2 bytes with FP16, 4 without) + 17 for rho, u, flags
constexpr LatticeMemory d3q19_fp16 { 19u * 2u + 17u };               // 55
constexpr LatticeMemory d3q19_fp16_surface { 19u * 2u + 17u + 12u }; // 67: phi, mass, flags
constexpr LatticeMemory d3q19_fp16_temperature { 19u * 2u + 17u + 7u * 2u + 4u }; // 73: gi, T
constexpr LatticeMemory d3q19_fp32_force { 19u * 4u + 17u + 12u };   // 105: F

struct ExampleGrid {
    const char* example;
    float aspect_x, aspect_y, aspect_z;
    std::uint32_t budget_mb;
    LatticeMemory lattice;
    GridSize grid; // from tests/baselines/<example>.txt
};

// Aspect ratios as the examples compute them: set_domain_aspect_ratio(), or the size in m divided by its largest side.
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

TEST(Lattice, GridForMemoryReproducesTheExampleGrids) {
    for(const ExampleGrid& e : example_grids) {
        EXPECT_EQ(grid_for_memory(e.aspect_x, e.aspect_y, e.aspect_z, e.budget_mb, e.lattice), e.grid) << e.example;
    }
}

TEST(Lattice, TwoDimensionalGridsHaveOneLayer) {
    const GridSize grid = grid_for_memory(1.0f, 2.0f, 1.0f, 100u, { 9u * 2u + 17u, 2u });
    EXPECT_EQ(grid.z, 1u);
    EXPECT_EQ(grid.y, 2u * grid.x);
}

TEST(Lattice, RequiredMemoryInWholeMegabytes) {
    EXPECT_EQ(required_memory_mb({ 256u, 256u, 256u }, { 93u }), 1488u); // 16.8 M cells x 93 bytes
    EXPECT_EQ(required_memory_mb({ 1u, 1u, 1u }, { 93u }), 0u);
}

TEST(Lattice, GridFitsItsBudget) {
    for(const ExampleGrid& e : example_grids) {
        const GridSize grid = grid_for_memory(e.aspect_x, e.aspect_y, e.aspect_z, e.budget_mb, e.lattice);
        EXPECT_LE(required_memory_mb(grid, e.lattice), e.budget_mb + e.budget_mb / 100u) << e.example; // rounding: within 1 %
    }
}

} // namespace
