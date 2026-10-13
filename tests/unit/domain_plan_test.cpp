#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/domain_plan.hpp"
#include "setup/core/setup_error.hpp"
#include "box_stl.hpp"

#include <filesystem>
#include <memory>
#include <string>

namespace {

constexpr LatticeMemory d3q19_fp16 { 55u };
constexpr LatticeMemory d3q19_fp16_surface { 67u };

TEST(DomainPlanner, BoxSizesTheGridFromTheVramBudget) {
    // breaking_waves: 1 m x 5 m x 0.75 m in 2000 MB with FP16 and SURFACE
    const DomainPlan plan = DomainPlanner::plan(Domain::box(1.0_m, 5.0_m, 0.75_m).vram(2000_mb), d3q19_fp16_surface);
    EXPECT_EQ(plan.Nx, 203u);
    EXPECT_EQ(plan.Ny, 1014u);
    EXPECT_EQ(plan.Nz, 152u);
    EXPECT_FLOAT_EQ(plan.si_reference_size, 5.0f);      // the longest side
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, 1014.0f);
    EXPECT_FLOAT_EQ(plan.center_lbm.x, 101.0f); // the core's lbm.center(): cell i's center is at i
    EXPECT_FLOAT_EQ(plan.center_lbm.y, 506.5f);
    EXPECT_FLOAT_EQ(plan.center_lbm.z, 75.5f);
    EXPECT_TRUE(plan.stl_path.empty());
    EXPECT_FALSE(plan.mirror);
}

TEST(DomainPlanner, ClearancesAddCellsAroundTheModel) {
    const BoxStl box(test_file("fluidx3d_test_plan_geometry.stl"), 4.0f, 2.0f, 1.0f);
    const DomainPlan plan = DomainPlanner::plan(
        Domain::around(Model(resource_name(box.path()))).clearances(0.5_m, 1.0_m, 0.3_m).cell_size(0.1_m).vram_limit(20000_mb), d3q19_fp16);
    EXPECT_EQ(plan.Nx, 40u + 2u * 3u);
    EXPECT_EQ(plan.Ny, 20u + 2u * 3u);
    EXPECT_EQ(plan.Nz, 10u + 5u + 10u);
    EXPECT_FLOAT_EQ(plan.center_lbm.x, 22.5f);
    EXPECT_FLOAT_EQ(plan.center_lbm.y, 12.5f);
    EXPECT_FLOAT_EQ(plan.center_lbm.z, 10.0f); // resting on the 5-cell bottom clearance
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, 20.0f); // along Y
    EXPECT_NEAR(plan.si_reference_size, 2.0f, 1e-5f);
    EXPECT_FLOAT_EQ(plan.voxel_size, 40.0f);         // the longest side
    EXPECT_TRUE(std::filesystem::exists(plan.stl_path));
}

// A model 1.2 m long along Y (2 STL units) in a 1.2 m x 2.4 m x 1.2 m domain: half the domain's length.
TEST(DomainPlanner, SizeCentersTheModelWithAnOffset) {
    const BoxStl box(test_file("fluidx3d_test_plan_center.stl"), 4.0f, 2.0f, 1.0f);
    const DomainPlan plan = DomainPlanner::plan(
        Domain::around(Model(resource_name(box.path())).length(1.2_m)).size(1.2_m, 2.4_m, 1.2_m).model_offset(0_m, 0.12_m, 0_m).vram(100_mb),
        d3q19_fp16);
    const GridSize grid = grid_for_memory(1.2f, 2.4f, 1.2f, 100u, d3q19_fp16);
    EXPECT_EQ(plan.Nx, grid.x);
    EXPECT_EQ(plan.Ny, grid.y);
    EXPECT_EQ(plan.Nz, grid.z);
    const float length = 0.5f * (float)grid.y;
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, length);
    EXPECT_FLOAT_EQ(plan.si_reference_size, 1.2f);
    EXPECT_NEAR(plan.center_lbm.x, 0.5f * (float)grid.x - 0.5f, 1e-4f); // the core's lbm.center()
    EXPECT_NEAR(plan.center_lbm.y, 0.5f * (float)grid.y - 0.5f + 0.1f * length, 1e-3f); // 0.12 m is 0.1 model lengths
    EXPECT_NEAR(plan.center_lbm.z, 0.5f * (float)grid.z - 0.5f, 1e-4f);
    EXPECT_NEAR(plan.voxel_size, 2.0f * length, 1e-3f); // the longest side, X, is twice the length along Y
}

// An SDF model's box is its grid, sized as the core's read_sdf() does (the grid's cells times their mean size);
// voxelize_sdf() scales the grid's longest side before the rotation.
TEST(DomainPlanner, SdfModelIsItsGridsBox) {
    const BoxSdf sdf(test_file("fluidx3d_test_plan_box.sdf"), 8, 4, 3, 4.0f, 2.0f, 1.0f); // cells 0.5, 0.5, 0.333: mean 0.444
    const std::unique_ptr<SDF> core(read_sdf(sdf.path()));
    const float3 world = core->get_world_size();
    const GridSize grid = grid_for_memory(1.2f, 2.4f, 1.2f, 100u, d3q19_fp16);
    const float length = 0.5f * (float)grid.y;

    const DomainPlan plan = DomainPlanner::plan(
        Domain::around(Model(resource_name(sdf.path())).length(1.2_m)).size(1.2_m, 2.4_m, 1.2_m).vram(100_mb), d3q19_fp16);
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, length);
    EXPECT_FLOAT_EQ(plan.voxel_size, length * (world.x / world.y)); // X is the longest side
    EXPECT_EQ(plan.base_grid.z, (uint32_t)(length / world.y * world.z + 0.5f));

    // turned 90 degrees about Z: the length is along the grid's X, and the voxelizer still scales the grid's longest side
    const DomainPlan turned = DomainPlanner::plan(
        Domain::around(Model(resource_name(sdf.path())).rotation(0_deg, 0_deg, 90_deg).length(1.2_m)).size(1.2_m, 2.4_m, 1.2_m).vram(100_mb), d3q19_fp16);
    EXPECT_NEAR(turned.voxel_size, length, 1e-3f);
    EXPECT_NEAR((float)turned.base_grid.x, length / world.x * world.y, 1.0f);

    EXPECT_THROW(DomainPlanner::plan(Domain::around(Model(resource_name(sdf.path()))).clearances(1_m, 1_m, 1_m), d3q19_fp16), SetupError);
}

TEST(DomainPlanner, GapsPlaceTheModelsFrontAndBottom) {
    const BoxStl box(test_file("fluidx3d_test_plan_gaps.stl"), 4.0f, 2.0f, 1.0f);
    const DomainPlan plan = DomainPlanner::plan(
        Domain::around(Model(resource_name(box.path())).length(1.2_m)).size(1.2_m, 2.4_m, 1.2_m).gap_to_inlet(0.12_m).gap_to_floor(0.024_m).vram(100_mb),
        d3q19_fp16);
    const GridSize grid = grid_for_memory(1.2f, 2.4f, 1.2f, 100u, d3q19_fp16);
    const float length = 0.5f * (float)grid.y;
    const float cells_per_unit = length / 2.0f; // the model is 2 STL units long along Y
    EXPECT_EQ(plan.base_grid.x, (uint32_t)(cells_per_unit * 4.0f + 0.5f));
    EXPECT_EQ(plan.base_grid.y, (uint32_t)(cells_per_unit * 2.0f + 0.5f));
    EXPECT_EQ(plan.base_grid.z, (uint32_t)(cells_per_unit * 1.0f + 0.5f));
    // X centered; the bounding box minimum 0.1 and 0.02 model lengths from the inlet and the floor
    EXPECT_NEAR(plan.center_lbm.x, 0.5f * (float)grid.x - 0.5f, 1e-4f);
    EXPECT_NEAR(plan.center_lbm.y, 0.1f * length + 0.5f * cells_per_unit * 2.0f, 1e-3f);
    EXPECT_NEAR(plan.center_lbm.z, 0.02f * length + 0.5f * cells_per_unit * 1.0f, 1e-3f);
}

TEST(DomainPlanner, MirroredModelIsInThePlan) {
    const BoxStl box(test_file("fluidx3d_test_plan_mirror.stl"), 4.0f, 2.0f, 1.0f);
    const DomainPlan plan = DomainPlanner::plan(
        Domain::around(Model(resource_name(box.path())).length(1_m).mirrored(Axis::X)).size(2_m, 2_m, 1_m).vram(100_mb), d3q19_fp16);
    ASSERT_TRUE(plan.mirror);
    EXPECT_EQ(*plan.mirror, Axis::X);
}

// The lattices below: D3Q19 with FP16 DDFs, 38 bytes per cell in the DDF buffer, and the device's limit on one buffer.
// A box 2.01 m x 1 m x 1 m at 1 cm: 201 x 100 x 100 cells, the whole DDF buffer 76 MB.
constexpr LatticeMemory limited(std::uint32_t max_buffer_mb) { return { 55u, 3u, 38u, max_buffer_mb }; }

TEST(DomainPlanner, ALatticeOverTheBufferLimitIsSplitAlongItsLongestHorizontalAxis) {
    const Domain domain = Domain::box(2.01_m, 1.0_m, 1.0_m).cell_size(0.01_m);
    const DomainPlan whole = DomainPlanner::plan(domain, d3q19_fp16);
    const DomainPlan plan = DomainPlanner::plan(domain, limited(40u));
    ASSERT_EQ(whole.Nx, 201u);
    // the fewest domains that fit: two of 100 cells and their halos, 102 x 100 x 100 x 38 bytes = 37 MB
    EXPECT_EQ(plan.gpus.x, 2u);
    EXPECT_EQ(plan.gpus.y, 1u);
    EXPECT_EQ(plan.gpus.z, 1u);
    EXPECT_EQ(plan.Nx, 200u); // a multiple of the domains, so the core does not trim it again
    EXPECT_EQ(plan.Ny, 100u);
    EXPECT_EQ(plan.Nz, 100u);
    EXPECT_FLOAT_EQ(plan.center_lbm.x, whole.center_lbm.x); // the model's centre stays where it was
    const DomainPlan along_y = DomainPlanner::plan(Domain::box(1.0_m, 2.01_m, 1.0_m).cell_size(0.01_m), limited(40u));
    EXPECT_EQ(along_y.gpus.x, 1u);
    EXPECT_EQ(along_y.gpus.y, 2u);
    EXPECT_EQ(along_y.Ny, 200u);
}

TEST(DomainPlanner, ALatticeWithinTheLimitOrWithNoLimitKnownStaysWhole) {
    const Domain domain = Domain::box(2.01_m, 1.0_m, 1.0_m).cell_size(0.01_m);
    const DomainPlan within = DomainPlanner::plan(domain, limited(1000u));
    EXPECT_EQ(within.gpus.x, 1u);
    EXPECT_EQ(within.Nx, 201u);
    const DomainPlan unknown = DomainPlanner::plan(domain, limited(0u));
    EXPECT_EQ(unknown.gpus.x, 1u);
    EXPECT_EQ(unknown.Nx, 201u);
}

TEST(DomainPlanner, TheCallersSplitIsKeptAndRefusedOverTheBufferLimit) {
    const DomainPlan kept = DomainPlanner::plan(Domain::box(2.01_m, 1.0_m, 1.0_m).cell_size(0.01_m).gpus(1u, 2u, 1u), limited(1000u));
    EXPECT_EQ(kept.gpus.x, 1u);
    EXPECT_EQ(kept.gpus.y, 2u);
    // two domains along Y: 201 x 52 x 100 x 38 bytes = 38 MB, over 10 MB
    EXPECT_THROW(DomainPlanner::plan(Domain::box(2.01_m, 1.0_m, 1.0_m).cell_size(0.01_m).gpus(1u, 2u, 1u), limited(10u)), SetupError);
}

TEST(DomainPlanner, ALatticeNoSplitFitsIsRefused) {
    // the thinnest domain along X, 2 cells and 2 halo layers of 100 x 100 cells, is 1.5 MB, over a limit of 1 MB
    EXPECT_THROW(DomainPlanner::plan(Domain::box(2.01_m, 1.0_m, 1.0_m).cell_size(0.01_m), limited(1u)), SetupError);
}

TEST(DomainPlanner, GpusAreInThePlan) {
    const DomainPlan plan = DomainPlanner::plan(Domain::box(1.0_m, 2.0_m, 1.0_m).vram(100_mb).gpus(2u, 4u, 1u), d3q19_fp16);
    EXPECT_EQ(plan.gpus.x, 2u);
    EXPECT_EQ(plan.gpus.y, 4u);
    EXPECT_EQ(plan.gpus.z, 1u);
    EXPECT_THROW(Domain::box(1.0_m, 1.0_m, 1.0_m).gpus(0u, 1u, 1u).validate(), SetupError);
}

} // namespace
