#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/domain_plan.hpp"
#include "setup/core/setup_error.hpp"
#include "box_stl.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

namespace {

using namespace si::symbols;

constexpr LatticeMemory d3q19_fp16 { au::bytes(55.0) };
constexpr LatticeMemory d3q19_fp16_surface { au::bytes(67.0) };

TEST(DomainPlanner, BoxSizesTheGridFromTheVramBudget) {
    // breaking_waves: 1 m x 5 m x 0.75 m in 2000 MB with FP16 and SURFACE
    const DomainPlan plan = DomainPlanner::plan(Domain::box(1.0 * m, 5.0 * m, 0.75 * m).vram(2000.0 * MiB), d3q19_fp16_surface);
    EXPECT_EQ(plan.Nx, 203u);
    EXPECT_EQ(plan.Ny, 1014u);
    EXPECT_EQ(plan.Nz, 152u);
    EXPECT_FLOAT_EQ(static_cast<float>(plan.reference_length.in(m)), 5.0f); // the longest side
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
        Domain::around(Model(resource_name(box.path()))).clearances(0.5 * m, 1.0 * m, 0.3 * m).cell_size(0.1 * m).vram_limit(20000.0 * MiB), d3q19_fp16);
    EXPECT_EQ(plan.Nx, 40u + 2u * 3u);
    EXPECT_EQ(plan.Ny, 20u + 2u * 3u);
    EXPECT_EQ(plan.Nz, 10u + 5u + 10u);
    EXPECT_FLOAT_EQ(plan.center_lbm.x, 22.5f);
    EXPECT_FLOAT_EQ(plan.center_lbm.y, 12.5f);
    EXPECT_FLOAT_EQ(plan.center_lbm.z, 10.0f); // resting on the 5-cell bottom clearance
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, 20.0f); // along Y
    EXPECT_NEAR(static_cast<float>(plan.reference_length.in(m)), 2.0f, 1e-5f);
    EXPECT_FLOAT_EQ(plan.voxel_size, 40.0f);         // the longest side
    EXPECT_TRUE(std::filesystem::exists(plan.stl_path));
}

// A model 1.2 m long along Y (2 STL units) in a 1.2 m x 2.4 m x 1.2 m domain: half the domain's length.
TEST(DomainPlanner, SizeCentersTheModelWithAnOffset) {
    const BoxStl box(test_file("fluidx3d_test_plan_center.stl"), 4.0f, 2.0f, 1.0f);
    const DomainPlan plan = DomainPlanner::plan(
        Domain::around(Model(resource_name(box.path())).length(1.2 * m)).size(1.2 * m, 2.4 * m, 1.2 * m).model_offset(0.0 * m, 0.12 * m, 0.0 * m).vram(100.0 * MiB),
        d3q19_fp16);
    const GridSize grid{ d3q19_fp16.grid_for_memory(1.2f, 2.4f, 1.2f, 100.0 * MiB) };
    EXPECT_EQ(plan.Nx, grid.x);
    EXPECT_EQ(plan.Ny, grid.y);
    EXPECT_EQ(plan.Nz, grid.z);
    const float length = 0.5f * (float)grid.y;
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, length);
    EXPECT_FLOAT_EQ(static_cast<float>(plan.reference_length.in(m)), 1.2f);
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
    const GridSize grid{ d3q19_fp16.grid_for_memory(1.2f, 2.4f, 1.2f, 100.0 * MiB) };
    const float length = 0.5f * (float)grid.y;

    const DomainPlan plan = DomainPlanner::plan(
        Domain::around(Model(resource_name(sdf.path())).length(1.2 * m)).size(1.2 * m, 2.4 * m, 1.2 * m).vram(100.0 * MiB), d3q19_fp16);
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, length);
    EXPECT_FLOAT_EQ(plan.voxel_size, length * (world.x / world.y)); // X is the longest side
    EXPECT_EQ(plan.base_grid.z, (uint32_t)(length / world.y * world.z + 0.5f));

    // turned 90 degrees about Z: the length is along the grid's X, and the voxelizer still scales the grid's longest side
    const DomainPlan turned = DomainPlanner::plan(
        Domain::around(Model(resource_name(sdf.path())).rotation(0.0 * deg, 0.0 * deg, 90.0 * deg).length(1.2 * m)).size(1.2 * m, 2.4 * m, 1.2 * m).vram(100.0 * MiB), d3q19_fp16);
    EXPECT_NEAR(turned.voxel_size, length, 1e-3f);
    EXPECT_NEAR((float)turned.base_grid.x, length / world.x * world.y, 1.0f);

    EXPECT_THROW(DomainPlanner::plan(Domain::around(Model(resource_name(sdf.path()))).clearances(1.0 * m, 1.0 * m, 1.0 * m), d3q19_fp16), SetupError);
}

TEST(DomainPlanner, GapsPlaceTheModelsFrontAndBottom) {
    const BoxStl box(test_file("fluidx3d_test_plan_gaps.stl"), 4.0f, 2.0f, 1.0f);
    const DomainPlan plan = DomainPlanner::plan(
        Domain::around(Model(resource_name(box.path())).length(1.2 * m)).size(1.2 * m, 2.4 * m, 1.2 * m).gap_to_inlet(0.12 * m).gap_to_floor(0.024 * m).vram(100.0 * MiB),
        d3q19_fp16);
    const GridSize grid{ d3q19_fp16.grid_for_memory(1.2f, 2.4f, 1.2f, 100.0 * MiB) };
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
        Domain::around(Model(resource_name(box.path())).length(1.0 * m).mirrored(Axis::X)).size(2.0 * m, 2.0 * m, 1.0 * m).vram(100.0 * MiB), d3q19_fp16);
    ASSERT_TRUE(plan.mirror);
    EXPECT_EQ(*plan.mirror, Axis::X);
}

// The lattices below: D3Q19 with FP16 DDFs, 38 bytes per cell in the DDF buffer, and the device's limit on one buffer.
// A box 2.01 m x 1 m x 1 m at 1 cm: 201 x 100 x 100 cells, the whole DDF buffer 76 MB.
constexpr LatticeMemory limited(double max_buffer_mib) { return { au::bytes(55.0), 3u, au::bytes(38.0), si::mebibytes(max_buffer_mib) }; }

TEST(DomainPlanner, ALatticeOverTheBufferLimitIsSplitAlongItsLongestHorizontalAxis) {
    const Domain domain = Domain::box(2.01 * m, 1.0 * m, 1.0 * m).cell_size(0.01 * m);
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
    const DomainPlan along_y = DomainPlanner::plan(Domain::box(1.0 * m, 2.01 * m, 1.0 * m).cell_size(0.01 * m), limited(40u));
    EXPECT_EQ(along_y.gpus.x, 1u);
    EXPECT_EQ(along_y.gpus.y, 2u);
    EXPECT_EQ(along_y.Ny, 200u);
}

TEST(DomainPlanner, ALatticeWithinTheLimitOrWithNoLimitKnownStaysWhole) {
    const Domain domain = Domain::box(2.01 * m, 1.0 * m, 1.0 * m).cell_size(0.01 * m);
    const DomainPlan within = DomainPlanner::plan(domain, limited(1000u));
    EXPECT_EQ(within.gpus.x, 1u);
    EXPECT_EQ(within.Nx, 201u);
    const DomainPlan unknown = DomainPlanner::plan(domain, limited(0u));
    EXPECT_EQ(unknown.gpus.x, 1u);
    EXPECT_EQ(unknown.Nx, 201u);
}

TEST(DomainPlanner, TheCallersSplitIsKeptAndRefusedOverTheBufferLimit) {
    const DomainPlan kept = DomainPlanner::plan(Domain::box(2.01 * m, 1.0 * m, 1.0 * m).cell_size(0.01 * m).gpus(1u, 2u, 1u), limited(1000u));
    EXPECT_EQ(kept.gpus.x, 1u);
    EXPECT_EQ(kept.gpus.y, 2u);
    // two domains along Y: 201 x 52 x 100 x 38 bytes = 38 MB, over 10 MB
    EXPECT_THROW(DomainPlanner::plan(Domain::box(2.01 * m, 1.0 * m, 1.0 * m).cell_size(0.01 * m).gpus(1u, 2u, 1u), limited(10u)), SetupError);
}

TEST(DomainPlanner, ALatticeNoSplitFitsIsRefused) {
    // the thinnest domain along X, 2 cells and 2 halo layers of 100 x 100 cells, is 1.5 MB, over a limit of 1 MB
    EXPECT_THROW(DomainPlanner::plan(Domain::box(2.01 * m, 1.0 * m, 1.0 * m).cell_size(0.01 * m), limited(1u)), SetupError);
}

// SCALAR_FV: the core gives a domain 2 halo layers on each side of a cut (LBM::LBM(), halo_width()), not 1
TEST(DomainPlanner, TheSplitCountsTheHaloLayersOfTheLattice) {
    const Domain domain = Domain::box(2.01 * m, 1.0 * m, 1.0 * m).cell_size(0.01 * m);
    LatticeMemory one_halo = limited(37u); // 38 797 312 bytes on one buffer
    LatticeMemory two_halos = limited(37u);
    two_halos.halo_layers = 2u;
    // two domains: 102 x 100 x 100 x 38 = 38 760 000 bytes fit with one halo layer, 104 x 100 x 100 x 38 = 39 520 000 do not
    EXPECT_EQ(DomainPlanner::plan(domain, one_halo).gpus.x, 2u);
    const DomainPlan plan = DomainPlanner::plan(domain, two_halos);
    EXPECT_EQ(plan.gpus.x, 3u); // 67 + 4 cells: 26 980 000 bytes
    EXPECT_EQ(plan.Nx, 201u);
}

// keep_levels_whole(): the planner's own split stays horizontal, and a caller's split along z is refused
TEST(DomainPlanner, KeepLevelsWholeRefusesASplitAlongZ) {
    const Domain domain = Domain::box(2.01 * m, 1.0 * m, 1.0 * m).cell_size(0.01 * m).keep_levels_whole();
    EXPECT_EQ(DomainPlanner::plan(domain, limited(40u)).gpus.z, 1u);
    EXPECT_NO_THROW(Domain::box(1.0 * m, 1.0 * m, 1.0 * m).cell_size(0.1 * m).gpus(2u, 1u, 1u).keep_levels_whole().validate());
    EXPECT_THROW(Domain::box(1.0 * m, 1.0 * m, 1.0 * m).cell_size(0.1 * m).gpus(1u, 1u, 2u).keep_levels_whole().validate(), SetupError);
    EXPECT_THROW(DomainPlanner::plan(Domain::box(1.0 * m, 1.0 * m, 1.0 * m).cell_size(0.1 * m).gpus(1u, 1u, 2u).keep_levels_whole(), d3q19_fp16), SetupError);
    EXPECT_NO_THROW(Domain::box(1.0 * m, 1.0 * m, 1.0 * m).cell_size(0.1 * m).gpus(1u, 1u, 2u).validate()); // without it a z split stays allowed
}

TEST(Domain, CellIsTheCellSizeSetting) {
    EXPECT_EQ(Domain::box(1.0 * m, 1.0 * m, 1.0 * m).cell_size(0.1 * m).cell()->in(m), 0.1);
    EXPECT_FALSE(Domain::box(1.0 * m, 1.0 * m, 1.0 * m).vram(100.0 * MiB).cell().has_value());
}

TEST(DomainPlanner, GpusAreInThePlan) {
    const DomainPlan plan = DomainPlanner::plan(Domain::box(1.0 * m, 2.0 * m, 1.0 * m).vram(100.0 * MiB).gpus(2u, 4u, 1u), d3q19_fp16);
    EXPECT_EQ(plan.gpus.x, 2u);
    EXPECT_EQ(plan.gpus.y, 4u);
    EXPECT_EQ(plan.gpus.z, 1u);
    EXPECT_THROW(Domain::box(1.0 * m, 1.0 * m, 1.0 * m).gpus(0u, 1u, 1u).validate(), SetupError);
}

} // namespace

namespace {

using namespace si::symbols;

// LEVEL_FORCING indexes its levels by the local z: a z split is refused at once, x and y splits run
TEST(DomainPlanner, LevelForcingRefusesASplitAlongZ) {
    TransportBuild forcing{};
    forcing.level_forcing = true;
    const DomainPlan along_z = DomainPlanner::plan(Domain::box(1.0 * m, 1.0 * m, 1.0 * m).cell_size(0.1 * m).gpus(1u, 1u, 2u), d3q19_fp16);
    const DomainPlan along_x = DomainPlanner::plan(Domain::box(1.0 * m, 1.0 * m, 1.0 * m).cell_size(0.1 * m).gpus(2u, 1u, 1u), d3q19_fp16);
    ASSERT_TRUE(DomainPlanner::level_forcing_refusal(along_z, forcing).has_value());
    EXPECT_NE(DomainPlanner::level_forcing_refusal(along_z, forcing)->find("gpus z 2"), std::string::npos);
    EXPECT_FALSE(DomainPlanner::level_forcing_refusal(along_x, forcing).has_value());
    EXPECT_FALSE(DomainPlanner::level_forcing_refusal(along_z, TransportBuild{}).has_value()); // without LEVEL_FORCING
}

// Regression for defect 21: with clearances() an STL becomes a cached SDF; when the SDF cannot be made (here: an STL
// that cannot be read), the planner refuses it with a SetupError instead of voxelizing the STL in its place.
TEST(DomainPlanner, AnSdfThatCannotBeMadeIsRefused) {
    const std::filesystem::path broken = test_file("fluidx3d_test_plan_broken.stl");
    {
        std::ofstream file(broken, std::ios::binary);
        file << "not an STL";
    }
    const Domain domain = Domain::around(Model(resource_name(broken.string()))).clearances(0.5 * m, 0.5 * m, 0.5 * m).cell_size(0.1 * m);
    DomainPlan plan{};
    plan.stl_path = broken.string();
    plan.base_grid = uint3(4u, 4u, 4u);
    EXPECT_THROW(DomainPlanner::choose_voxelization(plan, domain), SetupError);
    std::error_code ignored;
    std::filesystem::remove(broken, ignored);
}

} // namespace
