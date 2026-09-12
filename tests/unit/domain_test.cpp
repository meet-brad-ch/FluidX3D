#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/domain.hpp"
#include "setup/domain/domain_plan.hpp"
#include "setup/core/setup_error.hpp"
#include "box_stl.hpp"

#include <cmath>
#include <cstdint>

namespace {

constexpr LatticeMemory d3q19_fp16 { 55u };
constexpr LatticeMemory d3q19_fp16_surface { 67u };

TEST(Domain, AroundAModelKeepsTheClearances) {
    const BoxStl box(test_file("fluidx3d_test_domain_around.stl"), 4.0f, 2.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan plan = DomainPlanner::plan(
        Domain::around(Model(file).rotation(0_deg, 0_deg, 90_deg)).clearances(0.5_m, 1.0_m, 0.3_m).cell_size(0.1_m).max_vram(20_gb),
        d3q19_fp16);
    EXPECT_EQ(plan.Nx, 20u + 6u); // measured after the rotation: 2 m along X, 4 m along Y
    EXPECT_EQ(plan.Ny, 40u + 6u);
    EXPECT_EQ(plan.Nz, 10u + 5u + 10u);
    EXPECT_FLOAT_EQ(plan.center_lbm.x, 13.0f);
    EXPECT_FLOAT_EQ(plan.center_lbm.y, 23.0f);
    EXPECT_FLOAT_EQ(plan.center_lbm.z, 10.0f);
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, 40.0f); // along Y
}

TEST(Domain, VramBudgetAroundAModel) {
    const BoxStl box(test_file("fluidx3d_test_domain_vram.stl"), 4.0f, 2.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan plan = DomainPlanner::plan(Domain::around(Model(file)).clearances(0.0_m, 1.0_m, 2.6_m).vram(100_mb), d3q19_fp16);
    const std::uint64_t used_mb = required_memory_mb({ plan.Nx, plan.Ny, plan.Nz }, d3q19_fp16);
    EXPECT_LE(used_mb, 102u); // whole cells: rounding the clearances can add a little
    EXPECT_GE(used_mb, 90u);
}

TEST(Domain, ConflictingSettingsThrow) {
    EXPECT_THROW(Domain::box(1_m, 1_m, 1_m).clearances(0_m, 0_m, 0_m).validate(), SetupError);
    EXPECT_THROW(Domain::box(1_m, 1_m, 1_m).vram(1_gb).max_vram(2_gb).validate(), SetupError);
    EXPECT_THROW(Domain::around(Model("any.stl")).vram(1_gb).cell_size(1_cm).validate(), SetupError);
    EXPECT_NO_THROW(Domain::box(1_m, 1_m, 1_m).vram(1_gb).validate());
}

// A cell size gives whole cells along each side, as an original example's fixed grid (dam_break: 128 x 256 x 256).
TEST(Domain, BoxWithACellSizeHasWholeCells) {
    const DomainPlan plan = DomainPlanner::plan(Domain::box(0.5_m, 1.0_m, 1.0_m).cell_size(1.0_m / 256.0f), d3q19_fp16_surface);
    EXPECT_EQ(plan.Nx, 128u);
    EXPECT_EQ(plan.Ny, 256u);
    EXPECT_EQ(plan.Nz, 256u);
    // 8.4 M cells need 536 MB
    EXPECT_THROW(DomainPlanner::plan(Domain::box(0.5_m, 1.0_m, 1.0_m).cell_size(1.0_m / 256.0f).max_vram(100_mb), d3q19_fp16_surface), SetupError);
    EXPECT_THROW(DomainPlanner::plan(Domain::box(0.5_m, 1.0_m, 1.0_m).cell_size(2_m), d3q19_fp16_surface), SetupError);
}

TEST(Domain, SizeWithACellSizeHasWholeCells) {
    const BoxStl box(test_file("fluidx3d_test_domain_cells.stl"), 4.0f, 2.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan plan = DomainPlanner::plan(Domain::around(Model(file).length(1.0_m)).size(2_m, 4_m, 1_m).cell_size(0.05_m), d3q19_fp16);
    EXPECT_EQ(plan.Nx, 40u);
    EXPECT_EQ(plan.Ny, 80u);
    EXPECT_EQ(plan.Nz, 20u);
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, 20.0f); // the model's 1 m along Y
}

// on_floor(): the model's bottom one cell above z = 0 at any resolution, as the originals placed it (pmin.z = 1).
TEST(Domain, OnFloorPutsTheModelOneCellAboveZeroZ) {
    const BoxStl box(test_file("fluidx3d_test_domain_floor.stl"), 4.0f, 2.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan plan = DomainPlanner::plan(
        Domain::around(Model(file).length(1.2_m)).size(1.2_m, 2.4_m, 1.2_m).on_floor().vram(100_mb), d3q19_fp16);
    const float half_height = 0.5f * 0.5f * plan.lbm_reference_size; // 1 of the box's 2 STL units along Y, halved
    EXPECT_NEAR(plan.center_lbm.z, 1.0f + half_height, 1e-3f);
    EXPECT_THROW(Domain::around(Model(file).length(1.2_m)).size(1.2_m, 2.4_m, 1.2_m).on_floor().gap_to_floor(1_cm).validate(), SetupError);
    EXPECT_THROW(Domain::box(1_m, 1_m, 1_m).on_floor().validate(), SetupError); // no model to place
}

// Model::length() is the extent along its axis. The core's voxelizer scales the longest side, so the plan converts:
// a 4 x 2 x 1 box 1 m long along Y is 2 m long along X, its longest side.
TEST(Domain, LengthIsTheExtentAlongItsAxis) {
    const BoxStl box(test_file("fluidx3d_test_domain_axis.stl"), 4.0f, 2.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan along_y = DomainPlanner::plan(Domain::around(Model(file).length(1.0_m, Axis::Y)).size(4_m, 4_m, 2_m).vram(100_mb), d3q19_fp16);
    EXPECT_FLOAT_EQ(along_y.lbm_reference_size, 0.25f * (float)along_y.Ny); // 1 m of the 4 m domain length
    EXPECT_NEAR(along_y.voxel_size, 2.0f * along_y.lbm_reference_size, 1e-3f);
    EXPECT_FLOAT_EQ(along_y.si_reference_size, 1.0f);
    const DomainPlan along_x = DomainPlanner::plan(Domain::around(Model(file).length(2.0_m, Axis::X)).size(4_m, 4_m, 2_m).vram(100_mb), d3q19_fp16);
    EXPECT_EQ(along_x.voxel_size, along_x.lbm_reference_size); // the longest side: exactly the core's size
    EXPECT_NEAR(along_x.voxel_size, along_y.voxel_size, 1e-3f);  // the same model size either way
}

// The angle of attack tilts the model after it is measured: a 1 x 4 x 1 box, 4 m long along Y, pitched by 30 degrees has
// a longest side of 4 cos 30 + 1 sin 30 = 3.96 m, which the voxelizer gets.
TEST(Domain, LengthIsMeasuredBeforeTheAngleOfAttack) {
    const BoxStl box(test_file("fluidx3d_test_domain_pitch.stl"), 1.0f, 4.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan plan = DomainPlanner::plan(Domain::around(Model(file).angle_of_attack(30_deg).length(4.0_m)).size(8_m, 8_m, 8_m).vram(100_mb), d3q19_fp16);
    const float pitched = 4.0f * std::cos(0.5235988f) + 1.0f * std::sin(0.5235988f);
    EXPECT_NEAR(plan.voxel_size, plan.lbm_reference_size * pitched / 4.0f, 1e-2f);
}

TEST(Domain, SizeAndPlacementNeedTheirCounterparts) {
    EXPECT_THROW(Domain::around(Model("any.stl")).size(1_m, 2_m, 1_m).validate(), SetupError); // no Model::length()
    EXPECT_THROW(Domain::around(Model("any.stl").length(1_m)).size(1_m, 2_m, 1_m).clearances(0_m, 0_m, 0_m).validate(), SetupError);
    EXPECT_THROW(Domain::around(Model("any.stl").length(1_m)).size(1_m, 2_m, 1_m).model_offset(0_m, 0_m, 0_m).gap_to_inlet(1_m).validate(), SetupError);
    EXPECT_THROW(Domain::around(Model("any.stl").length(1_m)).clearances(0_m, 0_m, 0_m).validate(), SetupError); // length() is for size()
    EXPECT_THROW(Domain::around(Model("any.stl")).clearances(0_m, 0_m, 0_m).gap_to_inlet(1_m).validate(), SetupError); // placement needs size()
    EXPECT_THROW(Domain::box(1_m, 1_m, 1_m).model_offset(0_m, 0_m, 0_m).validate(), SetupError); // no model to place
    EXPECT_NO_THROW(Domain::around(Model("any.stl").length(1_m)).size(1_m, 2_m, 1_m).gap_to_inlet(0.1_m).validate());
}

TEST(Model, KnowsAnSdfByItsExtension) {
    EXPECT_TRUE(Model("sdf_cache/cow_128x428x258.SDF").is_sdf());
    EXPECT_FALSE(Model("Cow_t.stl").is_sdf());
    EXPECT_FALSE(Model("no_extension").is_sdf());
}

} // namespace
