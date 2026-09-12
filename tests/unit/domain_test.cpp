#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/domain.hpp"
#include "setup/domain/domain_plan.hpp"
#include "setup/core/setup_error.hpp"
#include "box_stl.hpp"

#include <cmath>

namespace {

constexpr LatticeMemory d3q19_fp16 { 55u };
constexpr LatticeMemory d3q19_fp16_surface { 67u };

void expect_same_plan(const DomainPlan& a, const DomainPlan& b) {
    EXPECT_EQ(a.Nx, b.Nx);
    EXPECT_EQ(a.Ny, b.Ny);
    EXPECT_EQ(a.Nz, b.Nz);
    EXPECT_EQ(a.center_lbm.x, b.center_lbm.x);
    EXPECT_EQ(a.center_lbm.y, b.center_lbm.y);
    EXPECT_EQ(a.center_lbm.z, b.center_lbm.z);
    EXPECT_EQ(a.lbm_reference_size, b.lbm_reference_size);
    EXPECT_EQ(a.si_reference_size, b.si_reference_size);
    const float3x3& ra = a.rotation_matrix;
    const float3x3& rb = b.rotation_matrix;
    EXPECT_EQ(ra.xx, rb.xx); EXPECT_EQ(ra.xy, rb.xy); EXPECT_EQ(ra.xz, rb.xz);
    EXPECT_EQ(ra.yx, rb.yx); EXPECT_EQ(ra.yy, rb.yy); EXPECT_EQ(ra.yz, rb.yz);
    EXPECT_EQ(ra.zx, rb.zx); EXPECT_EQ(ra.zy, rb.zy); EXPECT_EQ(ra.zz, rb.zz);
}

TEST(Domain, BoxPlansLikeADomainSizeInMetres) {
    const DomainPlan from_domain = DomainPlanner::plan(Domain::box(1.0_m, 5.0_m, 0.75_m).vram(2000_mb).config(), d3q19_fp16_surface);
    const DomainPlan from_config = DomainPlanner::plan(SimulationConfig().set_domain_size_m(1.0f, 5.0f, 0.75f).set_vram_mb(2000u), d3q19_fp16_surface);
    expect_same_plan(from_domain, from_config);
    EXPECT_EQ(from_domain.Ny, 1014u); // breaking_waves
}

TEST(Domain, AroundAModelKeepsTheClearances) {
    const BoxStl box(test_file("fluidx3d_test_domain_around.stl"), 4.0f, 2.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan from_domain = DomainPlanner::plan(
        Domain::around(Model(file).rotation(0_deg, 0_deg, 90_deg)).clearances(0.5_m, 1.0_m, 0.3_m).cell_size(0.1_m).max_vram(20_gb).config(),
        d3q19_fp16);
    const DomainPlan from_config = DomainPlanner::plan(
        SimulationConfig(file).set_rotation_deg(0.0f, 0.0f, 90.0f).set_clearances_m(0.5f, 1.0f, 0.3f).set_voxel_size_m(0.1f).set_max_vram_mb(20480u),
        d3q19_fp16);
    expect_same_plan(from_domain, from_config);
    EXPECT_EQ(from_domain.Nx, 20u + 6u); // measured after the rotation: 2 m along X, 4 m along Y
    EXPECT_EQ(from_domain.Ny, 40u + 6u);
    EXPECT_EQ(from_domain.Nz, 10u + 5u + 10u);
}

TEST(Domain, VramBudgetAroundAModel) {
    const BoxStl box(test_file("fluidx3d_test_domain_vram.stl"), 4.0f, 2.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan from_domain = DomainPlanner::plan(Domain::around(Model(file)).clearances(0.0_m, 1.0_m, 2.6_m).vram(100_mb).config(), d3q19_fp16);
    const DomainPlan from_config = DomainPlanner::plan(SimulationConfig(file).set_clearances_m(0.0f, 1.0f, 2.6f).set_vram_mb(100u), d3q19_fp16);
    expect_same_plan(from_domain, from_config);
}

TEST(Domain, ConflictingSettingsThrow) {
    EXPECT_THROW(Domain::box(1_m, 1_m, 1_m).clearances(0_m, 0_m, 0_m).config(), SetupError);
    EXPECT_THROW(Domain::box(1_m, 1_m, 1_m).vram(1_gb).max_vram(2_gb).config(), SetupError);
    EXPECT_THROW(Domain::around(Model("any.stl")).vram(1_gb).cell_size(1_cm).config(), SetupError);
    EXPECT_NO_THROW(Domain::box(1_m, 1_m, 1_m).vram(1_gb).config());
}

// A cell size gives whole cells along each side, as an original example's fixed grid (dam_break: 128 x 256 x 256).
TEST(Domain, BoxWithACellSizeHasWholeCells) {
    const DomainPlan plan = DomainPlanner::plan(Domain::box(0.5_m, 1.0_m, 1.0_m).cell_size(1.0_m / 256.0f).config(), d3q19_fp16_surface);
    EXPECT_EQ(plan.Nx, 128u);
    EXPECT_EQ(plan.Ny, 256u);
    EXPECT_EQ(plan.Nz, 256u);
    // 8.4 M cells need 536 MB
    EXPECT_THROW(DomainPlanner::plan(Domain::box(0.5_m, 1.0_m, 1.0_m).cell_size(1.0_m / 256.0f).max_vram(100_mb).config(), d3q19_fp16_surface), SetupError);
    EXPECT_THROW(DomainPlanner::plan(Domain::box(0.5_m, 1.0_m, 1.0_m).cell_size(2_m).config(), d3q19_fp16_surface), SetupError);
}

TEST(Domain, SizeWithACellSizeHasWholeCells) {
    const BoxStl box(test_file("fluidx3d_test_domain_cells.stl"), 4.0f, 2.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan plan = DomainPlanner::plan(Domain::around(Model(file).length(1.0_m)).size(2_m, 4_m, 1_m).cell_size(0.05_m).config(), d3q19_fp16);
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
        Domain::around(Model(file).length(1.2_m)).size(1.2_m, 2.4_m, 1.2_m).on_floor().vram(100_mb).config(), d3q19_fp16);
    const float half_height = 0.5f * 0.5f * plan.lbm_reference_size; // 1 of the box's 2 STL units along Y, halved
    EXPECT_NEAR(plan.center_lbm.z, 1.0f + half_height, 1e-3f);
    EXPECT_THROW(Domain::around(Model(file).length(1.2_m)).size(1.2_m, 2.4_m, 1.2_m).on_floor().gap_to_floor(1_cm).config(), SetupError);
    EXPECT_THROW(Domain::box(1_m, 1_m, 1_m).on_floor().config(), SetupError); // no model to place
}

// A size in metres around a model of known length plans like the aspect ratio and geometry scale it stands for.
TEST(Domain, SizeInMetresPlansLikeTheAspectRatio) {
    const BoxStl box(test_file("fluidx3d_test_domain_size.stl"), 4.0f, 2.0f, 1.0f); // STL units do not matter here
    const std::string file = resource_name(box.path());
    // model 1.2 m long along Y in a 1.2 m x 2.4 m x 1.2 m domain: aspect 1:2:1, geometry scale 0.5
    const DomainPlan from_domain = DomainPlanner::plan(
        Domain::around(Model(file).length(1.2_m)).size(1.2_m, 2.4_m, 1.2_m).model_offset(0_m, -0.12_m, 0_m).vram(100_mb).config(),
        d3q19_fp16);
    const DomainPlan from_config = DomainPlanner::plan(
        SimulationConfig(file).set_domain_aspect_ratio(1.0f, 2.0f, 1.0f).set_geometry_scale(0.5f).set_center_offset_ratio(0.0f, -0.1f, 0.0f).set_vram_mb(100u),
        d3q19_fp16);
    EXPECT_EQ(from_domain.Nx, from_config.Nx);
    EXPECT_EQ(from_domain.Ny, from_config.Ny);
    EXPECT_EQ(from_domain.Nz, from_config.Nz);
    EXPECT_FLOAT_EQ(from_domain.lbm_reference_size, from_config.lbm_reference_size);
    EXPECT_NEAR(from_domain.center_lbm.y, from_config.center_lbm.y, 1e-3f);
    EXPECT_FLOAT_EQ(from_domain.si_reference_size, 1.2f); // the model's real length, not the hidden 1 m (review A11)
    EXPECT_FLOAT_EQ(from_config.si_reference_size, 1.0f);
}

TEST(Domain, GapsPlaceTheModelsFrontAndBottom) {
    const BoxStl box(test_file("fluidx3d_test_domain_gaps.stl"), 4.0f, 2.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan from_domain = DomainPlanner::plan(
        Domain::around(Model(file).length(1.2_m)).size(1.2_m, 2.4_m, 1.2_m).gap_to_inlet(0.12_m).gap_to_floor(0.012_m).vram(100_mb).config(),
        d3q19_fp16);
    const DomainPlan from_config = DomainPlanner::plan(
        SimulationConfig(file).set_domain_aspect_ratio(1.0f, 2.0f, 1.0f).set_geometry_scale(0.5f).set_pmin_offset_ratio(0.0f, 0.1f, 0.01f).set_vram_mb(100u),
        d3q19_fp16);
    EXPECT_NEAR(from_domain.center_lbm.x, from_config.center_lbm.x, 1e-3f);
    EXPECT_NEAR(from_domain.center_lbm.y, from_config.center_lbm.y, 1e-3f);
    EXPECT_NEAR(from_domain.center_lbm.z, from_config.center_lbm.z, 1e-3f);
    EXPECT_EQ(from_domain.base_grid.y, from_config.base_grid.y);
}

// Model::length() is the extent along its axis. The core's voxelizer scales the longest side, so the plan converts:
// a 4 x 2 x 1 box 1 m long along Y is 2 m long along X, its longest side.
TEST(Domain, LengthIsTheExtentAlongItsAxis) {
    const BoxStl box(test_file("fluidx3d_test_domain_axis.stl"), 4.0f, 2.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan along_y = DomainPlanner::plan(Domain::around(Model(file).length(1.0_m, Axis::Y)).size(4_m, 4_m, 2_m).vram(100_mb).config(), d3q19_fp16);
    EXPECT_FLOAT_EQ(along_y.lbm_reference_size, 0.25f * (float)along_y.Ny); // 1 m of the 4 m domain length
    EXPECT_NEAR(along_y.voxel_size, 2.0f * along_y.lbm_reference_size, 1e-3f);
    EXPECT_FLOAT_EQ(along_y.si_reference_size, 1.0f);
    const DomainPlan along_x = DomainPlanner::plan(Domain::around(Model(file).length(2.0_m, Axis::X)).size(4_m, 4_m, 2_m).vram(100_mb).config(), d3q19_fp16);
    EXPECT_EQ(along_x.voxel_size, along_x.lbm_reference_size); // the longest side: exactly the core's size
    EXPECT_NEAR(along_x.voxel_size, along_y.voxel_size, 1e-3f);  // the same model size either way
}

// The angle of attack tilts the model after it is measured: a 1 x 4 x 1 box, 4 m long along Y, pitched by 30 degrees has
// a longest side of 4 cos 30 + 1 sin 30 = 3.96 m, which the voxelizer gets.
TEST(Domain, LengthIsMeasuredBeforeTheAngleOfAttack) {
    const BoxStl box(test_file("fluidx3d_test_domain_pitch.stl"), 1.0f, 4.0f, 1.0f);
    const std::string file = resource_name(box.path());
    const DomainPlan plan = DomainPlanner::plan(Domain::around(Model(file).angle_of_attack(30_deg).length(4.0_m)).size(8_m, 8_m, 8_m).vram(100_mb).config(), d3q19_fp16);
    const float pitched = 4.0f * std::cos(0.5235988f) + 1.0f * std::sin(0.5235988f);
    EXPECT_NEAR(plan.voxel_size, plan.lbm_reference_size * pitched / 4.0f, 1e-2f);
}

TEST(Domain, SizeAndPlacementNeedTheirCounterparts) {
    EXPECT_THROW(Domain::around(Model("any.stl")).size(1_m, 2_m, 1_m).config(), SetupError); // no Model::length()
    EXPECT_THROW(Domain::around(Model("any.stl").length(1_m)).size(1_m, 2_m, 1_m).clearances(0_m, 0_m, 0_m).config(), SetupError);
    EXPECT_THROW(Domain::around(Model("any.stl").length(1_m)).size(1_m, 2_m, 1_m).model_offset(0_m, 0_m, 0_m).gap_to_inlet(1_m).config(), SetupError);
    EXPECT_THROW(Domain::around(Model("any.stl").length(1_m)).clearances(0_m, 0_m, 0_m).config(), SetupError); // length() is for size()
    EXPECT_THROW(Domain::around(Model("any.stl")).clearances(0_m, 0_m, 0_m).gap_to_inlet(1_m).config(), SetupError); // placement needs size()
    EXPECT_THROW(Domain::box(1_m, 1_m, 1_m).model_offset(0_m, 0_m, 0_m).config(), SetupError); // no model to place
    EXPECT_NO_THROW(Domain::around(Model("any.stl").length(1_m)).size(1_m, 2_m, 1_m).gap_to_inlet(0.1_m).config());
}

} // namespace
