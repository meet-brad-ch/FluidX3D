#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/domain.hpp"
#include "setup/domain/domain_plan.hpp"
#include "setup/core/setup_error.hpp"
#include "box_stl.hpp"

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
    EXPECT_THROW(Domain::box(1_m, 1_m, 1_m).cell_size(1_cm).config(), SetupError);
    EXPECT_THROW(Domain::box(1_m, 1_m, 1_m).vram(1_gb).max_vram(2_gb).config(), SetupError);
    EXPECT_THROW(Domain::around(Model("any.stl")).vram(1_gb).cell_size(1_cm).config(), SetupError);
    EXPECT_NO_THROW(Domain::box(1_m, 1_m, 1_m).vram(1_gb).config());
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

TEST(Domain, SizeAndPlacementNeedTheirCounterparts) {
    EXPECT_THROW(Domain::around(Model("any.stl")).size(1_m, 2_m, 1_m).config(), SetupError); // no Model::length()
    EXPECT_THROW(Domain::around(Model("any.stl").length(1_m)).size(1_m, 2_m, 1_m).clearances(0_m, 0_m, 0_m).config(), SetupError);
    EXPECT_THROW(Domain::around(Model("any.stl").length(1_m)).size(1_m, 2_m, 1_m).cell_size(1_cm).config(), SetupError);
    EXPECT_THROW(Domain::around(Model("any.stl").length(1_m)).size(1_m, 2_m, 1_m).model_offset(0_m, 0_m, 0_m).gap_to_inlet(1_m).config(), SetupError);
    EXPECT_THROW(Domain::around(Model("any.stl").length(1_m)).clearances(0_m, 0_m, 0_m).config(), SetupError); // length() is for size()
    EXPECT_THROW(Domain::around(Model("any.stl")).clearances(0_m, 0_m, 0_m).gap_to_inlet(1_m).config(), SetupError); // placement needs size()
    EXPECT_THROW(Domain::box(1_m, 1_m, 1_m).model_offset(0_m, 0_m, 0_m).config(), SetupError); // no model to place
    EXPECT_NO_THROW(Domain::around(Model("any.stl").length(1_m)).size(1_m, 2_m, 1_m).gap_to_inlet(0.1_m).config());
}

} // namespace
