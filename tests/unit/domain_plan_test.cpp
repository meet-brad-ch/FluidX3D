#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/domain_plan.hpp"
#include "box_stl.hpp"

#include <filesystem>
#include <string>

namespace {

constexpr LatticeMemory d3q19_fp16 { 55u };
constexpr LatticeMemory d3q19_fp16_surface { 67u };

// SimulationConfig looks geometry files up in resources/ (and exits if they are missing), so the test files are
// written to the test build directory and named relative to resources/.
std::filesystem::path test_file(const char* name) { return std::filesystem::path(FLUIDX3D_TEST_DIR) / name; }
std::string resource_name(const std::string& path) {
    return std::filesystem::relative(path, FLUIDX3D_RESOURCE_DIR).generic_string();
}

TEST(DomainPlanner, DomainOnlySizesTheBoxFromTheVramBudget) {
    // breaking_waves: 1 m x 5 m x 0.75 m in 2000 MB with FP16 and SURFACE
    const DomainPlan plan = DomainPlanner::plan(SimulationConfig().set_domain_size_m(1.0f, 5.0f, 0.75f).set_vram_mb(2000u), d3q19_fp16_surface);
    EXPECT_EQ(plan.Nx, 203u);
    EXPECT_EQ(plan.Ny, 1014u);
    EXPECT_EQ(plan.Nz, 152u);
    EXPECT_FLOAT_EQ(plan.si_reference_size, 5.0f);      // the longest side
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, 1014.0f);
    EXPECT_FLOAT_EQ(plan.center_lbm.x, 101.5f);
    EXPECT_FLOAT_EQ(plan.center_lbm.y, 507.0f);
    EXPECT_FLOAT_EQ(plan.center_lbm.z, 76.0f);
    EXPECT_TRUE(plan.stl_path.empty());
}

TEST(DomainPlanner, GeometryBasedAddsTheClearancesAroundTheModel) {
    const BoxStl box(test_file("fluidx3d_test_plan_geometry.stl"), 4.0f, 2.0f, 1.0f);
    const DomainPlan plan = DomainPlanner::plan(SimulationConfig(resource_name(box.path()))
        .set_voxel_size_m(0.1f)
        .set_max_vram_mb(20000u)
        .set_clearances_m(0.5f, 1.0f, 0.3f), d3q19_fp16);
    EXPECT_EQ(plan.Nx, 40u + 2u * 3u);
    EXPECT_EQ(plan.Ny, 20u + 2u * 3u);
    EXPECT_EQ(plan.Nz, 10u + 5u + 10u);
    EXPECT_FLOAT_EQ(plan.center_lbm.x, 23.0f);
    EXPECT_FLOAT_EQ(plan.center_lbm.y, 13.0f);
    EXPECT_FLOAT_EQ(plan.center_lbm.z, 10.0f); // resting on the 5-cell bottom clearance
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, 20.0f); // reference axis Y
    EXPECT_NEAR(plan.si_reference_size, 2.0f, 1e-5f);
    EXPECT_FLOAT_EQ(plan.voxel_size, 40.0f);         // the longest side
    EXPECT_TRUE(std::filesystem::exists(plan.stl_path));
}

TEST(DomainPlanner, AspectRatioCentersTheModelWithAnOffset) {
    const BoxStl box(test_file("fluidx3d_test_plan_center.stl"), 4.0f, 2.0f, 1.0f);
    const DomainPlan plan = DomainPlanner::plan(SimulationConfig(resource_name(box.path()))
        .set_domain_aspect_ratio(1.0f, 2.0f, 1.0f)
        .set_vram_mb(100u)
        .set_geometry_scale(0.5f)
        .set_center_offset_ratio(0.0f, 0.1f, 0.0f), d3q19_fp16);
    const GridSize grid = grid_for_memory(1.0f, 2.0f, 1.0f, 100u, d3q19_fp16);
    EXPECT_EQ(plan.Nx, grid.x);
    EXPECT_EQ(plan.Ny, grid.y);
    EXPECT_EQ(plan.Nz, grid.z);
    const float length = 0.5f * (float)grid.y; // geometry length: half of the reference axis Y
    EXPECT_FLOAT_EQ(plan.lbm_reference_size, length);
    EXPECT_FLOAT_EQ(plan.si_reference_size, 1.0f); // no SI length in this mode until configure_units_with_length()
    EXPECT_NEAR(plan.center_lbm.x, 0.5f * (float)grid.x, 1e-4f);
    EXPECT_NEAR(plan.center_lbm.y, 0.5f * (float)grid.y + 0.1f * length, 1e-3f);
    EXPECT_NEAR(plan.center_lbm.z, 0.5f * (float)grid.z, 1e-4f);
    EXPECT_FLOAT_EQ(plan.voxel_size, length);
}

TEST(DomainPlanner, AspectRatioPlacesTheBoundingBoxMinimum) {
    const BoxStl box(test_file("fluidx3d_test_plan_pmin.stl"), 4.0f, 2.0f, 1.0f);
    const DomainPlan plan = DomainPlanner::plan(SimulationConfig(resource_name(box.path()))
        .set_domain_aspect_ratio(1.0f, 2.0f, 1.0f)
        .set_vram_mb(100u)
        .set_geometry_scale(0.5f)
        .set_pmin_offset_ratio(0.3f, 0.1f, 0.02f), d3q19_fp16);
    const GridSize grid = grid_for_memory(1.0f, 2.0f, 1.0f, 100u, d3q19_fp16);
    const float length = 0.5f * (float)grid.y;
    const float cells_per_m = length / 2.0f; // the model is 2 m long along Y
    EXPECT_EQ(plan.base_grid.x, (uint32_t)(cells_per_m * 4.0f + 0.5f));
    EXPECT_EQ(plan.base_grid.y, (uint32_t)(cells_per_m * 2.0f + 0.5f));
    EXPECT_EQ(plan.base_grid.z, (uint32_t)(cells_per_m * 1.0f + 0.5f));
    // the bounding box minimum sits at 0.1 and 0.02 model lengths from the inlet and the floor;
    // X stays centered: the X offset (0.3) is ignored (review A7)
    EXPECT_NEAR(plan.center_lbm.x, 0.5f * (float)grid.x, 1e-4f);
    EXPECT_NEAR(plan.center_lbm.y, 0.1f * length + 0.5f * cells_per_m * 2.0f, 1e-3f);
    EXPECT_NEAR(plan.center_lbm.z, 0.02f * length + 0.5f * cells_per_m * 1.0f, 1e-3f);
}

} // namespace
