#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/simulation/mesh_loader.hpp"
#include "box_stl.hpp"

#include <memory>

namespace {

/// The planned centre of the whole model in the core's cell coordinates.
const float3 planned_centre{ 50.0f, 40.0f, 30.0f };
/// The half model's longest side in cells after the planner's scale.
constexpr float half_length_cells = 20.0f;

/// @brief Gives the plan of a half model in an STL file: no rotation, the longest side scaled to half_length_cells,
/// the whole model centred at planned_centre.
/// @param stl_path the half model's file
/// @return the plan
DomainPlan half_model_plan(const std::string& stl_path) {
    DomainPlan plan{};
    plan.stl_path = stl_path;
    plan.rotation_matrix = float3x3(1.0f);
    plan.lbm_reference_size = half_length_cells;
    plan.center_lbm = planned_centre;
    return plan;
}

// Regression for defect 6: the half model of a CAD file usually does not have its centre at the STL origin (here a
// box from the origin to (2, 1, 0.5) m). The two halves must meet at the symmetry plane and the whole model must
// sit at the planned centre: along x two halves of 20 cells, 30 to 70; along y 10 cells, 35 to 45; along z 5 cells,
// 27.5 to 32.5.
TEST(MeshLoader, AnOffCentreHalfModelIsMirroredAtItsPlane) {
    const BoxStl half("fluidx3d_test_mirrored_half.stl", 2.0f, 1.0f, 0.5f);
    const DomainPlan plan = half_model_plan(half.path());
    const std::unique_ptr<Mesh> whole = MeshLoader::load_mirrored(plan, Axis::X);
    whole->find_bounds();
    EXPECT_EQ(whole->triangle_number, 24u);
    EXPECT_NEAR(whole->pmin.x, 30.0f, 1e-4f);
    EXPECT_NEAR(whole->pmax.x, 70.0f, 1e-4f);
    EXPECT_NEAR(whole->pmin.y, 35.0f, 1e-4f);
    EXPECT_NEAR(whole->pmax.y, 45.0f, 1e-4f);
    EXPECT_NEAR(whole->pmin.z, 27.5f, 1e-4f);
    EXPECT_NEAR(whole->pmax.z, 32.5f, 1e-4f);
}

} // namespace
