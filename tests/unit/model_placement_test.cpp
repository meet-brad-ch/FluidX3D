#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/model_placement.hpp"
#include "setup/core/setup_error.hpp"
#include "box_stl.hpp"

#include <memory>

// the model 4 x 2 x 1 STL units, placed like voxelize_stl(): its longest side 40 cells, its center at (50, 50, 50)
class ModelPlacementTest : public ::testing::Test {
protected:
    static ModelPlacement placement(const BoxStl& model, const float3x3& rotation) {
        const std::unique_ptr<Mesh> rotated(read_stl(model.path(), 1.0f, rotation));
        return ModelPlacement(*rotated, rotation, 40.0f, float3(50.0f));
    }

    static void expect_bounds(const Mesh& mesh, const float3& pmin, const float3& pmax) {
        EXPECT_NEAR(mesh.pmin.x, pmin.x, 1e-4f); EXPECT_NEAR(mesh.pmin.y, pmin.y, 1e-4f); EXPECT_NEAR(mesh.pmin.z, pmin.z, 1e-4f);
        EXPECT_NEAR(mesh.pmax.x, pmax.x, 1e-4f); EXPECT_NEAR(mesh.pmax.y, pmax.y, 1e-4f); EXPECT_NEAR(mesh.pmax.z, pmax.z, 1e-4f);
    }
};

TEST_F(ModelPlacementTest, ScalesTheLongestSideAndCentersTheModel) {
    const BoxStl model("fluidx3d_test_placement_model.stl", 4.0f, 2.0f, 1.0f);
    const ModelPlacement p = placement(model, float3x3(1.0f));
    EXPECT_FLOAT_EQ(p.cells_per_unit(), 10.0f);
    expect_bounds(*p.load(model.path()), float3(30.0f, 40.0f, 45.0f), float3(70.0f, 60.0f, 55.0f));
}

TEST_F(ModelPlacementTest, PartsKeepTheirPlaceInTheAssembly) {
    const BoxStl model("fluidx3d_test_placement_body.stl", 4.0f, 2.0f, 1.0f);
    const BoxStl part("fluidx3d_test_placement_part.stl", 1.0f, 1.0f, 1.0f); // at the model's corner at the origin
    expect_bounds(*placement(model, float3x3(1.0f)).load(part.path()), float3(30.0f, 40.0f, 45.0f), float3(40.0f, 50.0f, 55.0f));
}

TEST_F(ModelPlacementTest, PartsTurnWithTheModel) {
    const BoxStl model("fluidx3d_test_placement_turned.stl", 4.0f, 2.0f, 1.0f);
    const BoxStl part("fluidx3d_test_placement_turned_part.stl", 1.0f, 1.0f, 1.0f);
    const ModelPlacement p = placement(model, float3x3(float3(0, 0, 1), radians(90.0f))); // x -> y, y -> -x
    expect_bounds(*p.load(model.path()), float3(40.0f, 30.0f, 45.0f), float3(60.0f, 70.0f, 55.0f));
    expect_bounds(*p.load(part.path()), float3(50.0f, 30.0f, 45.0f), float3(60.0f, 40.0f, 55.0f));
}

TEST_F(ModelPlacementTest, PartsInOtherCoordinatesAreCenteredOnTheModel) {
    const BoxStl model("fluidx3d_test_placement_center.stl", 4.0f, 2.0f, 1.0f);
    const BoxStl part("fluidx3d_test_placement_center_part.stl", 1.0f, 1.0f, 1.0f);
    const std::unique_ptr<Mesh> mesh = placement(model, float3x3(1.0f)).load_centered(part.path(), float3(0.0f, 5.0f, 0.0f));
    expect_bounds(*mesh, float3(45.0f, 50.0f, 45.0f), float3(55.0f, 60.0f, 55.0f)); // 10 cells, centered 5 cells up in y
}

TEST_F(ModelPlacementTest, APlanWithoutAModelThrows) {
    EXPECT_THROW(ModelPlacement::of(DomainPlan{}), SetupError);
}
