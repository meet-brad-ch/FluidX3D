#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/graphics/camera_view.hpp"
#include "setup/core/setup_error.hpp"

#include <cmath>

namespace {

const uint3 grid(100u, 200u, 50u); // cells of 0.1 m: a 10 m x 20 m x 5 m domain
constexpr float cell = 0.1f;

TEST(CameraView, OrbitShowsTheLongestSideByDefault) {
    const CameraPose pose = CameraView::orbit(-40_deg, 20_deg).field_of_view(78_deg).pose(grid, cell);
    EXPECT_FALSE(pose.free);
    EXPECT_FLOAT_EQ(pose.azimuth, -40.0f);
    EXPECT_FLOAT_EQ(pose.elevation, 20.0f);
    EXPECT_FLOAT_EQ(pose.field_of_view, 78.0f);
    EXPECT_FLOAT_EQ(pose.zoom, 1.0f);
}

TEST(CameraView, ViewHeightIsTheCoresZoom) {
    EXPECT_FLOAT_EQ(CameraView::orbit(0_deg, 0_deg).view_height(20_m).pose(grid, cell).zoom, 1.0f); // the longest side, Y
    EXPECT_FLOAT_EQ(CameraView::orbit(0_deg, 0_deg).view_height(8_m).pose(grid, cell).zoom, 2.5f);
}

TEST(CameraView, AtIsInMetresFromTheOriginCorner) {
    const CameraPose corner = CameraView::at({ 0_m, 0_m, 0_m }).pose(grid, cell);
    EXPECT_TRUE(corner.free);
    EXPECT_FLOAT_EQ(corner.position.x, -50.0f); // the core's coordinates are from the domain's center
    EXPECT_FLOAT_EQ(corner.position.y, -100.0f);
    EXPECT_FLOAT_EQ(corner.position.z, -25.0f);
    const CameraPose center = CameraView::at({ 5_m, 10_m, 2.5_m }, 90_deg, 45_deg).field_of_view(60_deg).pose(grid, cell);
    EXPECT_NEAR(center.position.x, 0.0f, 1e-4f);
    EXPECT_NEAR(center.position.y, 0.0f, 1e-4f);
    EXPECT_NEAR(center.position.z, 0.0f, 1e-4f);
    EXPECT_FLOAT_EQ(center.azimuth, 90.0f);
    EXPECT_FLOAT_EQ(center.elevation, 45.0f);
    EXPECT_FLOAT_EQ(center.field_of_view, 60.0f);
}

TEST(CameraView, LookAtTurnsTowardThePoint) {
    const Position target { 5_m, 10_m, 2.5_m };
    const CameraPose along_x = CameraView::at({ 15_m, 10_m, 2.5_m }).look_at(target).pose(grid, cell); // seen from +X
    EXPECT_NEAR(along_x.azimuth, 0.0f, 1e-4f);
    EXPECT_NEAR(along_x.elevation, 0.0f, 1e-4f);
    const CameraPose above = CameraView::at({ 5_m, 20_m, 12.5_m }).look_at(target).pose(grid, cell); // from +Y, 45 degrees up
    EXPECT_NEAR(above.azimuth, 90.0f, 1e-3f);
    EXPECT_NEAR(above.elevation, 45.0f, 1e-3f);
}

// The core points a camera along -Rz, with Rz the last row of the matrix of the core camera's update_matrix() (graphics.hpp) for
// rx = pi/2 + azimuth and ry = pi - elevation (LBM::Graphics::set_camera_free()): look_at() must give a Rz from the
// target to the camera.
TEST(CameraView, LookAtMatchesTheCoresViewDirection) {
    const Position camera_at { 2_m, 3_m, 4_m }, target { 7_m, 15_m, 1_m };
    const CameraPose pose = CameraView::at(camera_at).look_at(target).pose(grid, cell);
    const double rx = 0.5 * 3.14159265358979 + pose.azimuth * 3.14159265358979 / 180.0;
    const double ry = 3.14159265358979 - pose.elevation * 3.14159265358979 / 180.0;
    const double Rz[3] = { -std::sin(rx) * std::cos(ry), std::cos(rx) * std::cos(ry), std::sin(ry) };
    const double d[3] = { -5.0, -12.0, 3.0 }; // camera - target
    const double length = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    for(int i = 0; i < 3; i++) EXPECT_NEAR(Rz[i], d[i] / length, 1e-5);
}

TEST(CameraView, ContradictorySettingsThrow) {
    EXPECT_THROW(CameraView::orbit(0_deg, 0_deg).look_at({ 1_m, 1_m, 1_m }).pose(grid, cell), SetupError);
    EXPECT_THROW(CameraView::at({ 1_m, 1_m, 1_m }).view_height(1_m).pose(grid, cell), SetupError);
}

} // namespace
