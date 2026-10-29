#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/graphics/camera_view.hpp"
#include "setup/core/setup_error.hpp"

#include <cmath>

namespace {

using namespace si::symbols;

const uint3 grid(100u, 200u, 50u); // cells of 0.1 m: a 10 m x 20 m x 5 m domain
const si::Length cell = static_cast<double>(0.1f) * si::symbols::m; // the core's float cell of 0.1 m, as the builders pass it

TEST(CameraView, OrbitShowsTheLongestSideByDefault) {
    const CameraPose pose = CameraView::orbit(-40.0 * deg, 20.0 * deg).field_of_view(78.0 * deg).pose(grid, cell);
    EXPECT_FALSE(pose.free);
    EXPECT_FLOAT_EQ(pose.azimuth, -40.0f);
    EXPECT_FLOAT_EQ(pose.elevation, 20.0f);
    EXPECT_FLOAT_EQ(pose.field_of_view, 78.0f);
    EXPECT_FLOAT_EQ(pose.zoom, 1.0f);
}

TEST(CameraView, ViewHeightIsTheCoresZoom) {
    EXPECT_FLOAT_EQ(CameraView::orbit(0.0 * deg, 0.0 * deg).view_height(20.0 * m).pose(grid, cell).zoom, 1.0f); // the longest side, Y
    EXPECT_FLOAT_EQ(CameraView::orbit(0.0 * deg, 0.0 * deg).view_height(8.0 * m).pose(grid, cell).zoom, 2.5f);
}

TEST(CameraView, AtIsInMetresFromTheOriginCorner) {
    const CameraPose corner = CameraView::at({ 0.0 * m, 0.0 * m, 0.0 * m }).pose(grid, cell);
    EXPECT_TRUE(corner.free);
    EXPECT_FLOAT_EQ(corner.position.x, -50.0f); // the core's coordinates are from the domain's center
    EXPECT_FLOAT_EQ(corner.position.y, -100.0f);
    EXPECT_FLOAT_EQ(corner.position.z, -25.0f);
    const CameraPose center = CameraView::at({ 5.0 * m, 10.0 * m, 2.5 * m }, 90.0 * deg, 45.0 * deg).field_of_view(60.0 * deg).pose(grid, cell);
    EXPECT_NEAR(center.position.x, 0.0f, 1e-4f);
    EXPECT_NEAR(center.position.y, 0.0f, 1e-4f);
    EXPECT_NEAR(center.position.z, 0.0f, 1e-4f);
    EXPECT_FLOAT_EQ(center.azimuth, 90.0f);
    EXPECT_FLOAT_EQ(center.elevation, 45.0f);
    EXPECT_FLOAT_EQ(center.field_of_view, 60.0f);
}

TEST(CameraView, LookAtTurnsTowardThePoint) {
    const si::Position target { 5.0 * m, 10.0 * m, 2.5 * m };
    const CameraPose along_x = CameraView::at({ 15.0 * m, 10.0 * m, 2.5 * m }).look_at(target).pose(grid, cell); // seen from +X
    EXPECT_NEAR(along_x.azimuth, 0.0f, 1e-4f);
    EXPECT_NEAR(along_x.elevation, 0.0f, 1e-4f);
    const CameraPose above = CameraView::at({ 5.0 * m, 20.0 * m, 12.5 * m }).look_at(target).pose(grid, cell); // from +Y, 45 degrees up
    EXPECT_NEAR(above.azimuth, 90.0f, 1e-3f);
    EXPECT_NEAR(above.elevation, 45.0f, 1e-3f);
}

// The core points a camera along -Rz, with Rz the last row of the matrix of the core camera's update_matrix() (graphics.hpp) for
// rx = pi/2 + azimuth and ry = pi - elevation (LBM::Graphics::set_camera_free()): look_at() must give a Rz from the
// target to the camera.
TEST(CameraView, LookAtMatchesTheCoresViewDirection) {
    const si::Position camera_at { 2.0 * m, 3.0 * m, 4.0 * m }, target { 7.0 * m, 15.0 * m, 1.0 * m };
    const CameraPose pose = CameraView::at(camera_at).look_at(target).pose(grid, cell);
    const double rx = 0.5 * 3.14159265358979 + pose.azimuth * 3.14159265358979 / 180.0;
    const double ry = 3.14159265358979 - pose.elevation * 3.14159265358979 / 180.0;
    const double Rz[3] = { -std::sin(rx) * std::cos(ry), std::cos(rx) * std::cos(ry), std::sin(ry) };
    const double d[3] = { -5.0, -12.0, 3.0 }; // camera - target
    const double length = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    for(int i = 0; i < 3; i++) EXPECT_NEAR(Rz[i], d[i] / length, 1e-5);
}

TEST(CameraView, ContradictorySettingsThrow) {
    EXPECT_THROW(CameraView::orbit(0.0 * deg, 0.0 * deg).look_at({ 1.0 * m, 1.0 * m, 1.0 * m }).pose(grid, cell), SetupError);
    EXPECT_THROW(CameraView::at({ 1.0 * m, 1.0 * m, 1.0 * m }).view_height(1.0 * m).pose(grid, cell), SetupError);
}

} // namespace
