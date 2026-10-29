#include "setup/graphics/camera_view.hpp"
#include "setup/core/setup_error.hpp"

#include <algorithm>
#include <cmath>

CameraPose CameraView::pose(const uint3& cells, si::Length cell) const {
    CameraPose pose{};
    pose.azimuth = static_cast<float>(azimuth_.in(au::degrees));
    pose.elevation = static_cast<float>(elevation_.in(au::degrees));
    pose.field_of_view = static_cast<float>(field_of_view_.in(au::degrees));
    if(position_.has_value()) {
        if(view_height_.has_value()) {
            throw SetupError("CameraView::view_height() is for CameraView::orbit(); move an at() camera or narrow its field_of_view()");
        }
        const si::Position& position{ position_.value() };
        const CameraPose free{ free_pose(pose, position, cells, cell) };
        return free;
    }
    if(target_.has_value()) {
        throw SetupError("CameraView::look_at() is for CameraView::at(); an orbit() camera looks at the domain's center");
    }
    if(view_height_.has_value()) {
        // the core's zoom shows 0.5 max(N) / zoom cells on each side of the centre across the frame's smaller side
        const std::uint32_t longest_cells{ std::max(std::max(cells.x, cells.y), cells.z) };
        const si::Length longest{ cell * static_cast<double>(longest_cells) };
        const si::Length view_height{ view_height_.value() };
        const double zoom{ au::as_raw_number(longest / view_height) };
        pose.zoom = static_cast<float>(zoom);
    }
    return pose;
}

CameraPose CameraView::free_pose(CameraPose pose, const si::Position& position, const uint3& cells, si::Length cell) const {
    // the core's free camera is in cells from the domain's centre, which is half the grid from the origin corner
    const double half_x{ 0.5 * static_cast<double>(cells.x) };
    const double half_y{ 0.5 * static_cast<double>(cells.y) };
    const double half_z{ 0.5 * static_cast<double>(cells.z) };
    const double x{ au::as_raw_number(position.x / cell) - half_x };
    const double y{ au::as_raw_number(position.y / cell) - half_y };
    const double z{ au::as_raw_number(position.z / cell) - half_z };
    pose.free = true;
    pose.position = float3(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    if(!target_.has_value()) {
        return pose;
    }
    const si::Position& target{ target_.value() };
    const si::Position d{ position - target }; // from the target to the camera
    const double dx{ d.x.in(au::meters) };
    const double dy{ d.y.in(au::meters) };
    const double dz{ d.z.in(au::meters) };
    const double horizontal{ std::sqrt(dx * dx + dy * dy) };
    const si::Angle azimuth{ au::radians(std::atan2(dy, dx)) };
    const si::Angle elevation{ au::radians(std::atan2(dz, horizontal)) };
    pose.azimuth = static_cast<float>(azimuth.in(au::degrees));
    pose.elevation = static_cast<float>(elevation.in(au::degrees));
    return pose;
}
