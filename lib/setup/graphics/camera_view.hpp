#pragma once
#include "utilities.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include <optional>

/// @file camera_view.hpp
/// @brief A camera in physical units (CameraView) and the core's camera numbers it gives (CameraPose).

/// @brief The core's camera numbers, as LBM::Graphics::set_camera_free() and set_camera_centered() take them.
struct CameraPose {
    /// The core's field of view of a pose made without a CameraView, in degrees (as CameraView's default).
    static constexpr float default_field_of_view_deg{ 100.0f };

    bool free = false;            ///< true: set_camera_free() at position; false: set_camera_centered()
    float3 position{};            ///< free: the camera's position in cells from the domain's centre (the core's coordinates)
    float azimuth = 0.0f;         ///< the azimuth in degrees (the core's rx)
    float elevation = 0.0f;       ///< the elevation in degrees (the core's ry)
    float field_of_view = default_field_of_view_deg; ///< the horizontal field of view in degrees
    float zoom = 1.0f;            ///< centred: at 1 the frame's smaller side spans the domain's longest side
};

/// @brief A camera for pictures and videos in physical units.
///
/// Its angles give the direction from what it looks at to the camera: the azimuth about Z from +X toward +Y, and the
/// elevation above the horizontal. An orbit() camera circles the domain's centre and looks at it; an at() camera
/// stands at a position in metres from the domain's origin corner.
/// @code
/// using namespace si::symbols;
/// CameraView::orbit(-40.0 * deg, 20.0 * deg).field_of_view(78.0 * deg).view_height(3.0 * m)
/// CameraView::at({ 20.0 * m, 2.7 * m, 15.0 * m }, -33.0 * deg, 33.0 * deg).field_of_view(80.0 * deg)
/// CameraView::at({ 20.0 * m, 2.7 * m, 15.0 * m }).look_at({ 6.8 * m, 13.5 * m, 6.8 * m })
/// @endcode
class CameraView {
public:
    /// The field of view without field_of_view().
    static constexpr si::Angle default_field_of_view{ au::degrees(100.0) };

    /// @brief Makes a camera that circles the domain's centre and looks at it.
    /// @param azimuth the azimuth from which the centre is seen
    /// @param elevation the elevation from which the centre is seen
    /// @return the view
    static CameraView orbit(si::Angle azimuth, si::Angle elevation) {
        CameraView view{};
        view.azimuth_ = azimuth;
        view.elevation_ = elevation;
        return view;
    }

    /// @brief Makes a camera at a position, looking toward what it sees from an azimuth and an elevation.
    /// @param position the camera's position in metres from the domain's origin corner
    /// @param azimuth the azimuth of the direction from what it looks at to the camera
    /// @param elevation the elevation of that direction
    /// @return the view
    static CameraView at(si::Position position, si::Angle azimuth = si::Angle{ au::ZERO }, si::Angle elevation = si::Angle{ au::ZERO }) {
        CameraView view{ orbit(azimuth, elevation) };
        view.position_ = position;
        return view;
    }

    /// @brief Turns an at() camera toward a point: the point sets the azimuth and the elevation.
    /// @param target the point in metres from the domain's origin corner
    /// @return this view
    CameraView& look_at(si::Position target) {
        target_ = target;
        return *this;
    }

    /// @brief Sets the horizontal field of view (default 100 degrees).
    /// @param fov the field of view
    /// @return this view
    CameraView& field_of_view(si::Angle fov) {
        field_of_view_ = fov;
        return *this;
    }

    /// @brief Sets the length that the frame's smaller side spans at the domain's centre (orbit() only; default: the
    /// domain's longest side).
    /// @param height the length, more than 0
    /// @return this view
    CameraView& view_height(si::Length height) {
        view_height_ = height;
        return *this;
    }

    /// @brief Gives the core's camera numbers on a grid.
    /// @param cells the grid's cells along X, Y and Z
    /// @param cell the grid's cell size, more than 0
    /// @return the pose
    /// @throws SetupError for look_at() on an orbit() camera, or view_height() on an at() camera
    CameraPose pose(const uint3& cells, si::Length cell) const;

private:
    /// @brief Gives a free camera's numbers: its position in the core's coordinates, turned toward the target when
    /// look_at() is set.
    /// @param pose the pose with the angles
    /// @param position the at() camera's position in metres from the domain's origin corner
    /// @param cells the grid's cells along X, Y and Z
    /// @param cell the grid's cell size
    /// @return the pose of the free camera
    CameraPose free_pose(CameraPose pose, const si::Position& position, const uint3& cells, si::Length cell) const;

    std::optional<si::Position> position_{};                           ///< at(): the camera's position, or none for orbit()
    si::Angle azimuth_{ au::ZERO };                                     ///< the azimuth of the direction from what the camera looks at to the camera
    si::Angle elevation_{ au::ZERO };                                   ///< the elevation of that direction
    si::Angle field_of_view_{ default_field_of_view }; ///< the horizontal field of view
    std::optional<si::Position> target_{};                             ///< look_at(): the point the camera looks at, or none
    std::optional<si::Length> view_height_{};                          ///< view_height(), or none
};
