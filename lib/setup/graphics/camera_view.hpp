#pragma once
#include "utilities.hpp"
#include "setup/core/quantity.hpp"
#include <optional>

/// The core's camera parameters, as LBM::Graphics::set_camera_free() and set_camera_centered() take them.
struct CameraPose {
    bool free = false;            ///< set_camera_free(); otherwise set_camera_centered()
    float3 position{};            ///< free: in cells from the domain's center (the core's coordinates)
    float azimuth = 0.0f;         ///< degrees (the core's rx)
    float elevation = 0.0f;       ///< degrees (the core's ry)
    float field_of_view = 100.0f; ///< degrees
    float zoom = 1.0f;            ///< centered: at 1 the frame's smaller side spans the domain's longest side
};

/// @brief A camera for pictures and videos in physical units.
///
/// Its angles give the direction from what it looks at to the camera: the azimuth about Z from +X toward +Y and the
/// elevation above the horizontal. An orbit() camera circles the domain's center and looks at it; an at() camera stands
/// at a position in metres from the domain's origin corner.
/// @code
/// CameraView::orbit(-40_deg, 20_deg).field_of_view(78_deg).view_height(3_m)
/// CameraView::at({ 20_m, 2.7_m, 15_m }, -33_deg, 33_deg).field_of_view(80_deg)
/// CameraView::at({ 20_m, 2.7_m, 15_m }).look_at({ 6.8_m, 13.5_m, 6.8_m })
/// @endcode
class CameraView {
public:
    /// @brief Circles the domain's center.
    /// @param azimuth   the azimuth the center is seen from
    /// @param elevation the elevation
    /// @return the view
    static CameraView orbit(Angle azimuth, Angle elevation) {
        CameraView view;
        view.azimuth_ = azimuth;
        view.elevation_ = elevation;
        return view;
    }

    /// @brief At a position, looking at what it sees from an azimuth and an elevation.
    /// @param position  the camera's position
    /// @param azimuth   the azimuth of the direction from what it looks at to the camera
    /// @param elevation the elevation of that direction
    /// @return the view
    static CameraView at(Position position, Angle azimuth = {}, Angle elevation = {}) {
        CameraView view = orbit(azimuth, elevation);
        view.position_ = position;
        return view;
    }

    /// @brief at(): turned toward a point.
    /// @param target the point
    /// @return this view
    CameraView& look_at(Position target) {
        target_ = target;
        return *this;
    }

    /// @brief The horizontal field of view (default 100 degrees).
    /// @param fov the field of view
    /// @return this view
    CameraView& field_of_view(Angle fov) {
        field_of_view_ = fov;
        return *this;
    }

    /// @brief orbit(): the frame's smaller side spans a length at the domain's center (default: the domain's longest
    /// side).
    /// @param height the length
    /// @return this view
    CameraView& view_height(Length height) {
        view_height_ = height;
        return *this;
    }

    /// @brief The core's parameters on a grid.
    /// @param cells     the grid's cells along X, Y and Z
    /// @param cell_size the cell size in m
    /// @return the pose
    /// @throws SetupError for look_at() on an orbit() camera, or view_height() on an at() camera
    CameraPose pose(const uint3& cells, float cell_size) const;

private:
    std::optional<Position> position_;               ///< at()
    Angle azimuth_{};                                ///< the azimuth of the direction from what the camera looks at to the camera
    Angle elevation_{};                              ///< the elevation of that direction
    Angle field_of_view_ = Angle::from_deg(100.0f); ///< field_of_view()
    std::optional<Position> target_;                 ///< look_at()
    std::optional<Length> view_height_;              ///< view_height()
};
