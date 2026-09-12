#pragma once

#include "setup/core/types.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/graphics/camera_view.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <optional>

extern Units units; // global units object from lbm.cpp

enum class FieldMode { // lbm.graphics.field_mode
    VELOCITY = 0,
    DENSITY = 1,
    THERMAL = 2 // TEMPERATURE extension
};

enum class SliceMode { // lbm.graphics.slice_mode
    NONE = 0,
    X = 1,   // YZ plane
    Y = 2,   // XZ plane
    Z = 3,   // XY plane
    XZ = 4,
    XYZ = 5,
    YZ = 6,
    XY = 7
};

// Visualization modes and the camera of the LBM graphics; apply() writes them and turns camera autorotation off.
// Starts with no modes; inherit_modes() starts from the LBM's current modes instead.
class GraphicsConfig {
public:
    explicit GraphicsConfig(LBM& lbm) : lbm_(lbm) {
#ifdef GRAPHICS
        slice_mode_ = lbm_.graphics.slice_mode;
#endif
    }

    GraphicsConfig& inherit_modes() {
#ifdef GRAPHICS
        visualization_modes_ = lbm_.graphics.visualization_modes;
        field_mode_ = lbm_.graphics.field_mode;
#endif
        return *this;
    }

    GraphicsConfig& show_surface() { // solid surfaces (marching cubes)
        visualization_modes_ |= VIS_FLAG_SURFACE;
        return *this;
    }

    GraphicsConfig& show_flags() { // cell flags
        visualization_modes_ |= VIS_FLAG_LATTICE;
        return *this;
    }

    GraphicsConfig& show_lattice() { // same as show_flags()
        visualization_modes_ |= VIS_FLAG_LATTICE;
        return *this;
    }

    GraphicsConfig& show_vortices() { // Q-criterion
        visualization_modes_ |= VIS_Q_CRITERION;
        return *this;
    }

    GraphicsConfig& show_velocity_field() {
        visualization_modes_ |= VIS_FIELD;
        field_mode_ = static_cast<int32_t>(FieldMode::VELOCITY);
        return *this;
    }

    GraphicsConfig& show_density_field() {
        visualization_modes_ |= VIS_FIELD;
        field_mode_ = static_cast<int32_t>(FieldMode::DENSITY);
        return *this;
    }

    GraphicsConfig& show_streamlines() {
        visualization_modes_ |= VIS_STREAMLINES;
        return *this;
    }

    GraphicsConfig& show_free_surface() { // raytraced on one GPU, rasterized on several (SURFACE extension)
        visualization_modes_ |= lbm_.get_D() == 1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
        return *this;
    }

    // slice planes at the LBM's current slice positions
    GraphicsConfig& set_slice_mode(SliceMode mode) {
        slice_mode_ = static_cast<int32_t>(mode);
        return *this;
    }

    /// The view the graphics start with (also for interactive graphics); units from configure_units().
    GraphicsConfig& set_camera(const CameraView& view) {
        camera_ = view;
        return *this;
    }

    void apply() {
#ifdef GRAPHICS
        lbm_.graphics.visualization_modes = visualization_modes_;
        lbm_.graphics.field_mode = field_mode_;
        lbm_.graphics.slice_mode = slice_mode_;
        camera.autorotation = false;
        if(camera_) apply_camera(lbm_, *camera_);
#endif
    }

    /// Points the LBM's camera (the core's global camera) as this view, with the global units; exits with a message
    /// for contradictory camera settings.
    static void apply_camera(LBM& lbm, const CameraView& view) {
#ifdef GRAPHICS
        std::optional<CameraPose> pose;
        try {
            pose.emplace(view.pose(uint3(lbm.get_Nx(), lbm.get_Ny(), lbm.get_Nz()), units.si_x(1.0f)));
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        if(pose->free) lbm.graphics.set_camera_free(pose->position, pose->azimuth, pose->elevation, pose->field_of_view);
        else lbm.graphics.set_camera_centered(pose->azimuth, pose->elevation, pose->field_of_view, pose->zoom);
#else
        (void)lbm;
        (void)view;
#endif
    }

private:
    LBM& lbm_;
    std::optional<CameraView> camera_;
    int32_t visualization_modes_ = 0;
    int32_t field_mode_ = 0;
    int32_t slice_mode_ = 0;
};
