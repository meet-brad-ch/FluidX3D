#pragma once

#include "setup/core/types.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/core/unit_scale.hpp"
#include "setup/graphics/camera_view.hpp"
#include "lbm.hpp"
#include <optional>

/// The field the graphics color (lbm.graphics.field_mode).
enum class FieldMode {
    VELOCITY = 0, ///< the velocity
    DENSITY = 1,  ///< the density
    THERMAL = 2   ///< the temperature (TEMPERATURE extension)
};

/// The slice planes drawn (lbm.graphics.slice_mode).
enum class SliceMode {
    NONE = 0, ///< no slice
    X = 1,    ///< the YZ plane
    Y = 2,    ///< the XZ plane
    Z = 3,    ///< the XY plane
    XZ = 4,   ///< the YZ and XY planes
    XYZ = 5,  ///< all three planes
    YZ = 6,   ///< the XZ and XY planes
    XY = 7    ///< the YZ and XZ planes
};

/// @brief Visualization modes and the camera of the LBM graphics (Simulation::graphics()); apply() writes them and
/// turns the camera's autorotation off. Without the GRAPHICS extension it does nothing.
///
/// Starts with no modes; inherit_modes() starts from the LBM's current modes instead (e.g. the particles').
/// @code
/// sim.graphics().show_surface().show_vortices().set_camera(CameraView::orbit(-40_deg, 25_deg)).apply();
/// @endcode
class GraphicsConfig {
public:
    /// @param lbm        the LBM whose graphics are configured
    /// @param unit_scale the simulation's unit scale, for the camera
    GraphicsConfig(LBM& lbm, const UnitScale& unit_scale) : lbm_(lbm), units_(unit_scale) {
#ifdef GRAPHICS
        slice_mode_ = lbm_.graphics.slice_mode;
#endif
    }

    /// @brief Keeps the modes the LBM has now (those of ParticleManager::initialize(), for example).
    /// @return this configuration
    GraphicsConfig& inherit_modes() {
#ifdef GRAPHICS
        visualization_modes_ = lbm_.graphics.visualization_modes;
        field_mode_ = lbm_.graphics.field_mode;
#endif
        return *this;
    }

    /// @brief Solid surfaces (marching cubes).
    /// @return this configuration
    GraphicsConfig& show_surface() {
        visualization_modes_ |= VIS_FLAG_SURFACE;
        return *this;
    }

    /// @brief The cell flags as a wireframe.
    /// @return this configuration
    GraphicsConfig& show_flags() {
        visualization_modes_ |= VIS_FLAG_LATTICE;
        return *this;
    }

    /// @brief Vortices: the Q-criterion isosurface.
    /// @return this configuration
    GraphicsConfig& show_vortices() {
        visualization_modes_ |= VIS_Q_CRITERION;
        return *this;
    }

    /// @brief The velocity field as colored arrows.
    /// @return this configuration
    GraphicsConfig& show_velocity_field() {
        visualization_modes_ |= VIS_FIELD;
        field_mode_ = static_cast<int32_t>(FieldMode::VELOCITY);
        return *this;
    }

    /// @brief The density field as colored arrows.
    /// @return this configuration
    GraphicsConfig& show_density_field() {
        visualization_modes_ |= VIS_FIELD;
        field_mode_ = static_cast<int32_t>(FieldMode::DENSITY);
        return *this;
    }

    /// @brief Streamlines.
    /// @return this configuration
    GraphicsConfig& show_streamlines() {
        visualization_modes_ |= VIS_STREAMLINES;
        return *this;
    }

    /// @brief The free surface (SURFACE extension): raytraced on one GPU, rasterized on several.
    /// @return this configuration
    GraphicsConfig& show_free_surface() {
        visualization_modes_ |= lbm_.get_D() == 1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
        return *this;
    }

    /// @brief The free surface rasterized (marching cubes) also on one GPU, e.g. with show_flags().
    /// @return this configuration
    GraphicsConfig& show_free_surface_mesh() {
        visualization_modes_ |= VIS_PHI_RASTERIZE;
        return *this;
    }

    /// @brief Slice planes at the LBM's current slice positions.
    /// @param mode the planes
    /// @return this configuration
    GraphicsConfig& set_slice_mode(SliceMode mode) {
        slice_mode_ = static_cast<int32_t>(mode);
        return *this;
    }

    /// @brief The view the graphics start with (also for interactive graphics).
    /// @param view the view
    /// @return this configuration
    GraphicsConfig& set_camera(const CameraView& view) {
        camera_ = view;
        return *this;
    }

    /// Writes the modes and the camera to the LBM's graphics.
    void apply() {
#ifdef GRAPHICS
        lbm_.graphics.visualization_modes = visualization_modes_;
        lbm_.graphics.field_mode = field_mode_;
        lbm_.graphics.slice_mode = slice_mode_;
        camera.autorotation = false;
        if(camera_) apply_camera(lbm_, units_, *camera_);
#endif
    }

    /// @brief Points the LBM's camera (the core's global camera) as a view; exits with a message for contradictory
    /// camera settings.
    /// @param lbm        the LBM whose camera is pointed
    /// @param unit_scale the simulation's unit scale
    /// @param view       the view
    static void apply_camera(LBM& lbm, const UnitScale& unit_scale, const CameraView& view) {
#ifdef GRAPHICS
        std::optional<CameraPose> pose;
        try {
            pose.emplace(view.pose(uint3(lbm.get_Nx(), lbm.get_Ny(), lbm.get_Nz()), unit_scale.cell_size().si()));
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        if(pose->free) lbm.graphics.set_camera_free(pose->position, pose->azimuth, pose->elevation, pose->field_of_view);
        else lbm.graphics.set_camera_centered(pose->azimuth, pose->elevation, pose->field_of_view, pose->zoom);
#else
        (void)lbm;
        (void)unit_scale;
        (void)view;
#endif
    }

private:
    LBM& lbm_;                         ///< the LBM whose graphics are configured
    UnitScale units_;                  ///< the simulation's unit scale
    std::optional<CameraView> camera_; ///< set_camera()
    int32_t visualization_modes_ = 0;  ///< the core's VIS_* flags
    int32_t field_mode_ = 0;           ///< the FieldMode
    int32_t slice_mode_ = 0;           ///< the SliceMode
};
