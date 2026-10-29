#pragma once

#include "setup/core/types.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/graphics/camera_view.hpp"
#include "setup/units/lattice_converter.hpp"
#include "lbm.hpp"
#include <optional>

/// @file graphics_config.hpp
/// @brief The visualization modes and the camera of the LBM graphics (GraphicsConfig).

/// The field the graphics colour (lbm.graphics.field_mode).
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

/// @brief Sets the visualization modes and the camera of the LBM graphics (Simulation::graphics()); apply() writes
/// them and turns the camera's autorotation off. Without the GRAPHICS extension it does nothing.
///
/// A temporary configuration that keeps a reference to the LBM, which must outlive it. It writes lbm.graphics and the
/// core's global camera. It starts with no modes; inherit_modes() starts from the LBM's current modes instead (for
/// example the particles').
/// @code
/// using namespace si::symbols;
/// sim.graphics().show_surface().show_vortices().set_camera(CameraView::orbit(-40.0 * deg, 25.0 * deg)).apply();
/// @endcode
class GraphicsConfig {
public:
    /// @brief Makes the configuration; it starts from the LBM's current slice mode.
    /// @param lbm the LBM whose graphics are configured
    /// @param converter the simulation's conversions, for the camera's cell size
    GraphicsConfig(LBM& lbm, const si::LatticeConverter& converter) : lbm_(lbm), converter_(converter) {
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

    /// @brief Shows the solid surfaces (marching cubes).
    /// @return this configuration
    GraphicsConfig& show_surface() {
        visualization_modes_ |= VIS_FLAG_SURFACE;
        return *this;
    }

    /// @brief Shows the cell flags as a wireframe.
    /// @return this configuration
    GraphicsConfig& show_flags() {
        visualization_modes_ |= VIS_FLAG_LATTICE;
        return *this;
    }

    /// @brief Shows the vortices: the Q-criterion isosurface.
    /// @return this configuration
    GraphicsConfig& show_vortices() {
        visualization_modes_ |= VIS_Q_CRITERION;
        return *this;
    }

    /// @brief Shows the velocity field as coloured arrows.
    /// @return this configuration
    GraphicsConfig& show_velocity_field() {
        visualization_modes_ |= VIS_FIELD;
        field_mode_ = static_cast<int32_t>(FieldMode::VELOCITY);
        return *this;
    }

    /// @brief Shows the density field as coloured arrows.
    /// @return this configuration
    GraphicsConfig& show_density_field() {
        visualization_modes_ |= VIS_FIELD;
        field_mode_ = static_cast<int32_t>(FieldMode::DENSITY);
        return *this;
    }

    /// @brief Shows streamlines.
    /// @return this configuration
    GraphicsConfig& show_streamlines() {
        visualization_modes_ |= VIS_STREAMLINES;
        return *this;
    }

    /// @brief Shows the free surface (SURFACE extension): raytraced on one GPU, rasterized on several.
    /// @return this configuration
    GraphicsConfig& show_free_surface() {
        visualization_modes_ |= lbm_.get_D() == 1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
        return *this;
    }

    /// @brief Shows the free surface rasterized (marching cubes), also on one GPU, for example with show_flags().
    /// @return this configuration
    GraphicsConfig& show_free_surface_mesh() {
        visualization_modes_ |= VIS_PHI_RASTERIZE;
        return *this;
    }

    /// @brief Shows slice planes at the LBM's current slice positions.
    /// @param mode the planes to show
    /// @return this configuration
    GraphicsConfig& set_slice_mode(SliceMode mode) {
        slice_mode_ = static_cast<int32_t>(mode);
        return *this;
    }

    /// @brief Sets the view that the graphics start with (also for interactive graphics).
    /// @param view the camera in physical units
    /// @return this configuration
    GraphicsConfig& set_camera(const CameraView& view) {
        camera_ = view;
        return *this;
    }

    /// @brief Writes the modes and the camera to the LBM's graphics and turns the autorotation off.
    ///
    /// Ends the program with print_error() for contradictory camera settings (apply_camera()).
    void apply() {
#ifdef GRAPHICS
        lbm_.graphics.visualization_modes = visualization_modes_;
        lbm_.graphics.field_mode = field_mode_;
        lbm_.graphics.slice_mode = slice_mode_;
        camera.autorotation = false;
        if(camera_.has_value()) {
            const CameraView& view{ camera_.value() };
            apply_camera(lbm_, converter_, view);
        }
#endif
    }

    /// @brief Points the LBM's camera (the core's global camera) as a view.
    ///
    /// Ends the program with print_error() for contradictory camera settings (CameraView::pose()).
    /// @param lbm the LBM whose camera is pointed
    /// @param converter the simulation's conversions, for the cell size
    /// @param view the camera in physical units
    static void apply_camera(LBM& lbm, const si::LatticeConverter& converter, const CameraView& view) {
#ifdef GRAPHICS
        std::optional<CameraPose> pose{};
        try {
            const uint3 cells(lbm.get_Nx(), lbm.get_Ny(), lbm.get_Nz());
            const si::Length cell{ converter.cell() };
            pose.emplace(view.pose(cells, cell));
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        if(pose->free) {
            lbm.graphics.set_camera_free(pose->position, pose->azimuth, pose->elevation, pose->field_of_view);
        } else {
            lbm.graphics.set_camera_centered(pose->azimuth, pose->elevation, pose->field_of_view, pose->zoom);
        }
#else
        (void)lbm;
        (void)converter;
        (void)view;
#endif
    }

private:
    LBM& lbm_;                           ///< the LBM whose graphics are configured (not owned)
    si::LatticeConverter converter_;     ///< the simulation's conversions
    std::optional<CameraView> camera_{}; ///< set_camera(), or none
    int32_t visualization_modes_ = 0;    ///< the core's VIS_* flags
    int32_t field_mode_ = 0;             ///< the FieldMode, as the core's number
    int32_t slice_mode_ = 0;             ///< the SliceMode, as the core's number
};
