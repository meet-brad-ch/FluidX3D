#pragma once

#include "core/types.hpp"
#include "config/camera_presets.hpp"
#include "lbm.hpp"

/**
 * @file graphics_config.hpp
 * @brief Fluent API for configuring FluidX3D visualization settings
 *
 * Provides an intuitive interface for setting up visualization modes,
 * slicing planes, and camera positioning.
 *
 * @note VIS_* constants are defined in defines.hpp and used directly
 */

/**
 * @enum FieldMode
 * @brief Field visualization modes
 */
enum class FieldMode {
    VELOCITY = 0,     ///< Velocity magnitude field
    DENSITY = 1,      ///< Density field
    THERMAL = 2       ///< Temperature field (requires TEMPERATURE extension)
};

/**
 * @enum SliceMode
 * @brief Slice plane configurations
 */
enum class SliceMode {
    NONE = 0,   ///< No slicing
    X = 1,      ///< YZ plane slice at X position
    Y = 2,      ///< XZ plane slice at Y position
    Z = 3,      ///< XY plane slice at Z position
    XZ = 4,     ///< Two slices: X and Z
    XYZ = 5,    ///< Three slices: X, Y, and Z
    YZ = 6,     ///< Two slices: Y and Z
    XY = 7      ///< Two slices: X and Y
};

/**
 * @class GraphicsConfig
 * @brief Fluent API for configuring LBM graphics settings
 *
 * Provides intuitive methods for setting visualization modes, slicing,
 * and camera positioning. All settings are applied when apply() is called.
 *
 * @note Visualization modes are cleared by default. Use inherit_modes()
 *       to start from current LBM settings instead.
 *
 * @par Example:
 * @code
 * GraphicsConfig(lbm)
 *     .show_surface()
 *     .show_vortices()
 *     .slice_y(0.5f)
 *     .set_camera_isometric()
 *     .apply();
 * @endcode
 */
class GraphicsConfig {
public:
    /**
     * @brief Construct GraphicsConfig for an LBM simulation
     * @param lbm Reference to the LBM simulation object
     *
     * @note Visualization modes start cleared (empty). Call show_*() methods
     *       to enable specific modes, or use inherit_modes() to start from
     *       current LBM settings.
     */
    explicit GraphicsConfig(LBM& lbm) : lbm_(lbm) {
        // Modes start cleared by default - simpler API, no need for clear_modes()
        // Use inherit_modes() to load current LBM settings instead
#ifdef GRAPHICS
        visualization_modes_ = 0;  // Cleared by default
        field_mode_ = 0;
        slice_mode_ = lbm_.graphics.slice_mode;
        slice_x_ = lbm_.graphics.slice_x;
        slice_y_ = lbm_.graphics.slice_y;
        slice_z_ = lbm_.graphics.slice_z;
#endif
    }

    /**
     * @brief Inherit visualization modes from current LBM settings
     * @return Reference for method chaining
     *
     * Use this to start from current LBM visualization modes instead of
     * an empty slate. Useful when you want to modify existing settings
     * rather than replace them completely.
     *
     * @par Example:
     * @code
     * // Add vortices to whatever is currently showing
     * GraphicsConfig(lbm)
     *     .inherit_modes()
     *     .show_vortices()
     *     .apply();
     * @endcode
     */
    GraphicsConfig& inherit_modes() {
#ifdef GRAPHICS
        visualization_modes_ = lbm_.graphics.visualization_modes;
        field_mode_ = lbm_.graphics.field_mode;
#endif
        return *this;
    }

    // ========================================================================
    // Visualization Mode Presets
    // ========================================================================

    /**
     * @brief Show solid surfaces using marching cubes
     * @return Reference for method chaining
     */
    GraphicsConfig& show_surface() {
        visualization_modes_ |= VIS_FLAG_SURFACE;
        return *this;
    }

    /**
     * @brief Show lattice flags (cell types)
     * @return Reference for method chaining
     */
    GraphicsConfig& show_flags() {
        visualization_modes_ |= VIS_FLAG_LATTICE;
        return *this;
    }

    /**
     * @brief Show lattice structure visualization
     * @return Reference for method chaining
     */
    GraphicsConfig& show_lattice() {
        visualization_modes_ |= VIS_FLAG_LATTICE;
        return *this;
    }

    /**
     * @brief Show Q-criterion vortex visualization
     * @return Reference for method chaining
     */
    GraphicsConfig& show_vortices() {
        visualization_modes_ |= VIS_Q_CRITERION;
        return *this;
    }

    /**
     * @brief Show velocity field visualization
     * @return Reference for method chaining
     */
    GraphicsConfig& show_velocity_field() {
        visualization_modes_ |= VIS_FIELD;
        field_mode_ = static_cast<int32_t>(FieldMode::VELOCITY);
        return *this;
    }

    /**
     * @brief Show density field visualization
     * @return Reference for method chaining
     */
    GraphicsConfig& show_density_field() {
        visualization_modes_ |= VIS_FIELD;
        field_mode_ = static_cast<int32_t>(FieldMode::DENSITY);
        return *this;
    }

    /**
     * @brief Show temperature field visualization
     * @return Reference for method chaining
     * @note Requires TEMPERATURE extension
     */
    GraphicsConfig& show_temperature_field() {
        visualization_modes_ |= VIS_FIELD;
        field_mode_ = static_cast<int32_t>(FieldMode::THERMAL);
        return *this;
    }

    /**
     * @brief Show streamlines
     * @return Reference for method chaining
     */
    GraphicsConfig& show_streamlines() {
        visualization_modes_ |= VIS_STREAMLINES;
        return *this;
    }

    /**
     * @brief Show free surface using rasterization
     * @return Reference for method chaining
     * @note Requires SURFACE extension
     */
    GraphicsConfig& show_free_surface() {
        visualization_modes_ |= VIS_PHI_RASTERIZE;
        return *this;
    }

    /**
     * @brief Show free surface using raytracing (slower but higher quality)
     * @return Reference for method chaining
     * @note Requires SURFACE extension, single-GPU only
     */
    GraphicsConfig& show_free_surface_raytraced() {
        visualization_modes_ |= VIS_PHI_RAYTRACE;
        return *this;
    }

    /**
     * @brief Show particles
     * @return Reference for method chaining
     * @note Requires PARTICLES extension
     */
    GraphicsConfig& show_particles() {
        visualization_modes_ |= VIS_PARTICLES;
        return *this;
    }

    /**
     * @brief Hide solid surfaces
     * @return Reference for method chaining
     */
    GraphicsConfig& hide_surface() {
        visualization_modes_ &= ~VIS_FLAG_SURFACE;
        return *this;
    }

    /**
     * @brief Hide lattice flags
     * @return Reference for method chaining
     */
    GraphicsConfig& hide_flags() {
        visualization_modes_ &= ~VIS_FLAG_LATTICE;
        return *this;
    }

    /**
     * @brief Hide vortex visualization
     * @return Reference for method chaining
     */
    GraphicsConfig& hide_vortices() {
        visualization_modes_ &= ~VIS_Q_CRITERION;
        return *this;
    }

    /**
     * @brief Hide field visualization
     * @return Reference for method chaining
     */
    GraphicsConfig& hide_field() {
        visualization_modes_ &= ~VIS_FIELD;
        return *this;
    }

    /**
     * @brief Clear all visualization modes
     * @return Reference for method chaining
     */
    GraphicsConfig& clear_modes() {
        visualization_modes_ = 0;
        return *this;
    }

    /**
     * @brief Set visualization modes directly
     * @param modes Bit flags for visualization modes
     * @return Reference for method chaining
     */
    GraphicsConfig& set_modes(int32_t modes) {
        visualization_modes_ = modes;
        return *this;
    }

    /**
     * @brief Set field mode for field visualization
     * @param mode Field mode (velocity, density, or temperature)
     * @return Reference for method chaining
     */
    GraphicsConfig& set_field_mode(FieldMode mode) {
        field_mode_ = static_cast<int32_t>(mode);
        return *this;
    }

    // ========================================================================
    // Slicing
    // ========================================================================

    /**
     * @brief Set X-plane slice position
     * @param position_fraction Position as fraction of domain (0.0-1.0)
     * @return Reference for method chaining
     */
    GraphicsConfig& slice_x(float32_t position_fraction = 0.5f) {
        slice_x_ = static_cast<int32_t>(position_fraction * lbm_.get_Nx());
        if (slice_mode_ == static_cast<int32_t>(SliceMode::NONE)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::X);
        } else if (slice_mode_ == static_cast<int32_t>(SliceMode::Y)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::XY);
        } else if (slice_mode_ == static_cast<int32_t>(SliceMode::Z)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::XZ);
        } else if (slice_mode_ == static_cast<int32_t>(SliceMode::YZ)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::XYZ);
        }
        return *this;
    }

    /**
     * @brief Set Y-plane slice position
     * @param position_fraction Position as fraction of domain (0.0-1.0)
     * @return Reference for method chaining
     */
    GraphicsConfig& slice_y(float32_t position_fraction = 0.5f) {
        slice_y_ = static_cast<int32_t>(position_fraction * lbm_.get_Ny());
        if (slice_mode_ == static_cast<int32_t>(SliceMode::NONE)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::Y);
        } else if (slice_mode_ == static_cast<int32_t>(SliceMode::X)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::XY);
        } else if (slice_mode_ == static_cast<int32_t>(SliceMode::Z)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::YZ);
        } else if (slice_mode_ == static_cast<int32_t>(SliceMode::XZ)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::XYZ);
        }
        return *this;
    }

    /**
     * @brief Set Z-plane slice position
     * @param position_fraction Position as fraction of domain (0.0-1.0)
     * @return Reference for method chaining
     */
    GraphicsConfig& slice_z(float32_t position_fraction = 0.5f) {
        slice_z_ = static_cast<int32_t>(position_fraction * lbm_.get_Nz());
        if (slice_mode_ == static_cast<int32_t>(SliceMode::NONE)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::Z);
        } else if (slice_mode_ == static_cast<int32_t>(SliceMode::X)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::XZ);
        } else if (slice_mode_ == static_cast<int32_t>(SliceMode::Y)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::YZ);
        } else if (slice_mode_ == static_cast<int32_t>(SliceMode::XY)) {
            slice_mode_ = static_cast<int32_t>(SliceMode::XYZ);
        }
        return *this;
    }

    /**
     * @brief Set slice mode directly
     * @param mode Slice mode enum value
     * @return Reference for method chaining
     */
    GraphicsConfig& set_slice_mode(SliceMode mode) {
        slice_mode_ = static_cast<int32_t>(mode);
        return *this;
    }

    /**
     * @brief Clear all slicing
     * @return Reference for method chaining
     */
    GraphicsConfig& clear_slices() {
        slice_mode_ = static_cast<int32_t>(SliceMode::NONE);
        return *this;
    }

    // ========================================================================
    // Camera Presets
    // ========================================================================

    /**
     * @brief Set camera to centered mode with rotation angles
     * @param pitch_deg Pitch angle in degrees (vertical rotation)
     * @param yaw_deg Yaw angle in degrees (horizontal rotation)
     * @param zoom Zoom level (default: 1.0)
     * @param fov Field of view in degrees (default: 100)
     * @return Reference for method chaining
     */
    GraphicsConfig& set_camera(float32_t pitch_deg, float32_t yaw_deg,
                               float32_t zoom = CameraPresets::DEFAULT_ZOOM,
                               float32_t fov = CameraPresets::DEFAULT_FOV) {
#ifdef GRAPHICS
        camera_rx_ = pitch_deg; // degrees, as set_camera_centered() expects
        camera_ry_ = yaw_deg;
        camera_zoom_ = zoom;
        camera_fov_ = fov;
        camera_mode_ = CameraMode::CENTERED;
#endif
        return *this;
    }

    /**
     * @brief Set camera to top-down view (looking down Z axis)
     * @param zoom Zoom level (default: 1.0)
     * @return Reference for method chaining
     */
    GraphicsConfig& set_camera_top_view(float32_t zoom = CameraPresets::DEFAULT_ZOOM) {
        return set_camera(CameraPresets::TOP_VIEW_PITCH, CameraPresets::TOP_VIEW_YAW, zoom);
    }

    /**
     * @brief Set camera to side view (looking along X axis)
     * @param zoom Zoom level (default: 1.0)
     * @return Reference for method chaining
     */
    GraphicsConfig& set_camera_side_view(float32_t zoom = CameraPresets::DEFAULT_ZOOM) {
        return set_camera(CameraPresets::SIDE_VIEW_PITCH, CameraPresets::SIDE_VIEW_YAW, zoom);
    }

    /**
     * @brief Set camera to front view (looking along Y axis)
     * @param zoom Zoom level (default: 1.0)
     * @return Reference for method chaining
     */
    GraphicsConfig& set_camera_front_view(float32_t zoom = CameraPresets::DEFAULT_ZOOM) {
        return set_camera(CameraPresets::FRONT_VIEW_PITCH, CameraPresets::FRONT_VIEW_YAW, zoom);
    }

    /**
     * @brief Set camera to isometric view (45 degree angles)
     * @param zoom Zoom level (default: 1.0)
     * @return Reference for method chaining
     */
    GraphicsConfig& set_camera_isometric(float32_t zoom = CameraPresets::DEFAULT_ZOOM) {
        return set_camera(CameraPresets::ISOMETRIC_PITCH, CameraPresets::ISOMETRIC_YAW, zoom);
    }

    /**
     * @brief Enable camera auto-rotation
     * @return Reference for method chaining
     */
    GraphicsConfig& enable_autorotation() {
#ifdef GRAPHICS
        autorotation_ = true;
#endif
        return *this;
    }

    /**
     * @brief Disable camera auto-rotation
     * @return Reference for method chaining
     */
    GraphicsConfig& disable_autorotation() {
#ifdef GRAPHICS
        autorotation_ = false;
#endif
        return *this;
    }

    // ========================================================================
    // Apply Settings
    // ========================================================================

    /**
     * @brief Apply all configured settings to the LBM graphics object
     */
    void apply() {
#ifdef GRAPHICS
        lbm_.graphics.visualization_modes = visualization_modes_;
        lbm_.graphics.field_mode = field_mode_;
        lbm_.graphics.slice_mode = slice_mode_;
        lbm_.graphics.slice_x = slice_x_;
        lbm_.graphics.slice_y = slice_y_;
        lbm_.graphics.slice_z = slice_z_;

        if (camera_mode_ == CameraMode::CENTERED) {
            lbm_.graphics.set_camera_centered(camera_rx_, camera_ry_, camera_fov_, camera_zoom_);
        }

        camera.autorotation = autorotation_;
#endif
    }

    // ========================================================================
    // Getters
    // ========================================================================

    /**
     * @brief Get current visualization modes
     * @return Bit flags for active visualization modes
     */
    int32_t get_visualization_modes() const { return visualization_modes_; }

    /**
     * @brief Get current field mode
     * @return Field mode value
     */
    int32_t get_field_mode() const { return field_mode_; }

    /**
     * @brief Get current slice mode
     * @return Slice mode value
     */
    int32_t get_slice_mode() const { return slice_mode_; }

private:
    LBM& lbm_;

    // Visualization settings
    int32_t visualization_modes_ = 0;
    int32_t field_mode_ = 0;
    int32_t slice_mode_ = 0;
    int32_t slice_x_ = 0;
    int32_t slice_y_ = 0;
    int32_t slice_z_ = 0;

    // Camera settings
    enum class CameraMode { NONE, CENTERED, FREE };
    CameraMode camera_mode_ = CameraMode::NONE;
    float32_t camera_rx_ = 0.0f;
    float32_t camera_ry_ = 0.0f;
    float32_t camera_zoom_ = CameraPresets::DEFAULT_ZOOM;
    float32_t camera_fov_ = CameraPresets::DEFAULT_FOV;
    bool autorotation_ = false;
};
