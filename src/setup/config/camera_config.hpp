#pragma once

#include "core/types.hpp"
#include "config/camera_presets.hpp"
#include <string>

/**
 * @file camera_config.hpp
 * @brief Camera configuration for FluidX3D simulations
 */

/**
 * @class CameraConfig
 * @brief Fluent API for configuring camera settings
 *
 * Supports two camera modes:
 * - Centered: CameraConfig orbits around domain center (default)
 * - Free: CameraConfig at specific position looking at target
 *
 * @par Example (centered mode):
 * @code
 * CameraConfig camera = CameraConfig()
 *     .set_angles(-40.0f, 20.0f)
 *     .set_zoom(1.25f);
 * @endcode
 *
 * @par Example (free mode):
 * @code
 * CameraConfig camera = CameraConfig()
 *     .set_free_position(2.0f, -0.8f, 1.0f)  // Position as ratio of domain size
 *     .set_angles(-38.0f, 37.0f)
 *     .set_fov(60.0f);
 * @endcode
 */
class CameraConfig {
public:
    enum class Mode { CENTERED, FREE };

    CameraConfig() = default;

    /**
     * @brief Set camera rotation angles
     * @param pitch Pitch angle in degrees (vertical rotation)
     * @param yaw Yaw angle in degrees (horizontal rotation)
     * @return Reference for method chaining
     */
    CameraConfig& set_angles(float32_t pitch, float32_t yaw) {
        pitch_ = pitch;
        yaw_ = yaw;
        return *this;
    }

    /**
     * @brief Set camera pitch angle
     * @param pitch Pitch angle in degrees
     * @return Reference for method chaining
     */
    CameraConfig& set_pitch(float32_t pitch) {
        pitch_ = pitch;
        return *this;
    }

    /**
     * @brief Set camera yaw angle
     * @param yaw Yaw angle in degrees
     * @return Reference for method chaining
     */
    CameraConfig& set_yaw(float32_t yaw) {
        yaw_ = yaw;
        return *this;
    }

    /**
     * @brief Set camera field of view
     * @param fov Field of view in degrees
     * @return Reference for method chaining
     */
    CameraConfig& set_fov(float32_t fov) {
        fov_ = fov;
        return *this;
    }

    /**
     * @brief Set camera zoom level
     * @param zoom Zoom level (1.0 = default)
     * @return Reference for method chaining
     */
    CameraConfig& set_zoom(float32_t zoom) {
        zoom_ = zoom;
        return *this;
    }

    /**
     * @brief Set free camera position as ratio of domain size
     * @param x X position as ratio of Nx (e.g., 2.0 = 2*Nx)
     * @param y Y position as ratio of Ny
     * @param z Z position as ratio of Nz
     * @return Reference for method chaining
     *
     * This switches camera to FREE mode. The actual position is calculated
     * as (x*Nx, y*Ny, z*Nz) when applied.
     */
    CameraConfig& set_free_position(float32_t x, float32_t y, float32_t z) {
        mode_ = Mode::FREE;
        pos_x_ = x;
        pos_y_ = y;
        pos_z_ = z;
        return *this;
    }

    /**
     * @brief Set camera name for multi-camera recording
     * @param name Name used for export subdirectory (e.g., "top" -> export/top/)
     * @return Reference for method chaining
     *
     * When recording with multiple cameras, each camera's frames are exported
     * to a subdirectory named after the camera.
     */
    CameraConfig& set_name(const std::string& name) {
        name_ = name;
        return *this;
    }

    // ========================================================================
    // Preset Views
    // ========================================================================

    /**
     * @brief Set camera to top-down view
     * @param zoom Zoom level (default: 1.0)
     * @return Reference for method chaining
     */
    CameraConfig& top_view(float32_t zoom = CameraPresets::DEFAULT_ZOOM) {
        pitch_ = CameraPresets::TOP_VIEW_PITCH;
        yaw_ = CameraPresets::TOP_VIEW_YAW;
        zoom_ = zoom;
        return *this;
    }

    /**
     * @brief Set camera to side view
     * @param zoom Zoom level (default: 1.0)
     * @return Reference for method chaining
     */
    CameraConfig& side_view(float32_t zoom = CameraPresets::DEFAULT_ZOOM) {
        pitch_ = CameraPresets::SIDE_VIEW_PITCH;
        yaw_ = CameraPresets::SIDE_VIEW_YAW;
        zoom_ = zoom;
        return *this;
    }

    /**
     * @brief Set camera to front view
     * @param zoom Zoom level (default: 1.0)
     * @return Reference for method chaining
     */
    CameraConfig& front_view(float32_t zoom = CameraPresets::DEFAULT_ZOOM) {
        pitch_ = CameraPresets::FRONT_VIEW_PITCH;
        yaw_ = CameraPresets::FRONT_VIEW_YAW;
        zoom_ = zoom;
        return *this;
    }

    /**
     * @brief Set camera to isometric view
     * @param zoom Zoom level (default: 1.0)
     * @return Reference for method chaining
     */
    CameraConfig& isometric(float32_t zoom = CameraPresets::DEFAULT_ZOOM) {
        pitch_ = CameraPresets::ISOMETRIC_PITCH;
        yaw_ = CameraPresets::ISOMETRIC_YAW;
        zoom_ = zoom;
        return *this;
    }

    // ========================================================================
    // Getters
    // ========================================================================

    float32_t pitch() const { return pitch_; }
    float32_t yaw() const { return yaw_; }
    float32_t fov() const { return fov_; }
    float32_t zoom() const { return zoom_; }
    Mode mode() const { return mode_; }
    float32_t pos_x() const { return pos_x_; }
    float32_t pos_y() const { return pos_y_; }
    float32_t pos_z() const { return pos_z_; }
    bool is_free_mode() const { return mode_ == Mode::FREE; }
    const std::string& name() const { return name_; }
    bool has_name() const { return !name_.empty(); }

private:
    Mode mode_ = Mode::CENTERED;  ///< CameraConfig mode
    float32_t pitch_ = 0.0f;      ///< Pitch angle in degrees
    float32_t yaw_ = 0.0f;        ///< Yaw angle in degrees
    float32_t fov_ = CameraPresets::DEFAULT_FOV;  ///< Field of view in degrees (default matches FluidX3D)
    float32_t zoom_ = 1.0f;       ///< Zoom level
    float32_t pos_x_ = 0.0f;      ///< Free camera X position ratio
    float32_t pos_y_ = 0.0f;      ///< Free camera Y position ratio
    float32_t pos_z_ = 0.0f;      ///< Free camera Z position ratio
    std::string name_;            ///< CameraConfig name for export subdirectory
};
