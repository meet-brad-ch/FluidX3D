#pragma once

#include "setup/core/types.hpp"
#include <string>

// One camera for VideoRecorder: centered (orbits the domain center) or free (position as ratio of the domain size).
// Angles and field of view in degrees.
class CameraConfig {
public:
    enum class Mode { CENTERED, FREE };

    CameraConfig() = default;

    CameraConfig& set_angles(float32_t pitch, float32_t yaw) {
        pitch_ = pitch;
        yaw_ = yaw;
        return *this;
    }

    CameraConfig& set_fov(float32_t fov) {
        fov_ = fov;
        return *this;
    }

    CameraConfig& set_zoom(float32_t zoom) {
        zoom_ = zoom;
        return *this;
    }

    // free camera at (x*Nx, y*Ny, z*Nz)
    CameraConfig& set_free_position(float32_t x, float32_t y, float32_t z) {
        mode_ = Mode::FREE;
        pos_x_ = x;
        pos_y_ = y;
        pos_z_ = z;
        return *this;
    }

    // frames go to export/<name>/
    CameraConfig& set_name(const std::string& name) {
        name_ = name;
        return *this;
    }

    float32_t pitch() const { return pitch_; }
    float32_t yaw() const { return yaw_; }
    float32_t fov() const { return fov_; }
    float32_t zoom() const { return zoom_; }
    float32_t pos_x() const { return pos_x_; }
    float32_t pos_y() const { return pos_y_; }
    float32_t pos_z() const { return pos_z_; }
    bool is_free_mode() const { return mode_ == Mode::FREE; }
    const std::string& name() const { return name_; }
    bool has_name() const { return !name_.empty(); }

private:
    Mode mode_ = Mode::CENTERED;
    float32_t pitch_ = 0.0f;
    float32_t yaw_ = 0.0f;
    float32_t fov_ = 100.0f; // FluidX3D default
    float32_t zoom_ = 1.0f;
    float32_t pos_x_ = 0.0f;
    float32_t pos_y_ = 0.0f;
    float32_t pos_z_ = 0.0f;
    std::string name_;
};
