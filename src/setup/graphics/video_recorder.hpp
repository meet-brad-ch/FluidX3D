#pragma once

#include "core/types.hpp"
#include "config/camera_config.hpp"
#include "lbm.hpp"
#include <vector>
#include <string>

/**
 * @file video_recorder.hpp
 * @brief Video recording with multi-camera support for FluidX3D simulations
 */

/**
 * @class VideoRecorder
 * @brief Fluent API for recording simulation videos from multiple camera angles
 *
 * Records frames from one or more camera angles simultaneously. Each camera's
 * frames are exported to a subdirectory named after the camera.
 *
 * @par Example (single camera):
 * @code
 * VideoRecorder()
 *     .add(CameraConfig().set_angles(-40.0f, 20.0f).set_fov(78.0f))
 *     .set_fps(10.0f)
 *     .record(lbm, 10000ull);
 * @endcode
 *
 * @par Example (multiple cameras):
 * @code
 * VideoRecorder()
 *     .add("top", CameraConfig()
 *         .set_free_position(2.1f, -0.8f, 1.0f)
 *         .set_angles(-38.0f, 37.0f).set_fov(60.0f))
 *     .add("side", CameraConfig()
 *         .set_free_position(1.7f, 0.4f, 0.1f)
 *         .set_angles(24.0f, 2.0f).set_fov(92.0f))
 *     .set_fps(20.0f)
 *     .record(lbm, 108000ull);
 * @endcode
 */
class VideoRecorder {
public:
    VideoRecorder() = default;

    /**
     * @brief Add a camera configuration
     * @param camera CameraConfig configuration (should have name set via set_name())
     * @return Reference for method chaining
     */
    VideoRecorder& add(const CameraConfig& camera) {
        cameras_.push_back(camera);
        return *this;
    }

    /**
     * @brief Add a named camera configuration
     * @param name Name for export subdirectory (e.g., "top" -> export/top/)
     * @param camera CameraConfig configuration
     * @return Reference for method chaining
     */
    VideoRecorder& add(const std::string& name, CameraConfig camera) {
        camera.set_name(name);
        cameras_.push_back(camera);
        return *this;
    }

    /**
     * @brief Add standard orthogonal views (top, front, side, isometric)
     * @return Reference for method chaining
     *
     * Adds four standard camera views:
     * - "top": Looking down from above (pitch=90, yaw=0)
     * - "front": Looking from front (pitch=0, yaw=0)
     * - "side": Looking from side (pitch=0, yaw=90)
     * - "isometric": Standard isometric view (pitch=35, yaw=45)
     *
     * @par Example:
     * @code
     * VideoRecorder()
     *     .add_standard_views()
     *     .set_fps(10.0f)
     *     .record(lbm, 10000ull);
     * @endcode
     */
    VideoRecorder& add_standard_views() {
        add("top", CameraConfig().set_angles(0.0f, 90.0f).set_zoom(1.0f));
        add("front", CameraConfig().set_angles(0.0f, 0.0f).set_zoom(1.0f));
        add("side", CameraConfig().set_angles(90.0f, 0.0f).set_zoom(1.0f));
        add("isometric", CameraConfig().set_angles(45.0f, 35.0f).set_zoom(1.0f));
        return *this;
    }

    /**
     * @brief Add an orbiting camera view
     * @param name Name for export subdirectory
     * @param pitch Vertical angle in degrees (0=horizontal, 90=top-down)
     * @param zoom Zoom factor (1.0 = default)
     * @param start_yaw Starting horizontal angle in degrees
     * @param end_yaw Ending horizontal angle in degrees
     * @return Reference for method chaining
     *
     * Creates an orbiting camera that rotates around the object during recording.
     * The camera moves from start_yaw to end_yaw over the simulation duration.
     *
     * @note Only one orbit camera can be active at a time
     *
     * @par Example:
     * @code
     * VideoRecorder()
     *     .add_orbit("orbit", 30.0f, 1.0f, 0.0f, 360.0f)  // Full 360° orbit
     *     .set_fps(30.0f)
     *     .record(lbm, 10000ull);
     * @endcode
     */
    VideoRecorder& add_orbit(const std::string& name, float32_t pitch = 30.0f,
                              float32_t zoom = 1.0f,
                              float32_t start_yaw = 0.0f, float32_t end_yaw = 360.0f) {
        orbit_enabled_ = true;
        orbit_name_ = name;
        orbit_pitch_ = pitch;
        orbit_zoom_ = zoom;
        orbit_start_yaw_ = start_yaw;
        orbit_end_yaw_ = end_yaw;
        return *this;
    }

    /**
     * @brief Set frames per second for recording
     * @param fps Frames per second
     * @return Reference for method chaining
     */
    VideoRecorder& set_fps(float32_t fps) {
        fps_ = fps;
        return *this;
    }

    /**
     * @brief Set base output directory
     * @param dir Output directory path (default: "export/")
     * @return Reference for method chaining
     */
    VideoRecorder& set_output_dir(const std::string& dir) {
        output_dir_ = dir;
        if (!output_dir_.empty() && output_dir_.back() != '/') {
            output_dir_ += '/';
        }
        return *this;
    }

    /**
     * @brief Record simulation for specified number of timesteps
     * @param lbm LBM simulation object
     * @param total_steps Total simulation timesteps
     */
    void record(LBM& lbm, uint64_t total_steps) {
        if (cameras_.empty() && !orbit_enabled_) {
            // No cameras configured - just run simulation
            lbm.run(total_steps);
            return;
        }

        const uint32_t Nx = lbm.get_Nx();
        const uint32_t Ny = lbm.get_Ny();
        const uint32_t Nz = lbm.get_Nz();

        lbm.run(0u, total_steps);
        while (lbm.get_t() <= total_steps) {
            if (lbm.graphics.next_frame(total_steps, fps_)) {
                // Render static cameras
                for (const auto& camera : cameras_) {
                    apply_camera(lbm, camera, Nx, Ny, Nz);

                    if (camera.has_name()) {
                        lbm.graphics.write_frame(get_exe_path() + output_dir_ + camera.name() + "/");
                    } else {
                        lbm.graphics.write_frame();
                    }
                }

                // Render orbit camera if enabled
                if (orbit_enabled_) {
                    float32_t progress = (float32_t)lbm.get_t() / (float32_t)total_steps;
                    float32_t current_yaw = orbit_start_yaw_ + progress * (orbit_end_yaw_ - orbit_start_yaw_);
                    lbm.graphics.set_camera_centered(orbit_pitch_, current_yaw, 100.0f, orbit_zoom_);
                    lbm.graphics.write_frame(get_exe_path() + output_dir_ + orbit_name_ + "/");
                }
            }
            lbm.run(1u, total_steps);
        }
    }

    /**
     * @brief Record simulation for specified time in seconds
     * @param lbm LBM simulation object
     * @param seconds Simulation time in seconds
     * @param units Units object for time conversion
     */
    void record(LBM& lbm, float32_t seconds, const Units& units) {
        record(lbm, units.t(seconds));
    }

    /**
     * @brief Record simulation with moving parts callback
     * @param lbm LBM simulation object
     * @param seconds Simulation time in seconds
     * @param units Units object for time conversion
     * @param update_callback Callback to update moving parts (called every update_interval steps)
     * @param update_interval Timesteps between callback invocations
     *
     * @par Example with MovingPartsManager:
     * @code
     * MovingPartsManager parts(sim, lbm);
     * parts.add(MovingPart("rotor.stl").set_rotation_axis(RotationAxis::Z).set_rpm(348.0f));
     * parts.initialize();
     *
     * VideoRecorder()
     *     .add("front", CameraConfig().set_angles(-40.0f, 20.0f))
     *     .set_fps(10.0f)
     *     .record(lbm, 1.0f, units, [&]() { parts.update(); }, 4);
     * @endcode
     */
    template<typename Callback>
    void record(LBM& lbm, float32_t seconds, const Units& units,
                Callback update_callback, uint32_t update_interval) {
        record_with_callback(lbm, units.t(seconds), update_callback, update_interval);
    }

    /**
     * @brief Record simulation with callback (timesteps version)
     * @param lbm LBM simulation object
     * @param total_steps Total simulation timesteps
     * @param update_callback Callback for each update interval
     * @param update_interval Timesteps between callback invocations
     */
    template<typename Callback>
    void record(LBM& lbm, uint64_t total_steps,
                Callback update_callback, uint32_t update_interval) {
        record_with_callback(lbm, total_steps, update_callback, update_interval);
    }

    /**
     * @brief Record simulation with moving parts callback (timesteps version)
     * @param lbm LBM simulation object
     * @param total_steps Total simulation timesteps
     * @param update_callback Callback to update moving parts
     * @param update_interval Timesteps between callback invocations
     */
    template<typename Callback>
    void record_with_callback(LBM& lbm, uint64_t total_steps,
                              Callback update_callback, uint32_t update_interval) {
        if (cameras_.empty() && !orbit_enabled_) {
            // No cameras configured - just run simulation with callback
            lbm.run(0u, total_steps);
            while (lbm.get_t() <= total_steps) {
                update_callback();
                lbm.run(update_interval, total_steps);
            }
            return;
        }

        const uint32_t Nx = lbm.get_Nx();
        const uint32_t Ny = lbm.get_Ny();
        const uint32_t Nz = lbm.get_Nz();

        lbm.run(0u, total_steps);
        while (lbm.get_t() <= total_steps) {
            update_callback();

            if (lbm.graphics.next_frame(total_steps, fps_)) {
                // Render static cameras
                for (const auto& camera : cameras_) {
                    apply_camera(lbm, camera, Nx, Ny, Nz);

                    if (camera.has_name()) {
                        lbm.graphics.write_frame(get_exe_path() + output_dir_ + camera.name() + "/");
                    } else {
                        lbm.graphics.write_frame();
                    }
                }

                // Render orbit camera if enabled
                if (orbit_enabled_) {
                    float32_t progress = (float32_t)lbm.get_t() / (float32_t)total_steps;
                    float32_t current_yaw = orbit_start_yaw_ + progress * (orbit_end_yaw_ - orbit_start_yaw_);
                    lbm.graphics.set_camera_centered(orbit_pitch_, current_yaw, 100.0f, orbit_zoom_);
                    lbm.graphics.write_frame(get_exe_path() + output_dir_ + orbit_name_ + "/");
                }
            }
            lbm.run(update_interval, total_steps);
        }
    }

    /**
     * @brief Get the number of configured cameras
     * @return Number of cameras
     */
    size_t camera_count() const { return cameras_.size(); }

    /**
     * @brief Get current FPS setting
     * @return Frames per second
     */
    float32_t fps() const { return fps_; }

private:
    std::vector<CameraConfig> cameras_;
    float32_t fps_ = 10.0f;
    std::string output_dir_ = "export/";

    // Orbit camera configuration
    bool orbit_enabled_ = false;
    std::string orbit_name_ = "orbit";
    float32_t orbit_pitch_ = 30.0f;
    float32_t orbit_zoom_ = 1.0f;
    float32_t orbit_start_yaw_ = 0.0f;
    float32_t orbit_end_yaw_ = 360.0f;

    /**
     * @brief Apply camera configuration to LBM graphics
     */
    void apply_camera(LBM& lbm, const CameraConfig& camera,
                      uint32_t Nx, uint32_t Ny, uint32_t Nz) {
        if (camera.is_free_mode()) {
            const float3 pos(
                camera.pos_x() * (float)Nx,
                camera.pos_y() * (float)Ny,
                camera.pos_z() * (float)Nz
            );
            lbm.graphics.set_camera_free(pos, camera.pitch(), camera.yaw(), camera.fov());
        } else {
            lbm.graphics.set_camera_centered(camera.pitch(), camera.yaw(), camera.fov(), camera.zoom());
        }
    }
};
