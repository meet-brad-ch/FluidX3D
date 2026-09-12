#pragma once

#include "core/types.hpp"
#include "config/camera_config.hpp"
#include "lbm.hpp"
#include <vector>
#include <string>

// Runs the simulation and renders every frame from each camera; named cameras write to export/<name>/.
// Frames are rendered for 60 fps playback of set_video_length_s() seconds.
class VideoRecorder {
public:
    VideoRecorder() = default;

    VideoRecorder& add(const CameraConfig& camera) {
        cameras_.push_back(camera);
        return *this;
    }

    VideoRecorder& add(const std::string& name, CameraConfig camera) {
        camera.set_name(name);
        cameras_.push_back(camera);
        return *this;
    }

    VideoRecorder& set_video_length_s(float32_t seconds) {
        video_length_s_ = seconds;
        return *this;
    }

    void record(LBM& lbm, uint64_t total_steps) {
        if (cameras_.empty()) {
            lbm.run(total_steps);
            return;
        }
        record_with_callback(lbm, total_steps, []() {}, 1u);
    }

    void record(LBM& lbm, float32_t seconds, const Units& units) {
        record(lbm, units.t(seconds));
    }

    // update_callback() runs every update_interval time steps (e.g. MovingPartsManager::update)
    template<typename Callback>
    void record(LBM& lbm, float32_t seconds, const Units& units,
                Callback update_callback, uint32_t update_interval) {
        record_with_callback(lbm, units.t(seconds), update_callback, update_interval);
    }

private:
    std::vector<CameraConfig> cameras_;
    float32_t video_length_s_ = 10.0f;

    template<typename Callback>
    void record_with_callback(LBM& lbm, uint64_t total_steps,
                              Callback update_callback, uint32_t update_interval) {
        lbm.run(0u, total_steps);
        while (lbm.get_t() <= total_steps) {
            update_callback();
            if (!cameras_.empty() && lbm.graphics.next_frame(total_steps, video_length_s_)) {
                for (const auto& camera : cameras_) {
                    apply_camera(lbm, camera);
                    if (camera.has_name()) {
                        lbm.graphics.write_frame(get_exe_path() + "export/" + camera.name() + "/");
                    } else {
                        lbm.graphics.write_frame();
                    }
                }
            }
            lbm.run(update_interval, total_steps);
        }
    }

    static void apply_camera(LBM& lbm, const CameraConfig& camera) {
        if (camera.is_free_mode()) {
            const float3 pos(
                camera.pos_x() * (float)lbm.get_Nx(),
                camera.pos_y() * (float)lbm.get_Ny(),
                camera.pos_z() * (float)lbm.get_Nz()
            );
            lbm.graphics.set_camera_free(pos, camera.pitch(), camera.yaw(), camera.fov());
        } else {
            lbm.graphics.set_camera_centered(camera.pitch(), camera.yaw(), camera.fov(), camera.zoom());
        }
    }
};
