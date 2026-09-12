#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/config/camera_config.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <vector>
#include <string>

extern Units units; // global units object from lbm.cpp

// Runs the simulation and renders every frame from each camera; named cameras write to export/<name>/.
// Frames are rendered for 60 fps playback of a video set_video_length() long.
class VideoRecorder {
public:
    VideoRecorder() = default;

    VideoRecorder& add(const CameraConfig& view) {
        views_.push_back(view);
        return *this;
    }

    VideoRecorder& add(const std::string& name, CameraConfig view) {
        view.set_name(name);
        views_.push_back(view);
        return *this;
    }

    /// The length of the video (default 10 s).
    VideoRecorder& set_video_length(Duration length) {
        video_length_s_ = length.si();
        return *this;
    }

    void record(LBM& lbm, uint64_t total_steps) {
        if (views_.empty()) {
            lbm.run(total_steps);
            return;
        }
        record_with_callback(lbm, total_steps, []() {}, 1u);
    }

    /// Runs this much simulated time (converted with the global units) and records it.
    void record(LBM& lbm, Duration time) {
        record(lbm, units.t(time.si()));
    }

    /// As record(LBM&, Duration); update_callback() runs every update_interval time steps (e.g. MovingPartsManager::update).
    template<typename Callback>
    void record(LBM& lbm, Duration time, Callback update_callback, uint32_t update_interval) {
        record_with_callback(lbm, units.t(time.si()), update_callback, update_interval);
    }

private:
    std::vector<CameraConfig> views_;
    float32_t video_length_s_ = 10.0f;

    template<typename Callback>
    void record_with_callback(LBM& lbm, uint64_t total_steps,
                              Callback update_callback, uint32_t update_interval) {
        lbm.run(0u, total_steps);
        while (lbm.get_t() <= total_steps) {
            update_callback();
            if (!views_.empty() && lbm.graphics.next_frame(total_steps, video_length_s_)) {
                for (const auto& view : views_) {
                    apply_view(lbm, view);
                    if (view.has_name()) {
                        lbm.graphics.write_frame(get_exe_path() + "export/" + view.name() + "/");
                    } else {
                        lbm.graphics.write_frame();
                    }
                }
            }
            lbm.run(update_interval, total_steps);
        }
    }

    static void apply_view(LBM& lbm, const CameraConfig& view) {
        if (view.is_free_mode()) {
            const float3 pos(
                view.pos_x() * (float)lbm.get_Nx(),
                view.pos_y() * (float)lbm.get_Ny(),
                view.pos_z() * (float)lbm.get_Nz()
            );
            lbm.graphics.set_camera_free(pos, view.pitch(), view.yaw(), view.fov());
        } else {
            lbm.graphics.set_camera_centered(view.pitch(), view.yaw(), view.fov(), view.zoom());
        }
    }
};
