#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/graphics/camera_view.hpp"
#include "setup/graphics/graphics_config.hpp"
#include "setup/moving/moving_parts_manager.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <functional>
#include <string>
#include <vector>

extern Units units; // global units object from lbm.cpp

/// @brief Runs the simulation and renders it from each camera: 60 frames per second of a video set_video_length() long,
/// spread over the simulated time. Named cameras write to export/<name>/, an unnamed one to export/.
/// @code
/// VideoRecorder()
///     .add("side", CameraView::orbit(0_deg, 0_deg).field_of_view(25_deg))
///     .add("pan", [](float progress) { return CameraView::orbit(-70_deg + progress * 100_deg, 2_deg); })
///     .set_video_length(10_s)
///     .record(lbm, 2_s);
/// @endcode
class VideoRecorder {
public:
    /// A moving camera: its view at the progress of the recording, from 0 to 1.
    using CameraPath = std::function<CameraView(float progress)>;

    VideoRecorder& add(const CameraView& view) { return add("", view); }
    VideoRecorder& add(CameraPath path) { return add("", std::move(path)); }
    VideoRecorder& add(const std::string& name, const CameraView& view) {
        return add(name, CameraPath([view](float) { return view; }));
    }
    VideoRecorder& add(const std::string& name, CameraPath path) {
        views_.push_back({ name, std::move(path) });
        return *this;
    }

    /// The length of the video (default 10 s).
    VideoRecorder& set_video_length(Duration length) {
        video_length_s_ = length.si();
        return *this;
    }

    /// Runs this much simulated time (converted with the global units) and records it.
    void record(LBM& lbm, Duration time) {
        run_and_record(lbm, units.t(time.si()), nullptr);
    }

    /// As record(LBM&, Duration), turning the moving parts every MovingPartsManager::get_min_update_interval() steps.
    void record(LBM& lbm, Duration time, MovingPartsManager& parts) {
        run_and_record(lbm, units.t(time.si()), &parts);
    }

private:
    struct View {
        std::string name;
        CameraPath path;
    };

    std::vector<View> views_;
    float32_t video_length_s_ = 10.0f;

    void run_and_record(LBM& lbm, uint64_t total_steps, MovingPartsManager* parts) {
        if(views_.empty()) {
            if(parts) parts->run(total_steps);
            else lbm.run(total_steps);
            return;
        }
        const uint32_t interval = parts ? parts->get_min_update_interval() : 1u;
        GraphicsConfig::apply_camera(lbm, views_.front().path(0.0f)); // the first view from the start
        lbm.run(0u, total_steps);
        while(lbm.get_t() <= total_steps) {
            if(parts) parts->update();
            if(lbm.graphics.next_frame(total_steps, video_length_s_)) {
                const float progress = (float)lbm.get_t() / (float)total_steps;
                for(const View& view : views_) {
                    GraphicsConfig::apply_camera(lbm, view.path(progress));
                    if(view.name.empty()) lbm.graphics.write_frame();
                    else lbm.graphics.write_frame(get_exe_path() + "export/" + view.name + "/");
                }
            }
            lbm.run(interval, total_steps);
        }
    }
};
