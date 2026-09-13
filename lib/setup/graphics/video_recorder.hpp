#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/unit_scale.hpp"
#include "setup/graphics/camera_view.hpp"
#include "setup/graphics/graphics_config.hpp"
#include "setup/simulation/runner.hpp"
#include "lbm.hpp"
#include <functional>
#include <optional>
#include <string>
#include <vector>

#ifndef GRAPHICS
#error "setup/graphics/video_recorder.hpp needs GRAPHICS (or INTERACTIVE_GRAPHICS) in defines.hpp"
#endif // GRAPHICS

/// @brief The video of the simulation (Simulation::video()), rendered from each camera: 60 frames per second of a
/// video set_length() long, spread over the simulated time of Simulation::run_for(); or a frame every
/// set_frame_interval() of simulated time. Named cameras write to export/<name>/, an unnamed one to export/.
/// Written when built with GRAPHICS but not INTERACTIVE_GRAPHICS.
/// @code
/// sim.video()
///     .add("side", CameraView::orbit(0_deg, 0_deg).field_of_view(25_deg))
///     .add("pan", [](float progress) { return CameraView::orbit(-70_deg + progress * 100_deg, 2_deg); })
///     .set_length(10_s);
/// sim.run_for(2_s);
/// @endcode
class VideoRecorder {
public:
    /// A moving camera: its view at the progress of the recording, from 0 to 1.
    using CameraPath = std::function<CameraView(float progress)>;

    /// @param lbm        the LBM whose graphics render the frames
    /// @param unit_scale the simulation's unit scale, for the cameras
    VideoRecorder(LBM& lbm, const UnitScale& unit_scale) : lbm_(lbm), units_(unit_scale) {}

    VideoRecorder& add(const CameraView& view) { return add("", view); }
    VideoRecorder& add(CameraPath path) { return add("", std::move(path)); }
    VideoRecorder& add(const std::string& name, const CameraView& view) {
        return add(name, CameraPath([view](float) { return view; }));
    }
    VideoRecorder& add(const std::string& name, CameraPath path) {
        views_.push_back({ name, std::move(path) });
        return *this;
    }

    /// The length of the video (default 10 s): 60 frames per second of it, spread over the simulated time.
    VideoRecorder& set_length(Duration length) {
        video_length_s_ = length.si();
        return *this;
    }

    /// A frame every interval of simulated time instead (from the start on).
    VideoRecorder& set_frame_interval(Duration interval) {
        frame_interval_ = interval;
        return *this;
    }

    bool has_views() const { return !views_.empty(); }

    /// Points the camera at the first view and schedules the frames: this much simulated time from now makes the video.
    void start(Runner& runner, Duration simulated_time) {
        GraphicsConfig::apply_camera(lbm_, units_, views_.front().path(0.0f)); // the first view from the start
        const uint64_t start_step = lbm_.get_t();
        const uint64_t total_steps = start_step + units_.time_steps(simulated_time);
        if(frame_interval_) {
            runner.every(*frame_interval_, [this, start_step, total_steps](Duration) {
                write_frames((float)(lbm_.get_t() - start_step) / (float)(total_steps - start_step));
            });
        } else {
            runner.every_step([this, total_steps](Duration) {
                if(lbm_.graphics.next_frame(total_steps, video_length_s_)) write_frames((float)lbm_.get_t() / (float)total_steps);
            });
        }
    }

private:
    struct View {
        std::string name;
        CameraPath path;
    };

    LBM& lbm_;
    UnitScale units_;
    std::vector<View> views_;
    float32_t video_length_s_ = 10.0f;
    std::optional<Duration> frame_interval_;

    // a frame from each camera at this progress of the recording
    void write_frames(float progress) const {
        for(const View& view : views_) {
            GraphicsConfig::apply_camera(lbm_, units_, view.path(progress));
            if(view.name.empty()) lbm_.graphics.write_frame();
            else lbm_.graphics.write_frame(get_exe_path() + "export/" + view.name + "/");
        }
    }
};
