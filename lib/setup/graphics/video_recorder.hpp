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
/// set_frame_interval() of simulated time. A named camera writes to a folder of its name under export/, an unnamed
/// one to export/. Written when built with GRAPHICS but not INTERACTIVE_GRAPHICS.
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

    /// @brief An unnamed fixed camera.
    /// @param view the view
    /// @return this recorder
    VideoRecorder& add(const CameraView& view) { return add("", view); }

    /// @brief An unnamed moving camera.
    /// @param path the view at each progress of the recording
    /// @return this recorder
    VideoRecorder& add(CameraPath path) { return add("", std::move(path)); }

    /// @brief A named fixed camera.
    /// @param name the camera's name, its folder under export/
    /// @param view the view
    /// @return this recorder
    VideoRecorder& add(const std::string& name, const CameraView& view) {
        return add(name, CameraPath([view](float) { return view; }));
    }

    /// @brief A named moving camera.
    /// @param name the camera's name, its folder under export/
    /// @param path the view at each progress of the recording
    /// @return this recorder
    VideoRecorder& add(const std::string& name, CameraPath path) {
        views_.push_back({ name, std::move(path) });
        return *this;
    }

    /// @brief The length of the video (default 10 s): 60 frames per second of it, spread over the simulated time.
    /// @param length the length
    /// @return this recorder
    VideoRecorder& set_length(Duration length) {
        video_length_s_ = length.si();
        return *this;
    }

    /// @brief A frame every interval of simulated time instead (from the start on).
    /// @param interval the interval
    /// @return this recorder
    VideoRecorder& set_frame_interval(Duration interval) {
        frame_interval_ = interval;
        return *this;
    }

    /// @return whether a camera was added
    bool has_views() const { return !views_.empty(); }

    /// @brief Points the camera at the first view and schedules the frames.
    /// @param runner         the simulation's runner, which writes the frames
    /// @param simulated_time the simulated time from now that makes the video
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
    /// A camera.
    struct View {
        std::string name; ///< its name, empty for an unnamed camera
        CameraPath path;  ///< its view at each progress of the recording
    };

    LBM& lbm_;                               ///< the LBM whose graphics render the frames
    UnitScale units_;                        ///< the simulation's unit scale
    std::vector<View> views_;                ///< the cameras
    float32_t video_length_s_ = 10.0f;       ///< set_length(), in seconds
    std::optional<Duration> frame_interval_; ///< set_frame_interval()

    /// @brief Writes a frame from each camera.
    /// @param progress the progress of the recording, from 0 to 1
    void write_frames(float progress) const {
        for(const View& view : views_) {
            GraphicsConfig::apply_camera(lbm_, units_, view.path(progress));
            if(view.name.empty()) lbm_.graphics.write_frame();
            else lbm_.graphics.write_frame(get_exe_path() + "export/" + view.name + "/");
        }
    }
};
