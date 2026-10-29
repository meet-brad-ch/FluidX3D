#pragma once

#include "setup/core/types.hpp"
#include "setup/graphics/camera_view.hpp"
#include "setup/graphics/graphics_config.hpp"
#include "setup/simulation/runner.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"
#include "lbm.hpp"
#include <functional>
#include <optional>
#include <string>
#include <vector>

#ifndef GRAPHICS
#error "setup/graphics/video_recorder.hpp needs GRAPHICS (or INTERACTIVE_GRAPHICS) in defines.hpp"
#endif // GRAPHICS

/// @file video_recorder.hpp
/// @brief The video of a simulation from one or more cameras (VideoRecorder).

/// @brief Records the video of the simulation (Simulation::video()), rendered from each camera: 60 frames per second
/// of a video set_length() long, spread over the simulated time of Simulation::run_for(); or a frame every
/// set_frame_interval() of simulated time.
///
/// Owned by Simulation, which makes it with the LBM; it keeps a reference to the LBM. A named camera writes to a
/// folder of its name under export/, an unnamed one to export/. The frames are written when built with GRAPHICS but
/// not INTERACTIVE_GRAPHICS.
/// @code
/// using namespace si::symbols;
/// sim.video()
///     .add("side", CameraView::orbit(0.0 * deg, 0.0 * deg).field_of_view(25.0 * deg))
///     .add("pan", [](float progress) { return CameraView::orbit(-70.0 * deg + progress * 100.0 * deg, 2.0 * deg); })
///     .set_length(10.0 * s);
/// sim.run_for(2.0 * s);
/// @endcode
class VideoRecorder {
public:
    /// A moving camera: its view at the progress of the recording, from 0 to 1.
    using CameraPath = std::function<CameraView(float progress)>;

    /// The video's length without set_length().
    static constexpr si::Duration default_length{ au::seconds(10.0) };
    /// The frame rate of the video's playing time (the core's next_frame() rate: 60 frames per second).
    static constexpr si::Frequency frame_rate{ au::inverse(au::seconds)(60.0) };

    /// @brief Makes the recorder of an LBM.
    /// @param lbm the LBM whose graphics render the frames
    /// @param converter the simulation's conversions, for the cameras and the time steps
    VideoRecorder(LBM& lbm, const si::LatticeConverter& converter) : lbm_(lbm), converter_(converter) {}

    /// @brief Adds an unnamed fixed camera.
    /// @param view the camera in physical units
    /// @return this recorder
    VideoRecorder& add(const CameraView& view) { return add("", view); }

    /// @brief Adds an unnamed moving camera.
    /// @param path the view at each progress of the recording
    /// @return this recorder
    VideoRecorder& add(CameraPath path) { return add("", std::move(path)); }

    /// @brief Adds a named fixed camera.
    /// @param name the camera's name, its folder under export/
    /// @param view the camera in physical units
    /// @return this recorder
    VideoRecorder& add(const std::string& name, const CameraView& view) {
        CameraPath fixed{ [view](float) { return view; } };
        VideoRecorder& self{ add(name, std::move(fixed)) };
        return self;
    }

    /// @brief Adds a named moving camera.
    /// @param name the camera's name, its folder under export/
    /// @param path the view at each progress of the recording
    /// @return this recorder
    VideoRecorder& add(const std::string& name, CameraPath path) {
        views_.push_back(View{ .name = name, .path = std::move(path) });
        return *this;
    }

    /// @brief Sets the length of the video (default 10 s): 60 frames per second of it, spread over the simulated time.
    /// @param length the video's playing time, more than 0
    /// @return this recorder
    VideoRecorder& set_length(si::Duration length) {
        video_length_ = length;
        return *this;
    }

    /// @brief Writes a frame every interval of simulated time instead (from the start on).
    /// @param interval the simulated time between two frames
    /// @return this recorder
    VideoRecorder& set_frame_interval(si::Duration interval) {
        frame_interval_ = interval;
        return *this;
    }

    /// @brief Tells whether a camera was added, so that start() can record.
    /// @return true when a camera was added
    bool has_views() const { return !views_.empty(); }

    /// @brief Points the camera at the first view and schedules the frames with the runner.
    ///
    /// The recording spans the simulated time from now: both the frame times and the cameras' progress count from
    /// the time step at which start() is called, also when an earlier run came first. Ends the program with
    /// print_error() without a camera (add()), for a simulated time shorter than half a time step, or for
    /// contradictory camera settings (GraphicsConfig::apply_camera()).
    /// @param runner the simulation's runner, which writes the frames
    /// @param simulated_time the simulated time from now that makes the video
    void start(Runner& runner, si::Duration simulated_time) {
        if(views_.empty()) {
            print_error("VideoRecorder::start(): the video needs a camera: call add() first");
        }
        const CameraView first{ views_.front().path(0.0f) }; // the first view from the start
        GraphicsConfig::apply_camera(lbm_, converter_, first);
        const uint64_t start_step{ lbm_.get_t() };
        const uint64_t recorded_steps{ converter_.steps(simulated_time) };
        if(recorded_steps == 0u) {
            print_error("VideoRecorder::start(): the video needs a run of at least one time step");
        }
        const uint64_t total_steps{ start_step + recorded_steps };
        if(frame_interval_.has_value()) {
            schedule_every_interval(runner, frame_interval_.value(), start_step, total_steps);
        } else {
            schedule_at_frame_rate(runner, start_step, total_steps);
        }
    }

private:
    /// A camera.
    struct View {
        std::string name{}; ///< its name, empty for an unnamed camera
        CameraPath path{};  ///< its view at each progress of the recording
    };

    LBM& lbm_;                                     ///< the LBM whose graphics render the frames (not owned)
    si::LatticeConverter converter_;               ///< the simulation's conversions
    std::vector<View> views_{};                    ///< the cameras
    si::Duration video_length_{ default_length }; ///< set_length(): the video's playing time
    std::optional<si::Duration> frame_interval_{}; ///< set_frame_interval(), or none
    uint last_frame_{ 0u };                        ///< the number of the last frame written, as the core's last_exported_frame

    /// @brief Schedules a frame from each camera every interval of simulated time.
    /// @param runner the simulation's runner, which writes the frames
    /// @param interval the simulated time between two frames
    /// @param start_step the time step at which the recording starts
    /// @param total_steps the time step at which it ends, more than start_step
    void schedule_every_interval(Runner& runner, si::Duration interval, uint64_t start_step, uint64_t total_steps) {
        runner.every(interval, [this, start_step, total_steps](si::Duration) {
            const float progress{ progress_of(start_step, total_steps) };
            write_frames(progress);
        });
    }

    /// @brief Schedules a check at every time step that writes a frame from each camera when the video's next frame
    /// is due (frame_rate frames per second of playing time).
    /// @param runner the simulation's runner, which writes the frames
    /// @param start_step the time step at which the recording starts
    /// @param total_steps the time step at which it ends, more than start_step
    void schedule_at_frame_rate(Runner& runner, uint64_t start_step, uint64_t total_steps) {
        const si::Duration length{ video_length_ };
        runner.every_step([this, start_step, total_steps, length](si::Duration) {
            const float progress{ progress_of(start_step, total_steps) };
            const bool frame_due{ next_frame_due(progress, length) };
            if(frame_due) {
                write_frames(progress);
            }
        });
    }

    /// @brief Gives the progress of the recording at the current time step.
    /// @param start_step the time step at which the recording started
    /// @param total_steps the time step at which it ends, more than start_step
    /// @return (t - start_step) / (total_steps - start_step), from 0 to 1
    float progress_of(uint64_t start_step, uint64_t total_steps) const {
        const float done{ static_cast<float>(lbm_.get_t() - start_step) };
        const float span{ static_cast<float>(total_steps - start_step) };
        const float progress{ done / span };
        return progress;
    }

    /// @brief Tells whether the next frame of the video is due, as the core's next_frame() does but counted from the
    /// recording's start: the frame number is progress times the playing time times the frame rate.
    /// @param progress the progress of the recording, from 0 to 1
    /// @param length the video's playing time
    /// @return true once for each new frame number
    bool next_frame_due(float progress, si::Duration length) {
        const float length_s{ static_cast<float>(length.in(au::seconds)) }; // the core's float, in the core's order
        const float frames_per_second{ static_cast<float>(frame_rate.in(au::inverse(au::seconds))) };
        const float frame_time{ progress * length_s * frames_per_second };
        const uint frame{ to_uint(frame_time) };
        if(frame == last_frame_) {
            return false;
        }
        last_frame_ = frame;
        return true;
    }

    /// @brief Writes a frame from each camera.
    ///
    /// Ends the program with print_error() for contradictory camera settings (GraphicsConfig::apply_camera()).
    /// @param progress the progress of the recording, from 0 to 1
    void write_frames(float progress) const {
        for(const View& view : views_) {
            const CameraView frame_view{ view.path(progress) };
            GraphicsConfig::apply_camera(lbm_, converter_, frame_view);
            if(view.name.empty()) {
                lbm_.graphics.write_frame();
            } else {
                const std::string folder{ get_exe_path() + "export/" + view.name + "/" };
                lbm_.graphics.write_frame(folder);
            }
        }
    }
};
