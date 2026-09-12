#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include <optional>

enum class RotationAxis { X, Y, Z };

enum class MotionType {
    ROTATION, // fixed axis (propellers, rotors): re-voxelized with angular velocity
    TUMBLING  // arbitrary axis, turned in steps: unvoxelized and re-voxelized
};

// One moving part for MovingPartsManager; the STL (in resources/) gets the transform of the SimulationSetup model.
//   MovingPart("rotor.stl").set_rotation_axis(RotationAxis::Z).set_tip_speed(100.0_mps)
//   MovingPart("tie_fighter.stl").set_tumble(float3(0.2f, 1.0f, 0.1f), 180_deg / 1.0_s).set_update_interval(2.2_ms)
class MovingPart {
public:
    explicit MovingPart(const string& stl_filename) : stl_filename_(stl_filename) {}

    MovingPart& set_rotation_axis(RotationAxis axis) {
        switch(axis) {
            case RotationAxis::X: rotation_axis_ = float3(1.0f, 0.0f, 0.0f); break;
            case RotationAxis::Y: rotation_axis_ = float3(0.0f, 1.0f, 0.0f); break;
            case RotationAxis::Z: rotation_axis_ = float3(0.0f, 0.0f, 1.0f); break;
        }
        return *this;
    }

    /// The speed at half the part's largest dimension (a rotor's blade tips).
    MovingPart& set_tip_speed(Speed tip_speed) {
        tip_speed_ = tip_speed;
        return *this;
    }

    /// @brief For an STL in other coordinates than the model's: its bounding box center goes to the model's plus this
    /// offset. By default a part keeps its place relative to the model, as in the CAD assembly both files come from.
    MovingPart& centered_on_model(Position offset = {}) {
        centered_offset_ = offset;
        return *this;
    }

    /// Tumbles around this axis (in the domain's coordinates) at this rate, turned in steps at each update.
    MovingPart& set_tumble(float3 axis, AngularSpeed rate) {
        motion_type_ = MotionType::TUMBLING;
        const float len = sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
        rotation_axis_ = float3(axis.x/len, axis.y/len, axis.z/len);
        tumble_rate_ = rate;
        return *this;
    }

    /// @brief The simulated time between re-voxelizations. By default the part is re-voxelized whenever its tip has
    /// moved half a cell.
    MovingPart& set_update_interval(Duration interval) {
        update_interval_ = interval;
        return *this;
    }

    MovingPart& reverse_direction() { // counter-rotating parts
        direction_multiplier_ = -1.0f;
        return *this;
    }

    const string& get_stl_filename() const { return stl_filename_; }
    const float3& get_rotation_axis() const { return rotation_axis_; }
    MotionType get_motion_type() const { return motion_type_; }
    const std::optional<Duration>& get_update_interval() const { return update_interval_; } ///< set by set_update_interval()
    float32_t get_direction_multiplier() const { return direction_multiplier_; }
    const std::optional<Position>& get_centered_offset() const { return centered_offset_; } ///< set by centered_on_model()
    Speed get_tip_speed() const { return tip_speed_; }
    AngularSpeed get_tumble_rate() const { return tumble_rate_; }

private:
    string stl_filename_;
    float3 rotation_axis_{0.0f, 1.0f, 0.0f};
    Speed tip_speed_{};
    AngularSpeed tumble_rate_{};

    MotionType motion_type_{MotionType::ROTATION};
    std::optional<Duration> update_interval_;
    float32_t direction_multiplier_{1.0f};

    std::optional<Position> centered_offset_;
};
