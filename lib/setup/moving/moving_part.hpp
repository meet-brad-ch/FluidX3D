#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include <optional>

enum class RotationAxis { X, Y, Z };

enum class MotionType {
    ROTATION, // fixed axis (propellers, rotors): re-voxelized with angular velocity
    TUMBLING  // arbitrary axis, fixed angle per update: unvoxelized and re-voxelized
};

// One moving part for MovingPartsManager; the STL (in resources/) gets the transform of the SimulationSetup model.
//   MovingPart("rotor.stl").set_rotation_axis(RotationAxis::Z).set_tip_speed(100.0_mps).set_update_interval(4)
//   MovingPart("tie_fighter.stl").set_tumble(float3(0.2f, 1.0f, 0.1f), radians(0.4032f)).set_update_interval(28)
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

    // tumble around axis (normalized) by angle_per_update radians every update interval
    MovingPart& set_tumble(float3 axis, float32_t angle_per_update) {
        motion_type_ = MotionType::TUMBLING;
        const float len = sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
        rotation_axis_ = float3(axis.x/len, axis.y/len, axis.z/len);
        tumble_angle_ = angle_per_update;
        use_tumble_angle_ = true;
        return *this;
    }

    // LBM time steps between re-voxelizations (default 4)
    MovingPart& set_update_interval(uint32_t timesteps) {
        update_interval_ = timesteps;
        return *this;
    }

    MovingPart& reverse_direction() { // counter-rotating parts
        direction_multiplier_ = -1.0f;
        return *this;
    }

    const string& get_stl_filename() const { return stl_filename_; }
    const float3& get_rotation_axis() const { return rotation_axis_; }
    MotionType get_motion_type() const { return motion_type_; }
    uint32_t get_update_interval() const { return update_interval_; }
    float32_t get_direction_multiplier() const { return direction_multiplier_; }
    const std::optional<Position>& get_centered_offset() const { return centered_offset_; } ///< set by centered_on_model()
    Speed get_tip_speed() const { return tip_speed_; }
    bool uses_tumble_angle() const { return use_tumble_angle_; }
    float32_t get_tumble_angle() const { return tumble_angle_; }

private:
    string stl_filename_;
    float3 rotation_axis_{0.0f, 1.0f, 0.0f};
    Speed tip_speed_{};

    MotionType motion_type_{MotionType::ROTATION};
    uint32_t update_interval_{4};
    float32_t direction_multiplier_{1.0f};

    std::optional<Position> centered_offset_;

    bool use_tumble_angle_{false};
    float32_t tumble_angle_{0.0f};
};
