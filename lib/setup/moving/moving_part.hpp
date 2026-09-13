#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "utilities.hpp" // the core's float3 and string
#include <optional>

/// How a moving part moves.
enum class MotionType {
    ROTATION, ///< fixed axis (propellers, rotors): re-voxelized with angular velocity
    TUMBLING  ///< arbitrary axis, turned in steps: unvoxelized and re-voxelized
};

/// @brief One moving part for MovingPartsManager: an STL (in resources/) that gets the transform of the simulation's model.
/// @code
/// MovingPart("rotor.stl").set_rotation_axis(Axis::Z).set_tip_speed(100.0_mps)
/// MovingPart("tie_fighter.stl").set_tumble(float3(0.2f, 1.0f, 0.1f), 180_deg / 1.0_s).set_update_interval(2.2_ms)
/// @endcode
class MovingPart {
public:
    /// @param stl_filename the part's STL file, in resources/
    explicit MovingPart(const string& stl_filename) : stl_filename_(stl_filename) {}

    /// @brief Turns about an axis (default Y) through its center of mass.
    /// @param axis the axis
    /// @return this part
    MovingPart& set_rotation_axis(Axis axis) {
        rotation_axis_ = float3(axis == Axis::X ? 1.0f : 0.0f, axis == Axis::Y ? 1.0f : 0.0f, axis == Axis::Z ? 1.0f : 0.0f);
        return *this;
    }

    /// @brief The speed at half the part's largest dimension (a rotor's blade tips).
    /// @param tip_speed the speed
    /// @return this part
    MovingPart& set_tip_speed(Speed tip_speed) {
        tip_speed_ = tip_speed;
        return *this;
    }

    /// @brief For an STL in other coordinates than the model's: its bounding box center goes to the model's plus an
    /// offset. By default a part keeps its place relative to the model, as in the CAD assembly both files come from.
    /// @param offset the offset from the model's center
    /// @return this part
    MovingPart& centered_on_model(Position offset = {}) {
        centered_offset_ = offset;
        return *this;
    }

    /// @brief Tumbles around an axis at a rate, turned in steps at each update.
    /// @param axis a direction in the domain's coordinates (not the zero vector)
    /// @param rate the rate of rotation
    /// @return this part
    MovingPart& set_tumble(float3 axis, AngularSpeed rate) {
        const float len = sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
        if(len == 0.0f) print_error("MovingPart::set_tumble(): the axis must not be the zero vector");
        motion_type_ = MotionType::TUMBLING;
        rotation_axis_ = float3(axis.x/len, axis.y/len, axis.z/len);
        tumble_rate_ = rate;
        return *this;
    }

    /// @brief The simulated time between re-voxelizations. By default the part is re-voxelized whenever its tip has
    /// moved half a cell.
    /// @param interval the interval
    /// @return this part
    MovingPart& set_update_interval(Duration interval) {
        update_interval_ = interval;
        return *this;
    }

    /// @brief Turns the other way (counter-rotating parts).
    /// @return this part
    MovingPart& reverse_direction() {
        direction_multiplier_ = -1.0f;
        return *this;
    }

    /// @return the STL file, in resources/
    const string& get_stl_filename() const { return stl_filename_; }

    /// @return the unit vector of the rotation or tumble axis
    const float3& get_rotation_axis() const { return rotation_axis_; }

    /// @return rotation, or tumbling after set_tumble()
    MotionType get_motion_type() const { return motion_type_; }

    /// @return the interval of set_update_interval(), or none
    const std::optional<Duration>& get_update_interval() const { return update_interval_; }

    /// @return -1 after reverse_direction(), else 1
    float32_t get_direction_multiplier() const { return direction_multiplier_; }

    /// @return the offset of centered_on_model(), or none
    const std::optional<Position>& get_centered_offset() const { return centered_offset_; }

    /// @return the tip speed
    Speed get_tip_speed() const { return tip_speed_; }

    /// @return the tumble rate
    AngularSpeed get_tumble_rate() const { return tumble_rate_; }

private:
    string stl_filename_;                     ///< the STL file, in resources/
    float3 rotation_axis_{0.0f, 1.0f, 0.0f};  ///< the unit vector of the rotation or tumble axis
    Speed tip_speed_{};                       ///< set_tip_speed()
    AngularSpeed tumble_rate_{};              ///< set_tumble()

    MotionType motion_type_{MotionType::ROTATION}; ///< rotation, or tumbling after set_tumble()
    std::optional<Duration> update_interval_;      ///< set_update_interval()
    float32_t direction_multiplier_{1.0f};         ///< -1 after reverse_direction()

    std::optional<Position> centered_offset_; ///< centered_on_model()
};
