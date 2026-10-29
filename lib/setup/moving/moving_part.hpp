#pragma once

#include "setup/core/types.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include "utilities.hpp" // the core's float3 and string
#include <cmath>
#include <optional>

/// @file moving_part.hpp
/// @brief One moving part of a model, such as a rotor (MovingPart).

/// How a moving part moves.
enum class MotionType {
    ROTATION, ///< about a fixed axis (propellers, rotors): re-voxelized with its angular velocity
    TUMBLING  ///< about any axis, turned in steps: unvoxelized and re-voxelized
};

/// @brief One moving part for MovingPartsManager: an STL file (in resources/) that gets the transform of the
/// simulation's model, and its motion.
///
/// A value: the setters return the part, so it is built in one expression and handed to MovingPartsManager::add().
/// @code
/// using namespace si::symbols;
/// MovingPart("rotor.stl").set_rotation_axis(Axis::Z).set_tip_speed(100.0 * m / s)
/// MovingPart("tie_fighter.stl").set_tumble(si::Direction{ 0.2, 1.0, 0.1 }, 180.0 * deg / s).set_update_interval(2.2 * ms)
/// @endcode
class MovingPart {
public:
    /// @brief Names the part's STL file. The file is looked up in resources/ by MovingPartsManager::initialize().
    /// @param stl_filename the part's STL file name, in resources/
    explicit MovingPart(const string& stl_filename) : stl_filename_(stl_filename) {}

    /// @brief Turns the part about a coordinate axis (default Y) through its centre of mass.
    /// @param axis the coordinate axis of the rotation
    /// @return this part
    MovingPart& set_rotation_axis(Axis axis) {
        const float along_x{ axis == Axis::X ? 1.0f : 0.0f };
        const float along_y{ axis == Axis::Y ? 1.0f : 0.0f };
        const float along_z{ axis == Axis::Z ? 1.0f : 0.0f };
        rotation_axis_ = float3(along_x, along_y, along_z);
        return *this;
    }

    /// @brief Sets the speed at half the part's largest dimension (a rotor's blade tips), which sets its rotation rate.
    /// @param tip_speed the speed of the part at half its largest dimension
    /// @return this part
    MovingPart& set_tip_speed(si::Speed tip_speed) {
        tip_speed_ = tip_speed;
        return *this;
    }

    /// @brief Places an STL in other coordinates than the model's: its bounding box centre goes to the model's plus an
    /// offset. By default a part keeps its place relative to the model, as in the CAD assembly both files come from.
    /// @param offset the offset from the model's centre in metres
    /// @return this part
    MovingPart& centered_on_model(si::Position offset = {}) {
        centered_offset_ = offset;
        return *this;
    }

    /// @brief Tumbles the part about an axis at a rate, turned in steps at each update.
    ///
    /// Ends the program with print_error() for the zero vector.
    /// @param axis a direction in the domain's coordinates (any length but 0; it is made a unit vector)
    /// @param rate the rotation rate
    /// @return this part
    MovingPart& set_tumble(si::Direction axis, si::AngularSpeed rate) {
        const double length{ axis.magnitude() };
        if(length == 0.0) {
            print_error("MovingPart::set_tumble(): the axis must not be the zero vector");
        }
        const si::Direction unit{ axis / length };
        motion_type_ = MotionType::TUMBLING;
        rotation_axis_ = float3(static_cast<float>(unit.x), static_cast<float>(unit.y), static_cast<float>(unit.z));
        tumble_rate_ = rate;
        return *this;
    }

    /// @brief Sets the simulated time between re-voxelizations. By default the part is re-voxelized whenever its tip
    /// has moved half a cell.
    /// @param interval the simulated time between two re-voxelizations
    /// @return this part
    MovingPart& set_update_interval(si::Duration interval) {
        update_interval_ = interval;
        return *this;
    }

    /// @brief Turns the part the other way (counter-rotating parts).
    /// @return this part
    MovingPart& reverse_direction() {
        direction_multiplier_ = -1.0f;
        return *this;
    }

    /// @return the STL file name, in resources/
    const string& get_stl_filename() const { return stl_filename_; }

    /// @return the unit vector of the rotation or tumble axis, as the core's float3
    const float3& get_rotation_axis() const { return rotation_axis_; }

    /// @return ROTATION, or TUMBLING after set_tumble()
    MotionType get_motion_type() const { return motion_type_; }

    /// @return the interval of set_update_interval(), or none
    const std::optional<si::Duration>& get_update_interval() const { return update_interval_; }

    /// @return -1 after reverse_direction(), else 1
    float32_t get_direction_multiplier() const { return direction_multiplier_; }

    /// @return the offset of centered_on_model(), or none
    const std::optional<si::Position>& get_centered_offset() const { return centered_offset_; }

    /// @return the tip speed (0 when not set)
    si::Speed get_tip_speed() const { return tip_speed_; }

    /// @return the tumble rate (0 when not set)
    si::AngularSpeed get_tumble_rate() const { return tumble_rate_; }

private:
    string stl_filename_{};                    ///< the STL file name, in resources/
    float3 rotation_axis_{ 0.0f, 1.0f, 0.0f }; ///< the unit vector of the rotation or tumble axis
    si::Speed tip_speed_{ au::ZERO };          ///< set_tip_speed()
    si::AngularSpeed tumble_rate_{ au::ZERO }; ///< set_tumble()

    MotionType motion_type_{ MotionType::ROTATION }; ///< ROTATION, or TUMBLING after set_tumble()
    std::optional<si::Duration> update_interval_{};  ///< set_update_interval(), or none
    float32_t direction_multiplier_{ 1.0f };         ///< -1 after reverse_direction(), else 1

    std::optional<si::Position> centered_offset_{}; ///< centered_on_model(), or none
};
