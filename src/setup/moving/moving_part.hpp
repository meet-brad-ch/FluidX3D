#pragma once

#include "core/types.hpp"

/**
 * @enum RotationAxis
 * @brief Common rotation axes for convenience
 */
enum class RotationAxis {
    X,      ///< Rotate around X axis
    Y,      ///< Rotate around Y axis
    Z,      ///< Rotate around Z axis
    CUSTOM  ///< Use custom axis vector (set via set_rotation_axis(x,y,z))
};

/**
 * @enum MotionType
 * @brief Type of motion for the moving part
 */
enum class MotionType {
    ROTATION,   ///< Pure rotation about fixed axis (propellers, rotors) - efficient
    TUMBLING,   ///< Arbitrary axis rotation requiring unvoxelize (TIE Fighter style)
    ROLLING     ///< Wheel rotation following surface/vehicle motion
};

/**
 * @class MovingPart
 * @brief Configuration class for a single moving part using fluent interface
 *
 * This class holds configuration parameters for a moving part (propeller, wheel,
 * rotor) in FluidX3D simulations. It uses the fluent interface pattern for
 * readable configuration.
 *
 * @par Example - Propeller:
 * @code
 * MovingPart propeller("propeller.stl")
 *     .set_rotation_axis(RotationAxis::Y)
 *     .set_tip_speed_mps(100.0f)    // SI units: m/s
 *     .set_update_interval(4);       // Re-voxelize every 4 timesteps
 * @endcode
 *
 * @par Example - Helicopter Rotor:
 * @code
 * MovingPart rotor("main_rotor.stl")
 *     .set_rotation_axis(RotationAxis::Z)
 *     .set_rpm(348.0f);
 * @endcode
 *
 * @par Example - Wheel:
 * @code
 * MovingPart wheel("front_wheels.stl")
 *     .set_rotation_axis(RotationAxis::X)
 *     .set_rolling_velocity_mps(27.8f)  // Vehicle speed in m/s
 *     .set_offset_m(0.0f, 0.1f, 0.0f);  // Custom offset for ground clearance
 * @endcode
 *
 * @par Example - Tumbling Object:
 * @code
 * MovingPart tumbler("spacecraft.stl")
 *     .set_rotation_axis(0.2f, 1.0f, 0.1f)  // Custom axis
 *     .set_angular_velocity_radps(0.5f)
 *     .enable_tumbling();  // Uses unvoxelize/voxelize pattern
 * @endcode
 *
 * @note All spatial values are in SI units (meters, m/s, rad/s)
 * @note Angular velocity is specified in one of four ways: tip speed, RPM, rad/s, or rolling velocity
 */
class MovingPart {
public:
    /**
     * @brief Construct a MovingPart from STL filename
     * @param stl_filename STL filename (relative to resources/)
     *
     * The STL will be automatically scaled and positioned to match
     * the parent SimulationSetup geometry when added to MovingPartsManager.
     */
    explicit MovingPart(const string& stl_filename) : stl_filename_(stl_filename) {}

    // ========================================================================
    // Rotation Axis Configuration
    // ========================================================================

    /**
     * @brief Set rotation axis using predefined axis
     * @param axis Rotation axis (X, Y, or Z)
     * @return Reference for method chaining
     */
    MovingPart& set_rotation_axis(RotationAxis axis) {
        axis_type_ = axis;
        switch(axis) {
            case RotationAxis::X: rotation_axis_ = float3(1.0f, 0.0f, 0.0f); break;
            case RotationAxis::Y: rotation_axis_ = float3(0.0f, 1.0f, 0.0f); break;
            case RotationAxis::Z: rotation_axis_ = float3(0.0f, 0.0f, 1.0f); break;
            default: break;
        }
        return *this;
    }

    /**
     * @brief Set custom rotation axis
     * @param x X component of rotation axis
     * @param y Y component of rotation axis
     * @param z Z component of rotation axis
     * @return Reference for method chaining
     *
     * The axis will be normalized automatically.
     */
    MovingPart& set_rotation_axis(float32_t x, float32_t y, float32_t z) {
        axis_type_ = RotationAxis::CUSTOM;
        const float len = sqrt(x*x + y*y + z*z);
        rotation_axis_ = float3(x/len, y/len, z/len);
        return *this;
    }

    // ========================================================================
    // Angular Velocity Configuration (choose one)
    // ========================================================================

    /**
     * @brief Set angular velocity from tip speed in m/s (SI units)
     * @param tip_speed_mps Tip speed in meters per second
     * @return Reference for method chaining
     *
     * Angular velocity is calculated as: omega = tip_speed / radius
     * where radius is half the maximum dimension of the part.
     *
     * This is the recommended method for propellers/rotors.
     */
    MovingPart& set_tip_speed_mps(float32_t tip_speed_mps) {
        velocity_mode_ = VelocityMode::TIP_SPEED;
        tip_speed_mps_ = tip_speed_mps;
        is_tip_speed_lbm_ = false;
        return *this;
    }

    /**
     * @brief Set angular velocity from tip speed in LBM units
     *
     * Use this for Reynolds number based simulations where velocity is
     * already in LBM units (no SI conversion needed).
     *
     * @param lbm_tip_speed Tip speed in LBM units
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * sim.configure_reynolds(1000000.0f, 0.1f);
     * parts.add(MovingPart("rotor.stl")
     *     .set_rotation_axis(RotationAxis::Y)
     *     .set_tip_speed_lbm(sim.get_lbm_reference_velocity()));
     * @endcode
     */
    MovingPart& set_tip_speed_lbm(float32_t lbm_tip_speed) {
        velocity_mode_ = VelocityMode::TIP_SPEED;
        tip_speed_lbm_ = lbm_tip_speed;
        is_tip_speed_lbm_ = true;
        return *this;
    }

    /**
     * @brief Set angular velocity from RPM (SI units)
     * @param rpm Revolutions per minute
     * @return Reference for method chaining
     *
     * Angular velocity is calculated as: omega = rpm * 2*pi / 60
     */
    MovingPart& set_rpm(float32_t rpm) {
        velocity_mode_ = VelocityMode::RPM;
        rpm_ = rpm;
        return *this;
    }

    /**
     * @brief Set angular velocity directly in rad/s (SI units)
     * @param omega_radps Angular velocity in radians per second
     * @return Reference for method chaining
     */
    MovingPart& set_angular_velocity_radps(float32_t omega_radps) {
        velocity_mode_ = VelocityMode::DIRECT;
        angular_velocity_radps_ = omega_radps;
        return *this;
    }

    /**
     * @brief Set rolling velocity for wheels (SI units)
     * @param velocity_mps Vehicle/surface velocity in m/s
     * @return Reference for method chaining
     *
     * Angular velocity is calculated as: omega = velocity / radius
     * where radius is half the minimum dimension (wheel radius).
     */
    MovingPart& set_rolling_velocity_mps(float32_t velocity_mps) {
        velocity_mode_ = VelocityMode::ROLLING;
        motion_type_ = MotionType::ROLLING;
        rolling_velocity_mps_ = velocity_mps;
        return *this;
    }

    // ========================================================================
    // Position/Offset Configuration
    // ========================================================================

    /**
     * @brief Set position offset from auto-calculated position (SI units)
     * @param x X offset in meters
     * @param y Y offset in meters
     * @param z Z offset in meters
     * @return Reference for method chaining
     *
     * By default, the part inherits the same scale and offset as the body.
     * Use this to specify an additional offset (e.g., for wheel ground clearance).
     */
    MovingPart& set_offset_m(float32_t x, float32_t y, float32_t z) {
        has_custom_offset_ = true;
        offset_m_ = float3(x, y, z);
        return *this;
    }

    /**
     * @brief Set position offset as ratio of reference size
     *
     * Use this for Reynolds number based simulations where positions are
     * specified relative to the geometry's reference size.
     *
     * @param x X offset as ratio of reference size
     * @param y Y offset as ratio of reference size
     * @param z Z offset as ratio of reference size
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * // Rotor offset: 0.21 reference lengths behind the stator
     * parts.add(MovingPart("rotor.stl")
     *     .set_rotation_axis(RotationAxis::Y)
     *     .set_tip_speed_lbm(0.1f)
     *     .set_offset_ratio(0.0f, -0.21f, 0.0f));
     * @endcode
     */
    MovingPart& set_offset_ratio(float32_t x, float32_t y, float32_t z) {
        has_offset_ratio_ = true;
        offset_ratio_ = float3(x, y, z);
        return *this;
    }

    // ========================================================================
    // Motion Type Configuration
    // ========================================================================

    /**
     * @brief Set motion type
     * @param type MotionType (ROTATION, TUMBLING, or ROLLING)
     * @return Reference for method chaining
     *
     * - ROTATION: Pure rotation about fixed axis (default, efficient)
     * - TUMBLING: Arbitrary rotation requiring unvoxelize (TIE Fighter style)
     * - ROLLING: Wheel-style rotation
     */
    MovingPart& set_motion_type(MotionType type) {
        motion_type_ = type;
        return *this;
    }

    /**
     * @brief Enable tumbling mode for arbitrary rotation
     * @return Reference for method chaining
     *
     * Convenience method equivalent to set_motion_type(MotionType::TUMBLING).
     * Uses unvoxelize/voxelize pattern for arbitrary axis rotation.
     */
    MovingPart& enable_tumbling() {
        motion_type_ = MotionType::TUMBLING;
        return *this;
    }

    /**
     * @brief Configure tumbling with arbitrary axis and fixed rotation angle
     * @param axis Rotation axis (will be normalized)
     * @param angle_per_update Rotation angle in radians per update interval
     * @return Reference for method chaining
     *
     * Use this for objects tumbling through the domain with arbitrary rotation.
     * Unlike velocity-based rotation, this uses a fixed rotation angle per update.
     *
     * @par Example - TIE Fighter tumbling:
     * @code
     * parts.add(MovingPart("tie_fighter.stl")
     *     .set_tumble(float3(0.2f, 1.0f, 0.1f), radians(0.4032f))
     *     .set_update_interval(28));
     * @endcode
     */
    MovingPart& set_tumble(float3 axis, float32_t angle_per_update) {
        motion_type_ = MotionType::TUMBLING;
        const float len = sqrt(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
        rotation_axis_ = float3(axis.x/len, axis.y/len, axis.z/len);
        tumble_angle_ = angle_per_update;
        use_tumble_angle_ = true;
        return *this;
    }

    /**
     * @brief Mark part as static (voxelized once with velocity, no updates)
     * @return Reference for method chaining
     *
     * Use this for wheels or other parts that rotate in place without
     * needing re-voxelization. The part is voxelized once during initialize()
     * with the appropriate angular velocity, then skipped during update().
     *
     * @par Example - Rolling wheels:
     * @code
     * parts.add(MovingPart("front_wheels.stl")
     *     .set_rotation_axis(RotationAxis::X)
     *     .set_rolling_velocity_mps(car_speed_mps)
     *     .set_static());
     * @endcode
     */
    MovingPart& set_static() {
        is_static_ = true;
        return *this;
    }

    // ========================================================================
    // Timing Configuration
    // ========================================================================

    /**
     * @brief Set re-voxelization interval
     * @param timesteps Number of LBM timesteps between re-voxelizations
     * @return Reference for method chaining
     *
     * Default: 4 timesteps (typical for most simulations)
     * Lower values = more accurate but slower
     * Higher values = faster but less accurate
     */
    MovingPart& set_update_interval(uint32_t timesteps) {
        update_interval_ = timesteps;
        return *this;
    }

    // ========================================================================
    // Rotation Direction
    // ========================================================================

    /**
     * @brief Reverse rotation direction
     * @return Reference for method chaining
     *
     * Multiplies angular velocity by -1. Useful for counter-rotating
     * propellers or opposite-side wheels.
     */
    MovingPart& reverse_direction() {
        direction_multiplier_ = -1.0f;
        return *this;
    }

    // ========================================================================
    // Getters (for MovingPartsManager)
    // ========================================================================

    const string& get_stl_filename() const { return stl_filename_; }
    const float3& get_rotation_axis() const { return rotation_axis_; }
    MotionType get_motion_type() const { return motion_type_; }
    uint32_t get_update_interval() const { return update_interval_; }
    float32_t get_direction_multiplier() const { return direction_multiplier_; }
    bool has_custom_offset() const { return has_custom_offset_; }
    const float3& get_offset_m() const { return offset_m_; }
    bool has_offset_ratio() const { return has_offset_ratio_; }
    const float3& get_offset_ratio() const { return offset_ratio_; }

    enum class VelocityMode { TIP_SPEED, RPM, DIRECT, ROLLING };
    VelocityMode get_velocity_mode() const { return velocity_mode_; }
    float32_t get_tip_speed_mps() const { return tip_speed_mps_; }
    float32_t get_tip_speed_lbm() const { return tip_speed_lbm_; }
    bool is_tip_speed_in_lbm_units() const { return is_tip_speed_lbm_; }
    float32_t get_rpm() const { return rpm_; }
    float32_t get_angular_velocity_radps() const { return angular_velocity_radps_; }
    float32_t get_rolling_velocity_mps() const { return rolling_velocity_mps_; }

    // Tumble mode getters
    bool uses_tumble_angle() const { return use_tumble_angle_; }
    float32_t get_tumble_angle() const { return tumble_angle_; }

    // Static mode getter
    bool is_static() const { return is_static_; }

private:
    string stl_filename_;
    RotationAxis axis_type_{RotationAxis::Y};
    float3 rotation_axis_{0.0f, 1.0f, 0.0f};  // Default: Y axis

    // Angular velocity (exactly one mode should be used)
    VelocityMode velocity_mode_{VelocityMode::TIP_SPEED};
    float32_t tip_speed_mps_{0.0f};
    float32_t tip_speed_lbm_{0.0f};
    bool is_tip_speed_lbm_{false};
    float32_t rpm_{0.0f};
    float32_t angular_velocity_radps_{0.0f};
    float32_t rolling_velocity_mps_{0.0f};

    // Motion configuration
    MotionType motion_type_{MotionType::ROTATION};
    uint32_t update_interval_{4};
    float32_t direction_multiplier_{1.0f};

    // Position offset (SI units)
    bool has_custom_offset_{false};
    float3 offset_m_{0.0f, 0.0f, 0.0f};

    // Position offset (ratio of reference size, for Re-based simulations)
    bool has_offset_ratio_{false};
    float3 offset_ratio_{0.0f, 0.0f, 0.0f};

    // Tumble mode (fixed angle per update instead of velocity-based)
    bool use_tumble_angle_{false};
    float32_t tumble_angle_{0.0f};

    // Static mode (voxelized once with velocity, no updates)
    bool is_static_{false};
};
