#pragma once

#include "core/types.hpp"
#include "units.hpp"

/**
 * @file unit_helpers.hpp
 * @brief Unit conversion helper functions and utilities
 *
 * Provides standalone functions for common unit conversions that can be
 * used independently of SimulationSetup.
 */

namespace UnitHelpers {

// ============================================================================
// Physical Constants
// ============================================================================

/** @brief Standard gravitational acceleration (m/s^2) */
constexpr float32_t GRAVITY_STANDARD = 9.80665f;

/** @brief Air density at sea level, 15C (kg/m^3) */
constexpr float32_t AIR_DENSITY_SL = 1.225f;

/** @brief Water density at 20C (kg/m^3) */
constexpr float32_t WATER_DENSITY = 998.2f;

/** @brief Air kinematic viscosity at 15C (m^2/s) */
constexpr float32_t AIR_VISCOSITY = 1.48e-5f;

/** @brief Water kinematic viscosity at 20C (m^2/s) */
constexpr float32_t WATER_VISCOSITY = 1.004e-6f;

// ============================================================================
// Reynolds Number Calculations
// ============================================================================

/**
 * @brief Calculate Reynolds number from flow parameters
 *
 * @param length Reference length in meters
 * @param velocity Flow velocity in m/s
 * @param kinematic_viscosity Kinematic viscosity in m^2/s
 * @return Reynolds number (dimensionless)
 */
inline float32_t reynolds_number(float32_t length, float32_t velocity,
                                  float32_t kinematic_viscosity) {
    return length * velocity / kinematic_viscosity;
}

/**
 * @brief Calculate LBM viscosity from Reynolds number
 *
 * @param Re Reynolds number
 * @param lbm_length Reference length in LBM units
 * @param lbm_velocity Reference velocity in LBM units
 * @return Kinematic viscosity in LBM units
 */
inline float32_t lbm_viscosity_from_re(float32_t Re, float32_t lbm_length,
                                        float32_t lbm_velocity) {
    return lbm_velocity * lbm_length / Re;
}

/**
 * @brief Calculate LBM velocity for stable simulation
 *
 * Returns a safe LBM velocity that ensures stability. The Mach number
 * should be kept low (< 0.1) for incompressible flow assumption.
 *
 * @param mach_number Target Mach number (default: 0.1)
 * @return Safe LBM velocity
 */
inline float32_t safe_lbm_velocity(float32_t mach_number = 0.1f) {
    // Speed of sound in LBM is 1/sqrt(3) ≈ 0.577
    constexpr float32_t LBM_SPEED_OF_SOUND = 0.57735026919f;
    return mach_number * LBM_SPEED_OF_SOUND;
}

// ============================================================================
// Unit Conversion Wrappers
// ============================================================================

/**
 * @brief Convert time in seconds to LBM timesteps
 *
 * @param units Reference to Units object
 * @param seconds Time in seconds
 * @return Number of LBM timesteps
 */
inline uint64_t seconds_to_timesteps(const Units& units, float32_t seconds) {
    return units.t(seconds);
}

/**
 * @brief Convert SI velocity to LBM velocity
 *
 * @param units Reference to Units object
 * @param si_velocity Velocity in m/s
 * @return Velocity in LBM units
 */
inline float32_t si_to_lbm_velocity(const Units& units, float32_t si_velocity) {
    return units.u(si_velocity);
}

/**
 * @brief Convert LBM velocity to SI velocity
 *
 * @param units Reference to Units object
 * @param lbm_velocity Velocity in LBM units
 * @return Velocity in m/s
 */
inline float32_t lbm_to_si_velocity(const Units& units, float32_t lbm_velocity) {
    return units.si_u(lbm_velocity);
}

/**
 * @brief Convert SI length to LBM cells
 *
 * @param units Reference to Units object
 * @param si_length Length in meters
 * @return Length in LBM cells
 */
inline float32_t si_to_lbm_length(const Units& units, float32_t si_length) {
    return units.x(si_length);
}

/**
 * @brief Convert gravity to LBM force density
 *
 * @param units Reference to Units object
 * @param density Fluid density in kg/m^3
 * @param gravity Gravitational acceleration in m/s^2 (default: 9.81)
 * @return Force density in LBM units
 */
inline float32_t gravity_to_lbm_force(const Units& units, float32_t density,
                                       float32_t gravity = GRAVITY_STANDARD) {
    return units.f(density, gravity);
}

// ============================================================================
// Simulation Time Helpers
// ============================================================================

/**
 * @brief Calculate flow-through time
 *
 * Time for fluid to travel the reference length at reference velocity.
 *
 * @param length Reference length in meters
 * @param velocity Flow velocity in m/s
 * @return Flow-through time in seconds
 */
inline float32_t flow_through_time(float32_t length, float32_t velocity) {
    return length / velocity;
}

/**
 * @brief Calculate timesteps for multiple flow-throughs
 *
 * @param units Reference to Units object
 * @param length Reference length in meters
 * @param velocity Flow velocity in m/s
 * @param num_flow_throughs Number of flow-through times
 * @return Number of LBM timesteps
 */
inline uint64_t flow_through_timesteps(const Units& units, float32_t length,
                                        float32_t velocity, float32_t num_flow_throughs) {
    return units.t(num_flow_throughs * flow_through_time(length, velocity));
}

} // namespace UnitHelpers
