#pragma once

/**
 * @file camera_presets.hpp
 * @brief Shared camera preset constants for FluidX3D visualization
 *
 * These constants define standard camera views used by both CameraConfig
 * and GraphicsConfig classes, ensuring consistency across the API.
 */

namespace CameraPresets {

// ============================================================================
// Standard View Angles (in degrees)
// ============================================================================

/** @brief Top-down view: looking down Z axis */
constexpr float TOP_VIEW_PITCH = 90.0f;
constexpr float TOP_VIEW_YAW = 180.0f;

/** @brief Side view: looking along X axis */
constexpr float SIDE_VIEW_PITCH = 0.0f;
constexpr float SIDE_VIEW_YAW = 90.0f;

/** @brief Front view: looking along Y axis */
constexpr float FRONT_VIEW_PITCH = 0.0f;
constexpr float FRONT_VIEW_YAW = 180.0f;

/** @brief Isometric view: 30-degree pitch, 135-degree yaw */
constexpr float ISOMETRIC_PITCH = 30.0f;
constexpr float ISOMETRIC_YAW = 135.0f;

// ============================================================================
// Default Values
// ============================================================================

/** @brief Default field of view in degrees */
constexpr float DEFAULT_FOV = 100.0f;

/** @brief Default zoom level */
constexpr float DEFAULT_ZOOM = 1.0f;

/** @brief Degrees to radians conversion factor */
constexpr float DEG_TO_RAD = 3.14159265f / 180.0f;

} // namespace CameraPresets
