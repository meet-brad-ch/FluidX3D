#pragma once

#include "core/types.hpp"
#include "boundaries/boundary_flags.hpp"

/**
 * @file boundary_utils.hpp
 * @brief Shared boundary utility functions for FluidX3D setup API
 *
 * Provides common boundary-related helpers used across multiple builder classes
 * to avoid code duplication (DRY principle).
 */

namespace boundary_utils {

/**
 * @brief Check if a cell is on a specified domain face
 *
 * @param x Cell X coordinate
 * @param y Cell Y coordinate
 * @param z Cell Z coordinate
 * @param Nx Domain size in X
 * @param Ny Domain size in Y
 * @param Nz Domain size in Z
 * @param face Which face to check
 * @param offset Distance from face (0 = at face, 1 = one cell inward, etc.)
 * @return true if cell is on the specified face at given offset
 */
inline bool is_on_face(uint32_t x, uint32_t y, uint32_t z,
                       uint32_t Nx, uint32_t Ny, uint32_t Nz,
                       Face face, uint32_t offset = 0) {
    switch (face) {
        case Face::X_MIN: return x == offset;
        case Face::X_MAX: return x == Nx - 1 - offset;
        case Face::Y_MIN: return y == offset;
        case Face::Y_MAX: return y == Ny - 1 - offset;
        case Face::Z_MIN: return z == offset;
        case Face::Z_MAX: return z == Nz - 1 - offset;
    }
    return false;
}

/**
 * @brief Get the axis perpendicular to a face
 *
 * @param face The face
 * @return Axis perpendicular to that face
 */
inline Axis face_to_axis(Face face) {
    switch (face) {
        case Face::X_MIN:
        case Face::X_MAX: return Axis::X;
        case Face::Y_MIN:
        case Face::Y_MAX: return Axis::Y;
        case Face::Z_MIN:
        case Face::Z_MAX: return Axis::Z;
    }
    return Axis::X;
}

/**
 * @brief Get velocity sign for a face
 *
 * Returns +1 for MIN faces (flow entering domain)
 * Returns -1 for MAX faces (flow exiting domain)
 *
 * @param face The face
 * @return Sign multiplier for velocity
 */
inline int face_sign(Face face) {
    switch (face) {
        case Face::X_MIN:
        case Face::Y_MIN:
        case Face::Z_MIN: return 1;
        case Face::X_MAX:
        case Face::Y_MAX:
        case Face::Z_MAX: return -1;
    }
    return 1;
}

/**
 * @brief Check if a face is a MIN face
 *
 * @param face The face
 * @return true if face is X_MIN, Y_MIN, or Z_MIN
 */
inline bool is_min_face(Face face) {
    return face == Face::X_MIN || face == Face::Y_MIN || face == Face::Z_MIN;
}

/**
 * @brief Get the opposite face
 *
 * @param face The face
 * @return The face on the opposite side of the domain
 */
inline Face opposite_face(Face face) {
    switch (face) {
        case Face::X_MIN: return Face::X_MAX;
        case Face::X_MAX: return Face::X_MIN;
        case Face::Y_MIN: return Face::Y_MAX;
        case Face::Y_MAX: return Face::Y_MIN;
        case Face::Z_MIN: return Face::Z_MAX;
        case Face::Z_MAX: return Face::Z_MIN;
    }
    return face;
}

/**
 * @brief Get domain size along an axis
 *
 * @param axis The axis
 * @param Nx Domain X size
 * @param Ny Domain Y size
 * @param Nz Domain Z size
 * @return Size along specified axis
 */
inline uint32_t get_domain_size(Axis axis, uint32_t Nx, uint32_t Ny, uint32_t Nz) {
    switch (axis) {
        case Axis::X: return Nx;
        case Axis::Y: return Ny;
        case Axis::Z: return Nz;
    }
    return Nx;
}

/**
 * @brief Get cell coordinate along an axis
 *
 * @param axis The axis
 * @param x Cell X coordinate
 * @param y Cell Y coordinate
 * @param z Cell Z coordinate
 * @return Coordinate along specified axis
 */
inline uint32_t get_coordinate(Axis axis, uint32_t x, uint32_t y, uint32_t z) {
    switch (axis) {
        case Axis::X: return x;
        case Axis::Y: return y;
        case Axis::Z: return z;
    }
    return x;
}

} // namespace boundary_utils
