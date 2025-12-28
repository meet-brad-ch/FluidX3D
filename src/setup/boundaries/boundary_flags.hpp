#pragma once

#include <cstdint>

/**
 * @file boundary_flags.hpp
 * @brief Enum flags for boundary condition configuration
 *
 * Provides type-safe enum flags for boundary and initialization settings.
 * Includes bitwise operators for combining flags.
 */

/**
 * @enum BoundaryFlags
 * @brief Flags for solid and open boundary configuration
 */
enum class BoundaryFlags : uint32_t {
    NONE        = 0,
    FLOOR       = 1 << 0,   ///< Apply solid floor at z=0
    CEILING     = 1 << 1,   ///< Apply solid ceiling at z=Nz-1
    WALLS       = 1 << 2,   ///< Apply solid walls on x/y boundaries
    OPEN_X      = 1 << 3,   ///< Both X faces are open
    OPEN_Y      = 1 << 4,   ///< Both Y faces are open
    OPEN_X_MIN  = 1 << 5,   ///< X min face is open
    OPEN_X_MAX  = 1 << 6,   ///< X max face is open
    OPEN_Y_MIN  = 1 << 7,   ///< Y min face is open
    OPEN_Y_MAX  = 1 << 8,   ///< Y max face is open
    OPEN_Z_MIN  = 1 << 9,   ///< Z min face is open
    OPEN_Z_MAX  = 1 << 10,  ///< Z max face is open
};

/**
 * @enum InitFlags
 * @brief Flags for velocity and force initialization
 */
enum class InitFlags : uint32_t {
    NONE        = 0,
    VELOCITY_X  = 1 << 0,   ///< Initialize X velocity
    VELOCITY_Y  = 1 << 1,   ///< Initialize Y velocity
    VELOCITY_Z  = 1 << 2,   ///< Initialize Z velocity
    LBM_UNITS_X = 1 << 3,   ///< X velocity is in LBM units
    LBM_UNITS_Y = 1 << 4,   ///< Y velocity is in LBM units
    LBM_UNITS_Z = 1 << 5,   ///< Z velocity is in LBM units
    INLET       = 1 << 6,   ///< Has inlet velocity
    GRAVITY     = 1 << 7,   ///< Has gravity force
    HYDROSTATIC = 1 << 8,   ///< Has hydrostatic pressure
    WIND        = 1 << 9,   ///< Has wind profile
};

/**
 * @enum Face
 * @brief Identifies faces of the simulation domain
 */
enum class Face {
    X_MIN,  ///< Left face (x = 0)
    X_MAX,  ///< Right face (x = Nx-1)
    Y_MIN,  ///< Front face (y = 0)
    Y_MAX,  ///< Back face (y = Ny-1)
    Z_MIN,  ///< Bottom face (z = 0)
    Z_MAX   ///< Top face (z = Nz-1)
};

/**
 * @enum Axis
 * @brief Coordinate axis for directional operations
 */
enum class Axis {
    X,  ///< X axis
    Y,  ///< Y axis
    Z   ///< Z axis
};

/**
 * @enum UnitSystem
 * @brief Unit system for velocity specification
 */
enum class UnitSystem {
    SI,   ///< SI units (m/s)
    LBM   ///< LBM units (lattice units)
};

// ============================================================================
// Bitwise operators for BoundaryFlags
// ============================================================================

inline BoundaryFlags operator|(BoundaryFlags a, BoundaryFlags b) {
    return static_cast<BoundaryFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline BoundaryFlags operator&(BoundaryFlags a, BoundaryFlags b) {
    return static_cast<BoundaryFlags>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline BoundaryFlags& operator|=(BoundaryFlags& a, BoundaryFlags b) {
    a = a | b;
    return a;
}

inline BoundaryFlags& operator&=(BoundaryFlags& a, BoundaryFlags b) {
    a = a & b;
    return a;
}

inline bool has_flag(BoundaryFlags flags, BoundaryFlags flag) {
    return (static_cast<uint32_t>(flags) & static_cast<uint32_t>(flag)) != 0;
}

// ============================================================================
// Bitwise operators for InitFlags
// ============================================================================

inline InitFlags operator|(InitFlags a, InitFlags b) {
    return static_cast<InitFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

inline InitFlags operator&(InitFlags a, InitFlags b) {
    return static_cast<InitFlags>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}

inline InitFlags& operator|=(InitFlags& a, InitFlags b) {
    a = a | b;
    return a;
}

inline InitFlags& operator&=(InitFlags& a, InitFlags b) {
    a = a & b;
    return a;
}

inline bool has_flag(InitFlags flags, InitFlags flag) {
    return (static_cast<uint32_t>(flags) & static_cast<uint32_t>(flag)) != 0;
}
