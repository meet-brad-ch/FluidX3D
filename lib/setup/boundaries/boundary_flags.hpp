#pragma once

/// A face of the domain.
enum class Face {
    X_MIN,  ///< x = 0
    X_MAX,  ///< x = Nx-1
    Y_MIN,  ///< y = 0
    Y_MAX,  ///< y = Ny-1
    Z_MIN,  ///< z = 0
    Z_MAX   ///< z = Nz-1
};

/// A coordinate axis; Z is the height.
enum class Axis {
    X, ///< the X axis
    Y, ///< the Y axis
    Z  ///< the Z axis, the height
};

/// What a solid object is for: at rest, or at rest with the fluid's force on it measured (Simulation::forces(), the
/// FORCE_FIELD extension).
enum class Solid {
    FIXED,   ///< at rest
    MEASURED ///< at rest, with the fluid's force on it measured
};
