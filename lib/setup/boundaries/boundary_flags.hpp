#pragma once

/// A face of the box: the outermost layer of cells on one side.
enum class Face {
    X_MIN,  ///< the layer x = 0 (west)
    X_MAX,  ///< the layer x = Nx-1 (east)
    Y_MIN,  ///< the layer y = 0 (south)
    Y_MAX,  ///< the layer y = Ny-1 (north)
    Z_MIN,  ///< the layer z = 0 (the floor)
    Z_MAX   ///< the layer z = Nz-1 (the top)
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
