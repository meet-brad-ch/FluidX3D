#pragma once

enum class Face { // domain faces
    X_MIN,  // x = 0
    X_MAX,  // x = Nx-1
    Y_MIN,  // y = 0
    Y_MAX,  // y = Ny-1
    Z_MIN,  // z = 0
    Z_MAX   // z = Nz-1
};

enum class Axis { X, Y, Z };
