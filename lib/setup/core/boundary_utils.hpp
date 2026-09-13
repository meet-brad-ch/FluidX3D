#pragma once

#include "setup/core/types.hpp"
#include "setup/boundaries/boundary_flags.hpp"

/// Cell tests shared by the builders (internal).
namespace boundary_utils {

/// @brief Whether a cell lies on a face of the domain, or a layer inward from it.
/// @param x, y, z the cell's coordinates
/// @param Nx, Ny, Nz the grid's cells along each axis
/// @param face the face
/// @param offset how many cells inward from the face (0: the face itself)
/// @return true if the cell is on that layer
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

} // namespace boundary_utils
