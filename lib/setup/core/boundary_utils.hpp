#pragma once

#include "core/types.hpp"
#include "boundaries/boundary_flags.hpp"

namespace boundary_utils {

// cell (x, y, z) is on the given domain face, offset cells inward
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
