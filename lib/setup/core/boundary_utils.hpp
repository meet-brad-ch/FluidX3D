#pragma once

#include "setup/core/types.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/core/setup_error.hpp"
#include "utilities.hpp"

#include <string>

/// @file boundary_utils.hpp
/// @brief The cell test that the boundary builders share (internal to the Setup API): FaceLayer.

/// @brief A layer of cells parallel to one face of the box: the face's own layer, or a layer a number of cells inward
/// from it. The boundary builders use it to find the cells of a face.
/// @code
/// const FaceLayer lid(Face::Z_MAX, cells);
/// const bool on_lid = lid.contains(x, y, z);
/// @endcode
class FaceLayer {
public:
    /// @brief Makes the layer of a face of a box.
    /// @param face the face of the box (X_MIN is the layer x = 0, X_MAX the layer x = Nx - 1)
    /// @param cells the box's cells along x, y and z
    /// @param offset the number of cells inward from the face, less than the cells along that axis (0: the face's own
    /// layer)
    /// @throws SetupError when the offset is not less than the cells along the face's axis (the layer would be
    /// outside the box)
    FaceLayer(Face face, const uint3& cells, uint32_t offset = 0u) : face_(face), layer_(layer_index(face, cells, offset)) {}

    /// @brief Tells whether a cell lies in the layer.
    /// @param x the cell along x, counted from 0
    /// @param y the cell along y, counted from 0
    /// @param z the cell along z, counted from 0
    /// @return true when the cell is in the layer
    bool contains(uint32_t x, uint32_t y, uint32_t z) const {
        bool inside{ false };
        switch(face_) {
            case Face::X_MIN:
            case Face::X_MAX:
                inside = x == layer_;
                break;
            case Face::Y_MIN:
            case Face::Y_MAX:
                inside = y == layer_;
                break;
            case Face::Z_MIN:
            case Face::Z_MAX:
                inside = z == layer_;
                break;
        }
        return inside;
    }

private:
    /// @brief Gives the coordinate of a face's layer along the face's axis.
    /// @param face the face of the box
    /// @param cells the box's cells along x, y and z
    /// @param offset the number of cells inward from the face
    /// @return offset for a MIN face, the last cell minus offset for a MAX face
    /// @throws SetupError when the offset is not less than the cells along the face's axis
    static uint32_t layer_index(Face face, const uint3& cells, uint32_t offset) {
        const uint32_t along{ cells_along(face, cells) };
        if(offset >= along) {
            const std::string offset_text = std::to_string(offset);
            const std::string along_text = std::to_string(along);
            throw SetupError("FaceLayer: the offset of " + offset_text + " cells from the face is not less than the " + along_text +
                             " cells along the face's axis");
        }
        const bool max_face{ face == Face::X_MAX || face == Face::Y_MAX || face == Face::Z_MAX };
        uint32_t index{ offset };
        if(max_face) {
            index = along - 1u - offset;
        }
        return index;
    }

    /// @brief Gives the box's cells along a face's axis (the axis across the face).
    /// @param face the face of the box
    /// @param cells the box's cells along x, y and z
    /// @return cells.x for an X face, cells.y for a Y face, cells.z for a Z face
    static uint32_t cells_along(Face face, const uint3& cells) {
        uint32_t along{ cells.z };
        switch(face) {
            case Face::X_MIN:
            case Face::X_MAX:
                along = cells.x;
                break;
            case Face::Y_MIN:
            case Face::Y_MAX:
                along = cells.y;
                break;
            case Face::Z_MIN:
            case Face::Z_MAX:
                along = cells.z;
                break;
        }
        return along;
    }

    Face face_{ Face::X_MIN }; ///< the face that the layer is parallel to
    uint32_t layer_{};         ///< the layer's coordinate along the face's axis, counted from 0
};
