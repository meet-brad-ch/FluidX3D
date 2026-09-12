#pragma once

#include "core/types.hpp"
#include "lbm.hpp"

enum class FieldMode { // lbm.graphics.field_mode
    VELOCITY = 0,
    DENSITY = 1,
    THERMAL = 2 // TEMPERATURE extension
};

enum class SliceMode { // lbm.graphics.slice_mode
    NONE = 0,
    X = 1,   // YZ plane
    Y = 2,   // XZ plane
    Z = 3,   // XY plane
    XZ = 4,
    XYZ = 5,
    YZ = 6,
    XY = 7
};

// Visualization modes of the LBM graphics; apply() writes them and turns camera autorotation off.
// Starts with no modes; inherit_modes() starts from the LBM's current modes instead.
class GraphicsConfig {
public:
    explicit GraphicsConfig(LBM& lbm) : lbm_(lbm) {
#ifdef GRAPHICS
        slice_mode_ = lbm_.graphics.slice_mode;
#endif
    }

    GraphicsConfig& inherit_modes() {
#ifdef GRAPHICS
        visualization_modes_ = lbm_.graphics.visualization_modes;
        field_mode_ = lbm_.graphics.field_mode;
#endif
        return *this;
    }

    GraphicsConfig& show_surface() { // solid surfaces (marching cubes)
        visualization_modes_ |= VIS_FLAG_SURFACE;
        return *this;
    }

    GraphicsConfig& show_flags() { // cell flags
        visualization_modes_ |= VIS_FLAG_LATTICE;
        return *this;
    }

    GraphicsConfig& show_lattice() { // same as show_flags()
        visualization_modes_ |= VIS_FLAG_LATTICE;
        return *this;
    }

    GraphicsConfig& show_vortices() { // Q-criterion
        visualization_modes_ |= VIS_Q_CRITERION;
        return *this;
    }

    GraphicsConfig& show_density_field() {
        visualization_modes_ |= VIS_FIELD;
        field_mode_ = static_cast<int32_t>(FieldMode::DENSITY);
        return *this;
    }

    GraphicsConfig& show_streamlines() {
        visualization_modes_ |= VIS_STREAMLINES;
        return *this;
    }

    GraphicsConfig& show_free_surface() { // rasterized (SURFACE extension)
        visualization_modes_ |= VIS_PHI_RASTERIZE;
        return *this;
    }

    // slice planes at the LBM's current slice positions
    GraphicsConfig& set_slice_mode(SliceMode mode) {
        slice_mode_ = static_cast<int32_t>(mode);
        return *this;
    }

    void apply() {
#ifdef GRAPHICS
        lbm_.graphics.visualization_modes = visualization_modes_;
        lbm_.graphics.field_mode = field_mode_;
        lbm_.graphics.slice_mode = slice_mode_;
        camera.autorotation = false;
#endif
    }

private:
    LBM& lbm_;
    int32_t visualization_modes_ = 0;
    int32_t field_mode_ = 0;
    int32_t slice_mode_ = 0;
};
