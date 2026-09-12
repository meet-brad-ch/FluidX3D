#pragma once

#include "setup/core/types.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <vector>
#include <functional>

extern Units units; // global units object from lbm.cpp

// Free surface setup (SURFACE extension): fluid regions, solid walls and blocks, inlets and outlets; apply() writes them to the grid.
// Positions are in cells, velocities and gravity in LBM units.
class SurfaceBuilder {
public:
    using CellPredicate = std::function<bool(uint32_t, uint32_t, uint32_t)>;

    explicit SurfaceBuilder(LBM& lbm) : lbm_(lbm) {}

    // fluid below this fraction of the domain height
    SurfaceBuilder& set_water_level(float32_t height_fraction) {
        water_level_fraction_ = height_fraction;
        use_water_level_ = true;
        return *this;
    }

    SurfaceBuilder& set_water_level_cells(uint32_t height_cells) {
        water_level_cells_ = height_cells;
        use_water_level_cells_ = true;
        return *this;
    }

    // fluid where predicate(x, y, z) is true, in addition to the water level
    SurfaceBuilder& set_fluid_region(CellPredicate predicate) {
        fluid_predicate_ = predicate;
        use_predicate_ = true;
        return *this;
    }

    // gravity in LBM units for the hydrostatic initialization (the LBM's own volume force is set when it is created)
    SurfaceBuilder& set_gravity_lbm(float32_t g) {
        gravity_lbm_ = g;
        return *this;
    }

    SurfaceBuilder& set_solid_walls() { // all six faces
        solid_walls_ = true;
        return *this;
    }

    SurfaceBuilder& set_solid_faces(const std::vector<Face>& faces) {
        for (Face f : faces) {
            solid_faces_.push_back(f);
        }
        return *this;
    }

    // solid box [min, max) in cells
    SurfaceBuilder& add_solid_block_cells(uint32_t min_x, uint32_t max_x,
                                           uint32_t min_y, uint32_t max_y,
                                           uint32_t min_z, uint32_t max_z) {
        solid_blocks_.push_back({min_x, max_x, min_y, max_y, min_z, max_z});
        return *this;
    }

    // fluid cells with a velocity (LBM units) along axis where predicate is true
    SurfaceBuilder& add_velocity_inlet(CellPredicate predicate, float32_t velocity_lbm, Axis axis) {
        velocity_inlets_.push_back({predicate, velocity_lbm, axis});
        return *this;
    }

    // equilibrium (TYPE_E) cells where predicate is true, with an optional velocity (LBM units) along axis
    SurfaceBuilder& add_equilibrium_outlet(CellPredicate predicate, float32_t velocity_lbm = 0.0f, Axis axis = Axis::Y) {
        equilibrium_outlets_.push_back({predicate, velocity_lbm, axis});
        return *this;
    }

    // initial density from the hydrostatic pressure below the water level (or below Nz/2)
    SurfaceBuilder& initialize_hydrostatic() {
        init_hydrostatic_ = true;
        return *this;
    }

    void apply() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        uint32_t water_height = 0;
        if (use_water_level_) {
            water_height = (uint32_t)(water_level_fraction_ * (float32_t)Nz);
        } else if (use_water_level_cells_) {
            water_height = water_level_cells_;
        }
        const uint32_t ref_height = water_height != 0 ? water_height : Nz / 2; // hydrostatic reference height

        parallel_for(lbm_.get_N(), [&](uint64_t n) {
            uint32_t x = 0, y = 0, z = 0;
            lbm_.coordinates(n, x, y, z);

            const bool below_water_level = (use_water_level_ || use_water_level_cells_) && z < water_height;
            const bool in_fluid_region = use_predicate_ && fluid_predicate_ && fluid_predicate_(x, y, z);
            if (below_water_level || in_fluid_region) {
                lbm_.flags[n] = TYPE_F;
                if (init_hydrostatic_) {
                    lbm_.rho[n] = units.rho_hydrostatic(gravity_lbm_, (float32_t)z, (float32_t)ref_height);
                }
            }

            if (solid_walls_ && (x == 0 || x == Nx - 1 || y == 0 || y == Ny - 1 || z == 0 || z == Nz - 1)) {
                lbm_.flags[n] = TYPE_S;
            }
            for (Face f : solid_faces_) {
                if (boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, f)) lbm_.flags[n] = TYPE_S;
            }
            for (const auto& block : solid_blocks_) {
                if (x >= block.min_x && x < block.max_x && y >= block.min_y && y < block.max_y && z >= block.min_z && z < block.max_z) {
                    lbm_.flags[n] = TYPE_S;
                }
            }

            for (const auto& inlet : velocity_inlets_) {
                if (inlet.predicate && inlet.predicate(x, y, z)) {
                    lbm_.flags[n] = TYPE_F;
                    set_velocity(n, inlet.axis, inlet.velocity);
                }
            }
            for (const auto& outlet : equilibrium_outlets_) {
                if (outlet.predicate && outlet.predicate(x, y, z)) {
                    lbm_.flags[n] = TYPE_E;
                    if (outlet.velocity != 0.0f) set_velocity(n, outlet.axis, outlet.velocity);
                }
            }
        });
    }

    // raytraced free surface on one GPU, rasterized on several
    void configure_visualization() {
        if (lbm_.get_D() == 1u) {
            lbm_.graphics.visualization_modes = VIS_PHI_RAYTRACE;
        } else {
            lbm_.graphics.visualization_modes = VIS_PHI_RASTERIZE;
        }
    }

private:
    LBM& lbm_;

    bool use_water_level_ = false;
    bool use_water_level_cells_ = false;
    float32_t water_level_fraction_ = 0.5f;
    uint32_t water_level_cells_ = 0;

    bool use_predicate_ = false;
    CellPredicate fluid_predicate_;

    float32_t gravity_lbm_ = 0.0002f;

    bool solid_walls_ = false;
    std::vector<Face> solid_faces_;

    struct SolidBlock {
        uint32_t min_x, max_x, min_y, max_y, min_z, max_z;
    };
    std::vector<SolidBlock> solid_blocks_;

    struct CellVelocity {
        CellPredicate predicate;
        float32_t velocity; // LBM units
        Axis axis;
    };
    std::vector<CellVelocity> velocity_inlets_;
    std::vector<CellVelocity> equilibrium_outlets_;

    bool init_hydrostatic_ = false;

    void set_velocity(uint64_t n, Axis axis, float32_t velocity) {
        switch (axis) {
            case Axis::X: lbm_.u.x[n] = velocity; break;
            case Axis::Y: lbm_.u.y[n] = velocity; break;
            case Axis::Z: lbm_.u.z[n] = velocity; break;
        }
    }
};
