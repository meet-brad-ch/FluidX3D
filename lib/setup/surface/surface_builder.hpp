#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/domain/lattice.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <array>
#include <optional>
#include <vector>

extern Units units; // global units object from lbm.cpp

/// @brief Free surface setup (SURFACE extension) in physical units: water, solid walls and blocks, inflows and outflows.
///
/// Positions are in metres from the domain's origin corner and speeds in m/s, converted with the global units
/// (SimulationSetup::configure_units() first). Lengths become whole cells as CellSpan describes. apply() writes the
/// setup to the grid in this order: water, solid faces and blocks, inflows, outflows.
/// @code
/// SurfaceBuilder(lbm)
///     .set_water_level(0.576_m)
///     .initialize_hydrostatic()
///     .set_solid_faces({Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Z_MIN})
///     .add_solid_block({0_m, 0_m, 0_m}, {0.96_m, 0.53_m, 0.38_m})
///     .add_inflow(Face::Y_MIN, 1.36_mps, 0.38_m, 0.576_m)
///     .add_outflow(Face::Y_MAX, 0.68_mps)
///     .apply();
/// @endcode
class SurfaceBuilder {
public:
    /// @param lbm the free surface LBM, created with its gravity (SimulationSetup::create_lbm_surface())
    explicit SurfaceBuilder(LBM& lbm) : lbm_(lbm) {}

    /// Water in the whole domain below this height.
    SurfaceBuilder& set_water_level(Length height) {
        water_level_ = height;
        return *this;
    }

    /// Water in the box between two corners, in addition to the water level.
    SurfaceBuilder& add_water_box(Position from, Position to) {
        water_boxes_.push_back({ from, to });
        return *this;
    }

    /// Solid walls on all six faces.
    SurfaceBuilder& set_solid_walls() {
        solid_faces_.fill(true);
        return *this;
    }

    /// Solid walls on these faces.
    SurfaceBuilder& set_solid_faces(const std::vector<Face>& faces) {
        for(const Face face : faces) solid_faces_[static_cast<std::size_t>(face)] = true;
        return *this;
    }

    /// A solid box between two corners.
    SurfaceBuilder& add_solid_block(Position from, Position to) {
        solid_blocks_.push_back({ from, to });
        return *this;
    }

    /// @brief Water flowing in through an X or Y face at a speed, between two heights.
    ///
    /// The inflow is the cell layer one inward from the face, so the face itself can stay a wall; cells on solid
    /// faces are left out.
    SurfaceBuilder& add_inflow(Face face, Speed speed, Length from_height, Length to_height) {
        if(face == Face::Z_MIN || face == Face::Z_MAX) {
            print_warning("SurfaceBuilder: an inflow needs an X or Y face; this one is left out");
            return *this;
        }
        inflows_.push_back({ face, speed, from_height, to_height });
        return *this;
    }

    /// Outflow through a face at a speed: its cells become equilibrium boundaries (TYPE_E), except those on solid faces.
    SurfaceBuilder& add_outflow(Face face, Speed speed) {
        outflows_.push_back({ face, speed });
        return *this;
    }

    /// Initial density from the hydrostatic pressure of the LBM's gravity, below the water level (without one,
    /// below half the domain height).
    SurfaceBuilder& initialize_hydrostatic() {
        init_hydrostatic_ = true;
        return *this;
    }

    /// Writes the setup to the grid.
    void apply() {
        const uint32_t Nz = lbm_.get_Nz();
        const uint32_t water_height = water_level_ ? CellSpan::of(0.0f, units.x(water_level_->si()), Nz).end : 0u;
        const float32_t gravity = -lbm_.get_fz(); // the LBM's gravity along -z, in lattice units

        std::vector<CellBox> water, solid;
        for(const Box& box : water_boxes_) water.push_back(cells_of(box));
        uint32_t reference_height = water_height; // the surface of the hydrostatic pressure: the water level, ...
        for(const CellBox& box : water) reference_height = max(reference_height, box.z.end); // ... the highest water box
        if(reference_height == 0u) reference_height = Nz / 2u;                                     // ... or half the height
        for(const Box& box : solid_blocks_) solid.push_back(cells_of(box));
        std::vector<CellFlow> inflows, outflows;
        for(const Inflow& inflow : inflows_) {
            const float32_t velocity = units.u(inflow.speed.si());
            inflows.push_back({ inflow.face, is_min_face(inflow.face) ? velocity : -velocity, // into the domain
                                CellSpan::of(units.x(inflow.from_height.si()), units.x(inflow.to_height.si()), Nz), 1u });
        }
        for(const Outflow& outflow : outflows_) {
            const float32_t velocity = units.u(outflow.speed.si());
            outflows.push_back({ outflow.face, is_min_face(outflow.face) ? -velocity : velocity, // out of the domain
                                 CellSpan::of(0.0f, (float32_t)Nz, Nz), 0u });
        }

        parallel_for(lbm_.get_N(), [&](uint64_t n) {
            uint32_t x = 0u, y = 0u, z = 0u;
            lbm_.coordinates(n, x, y, z);

            if(z < water_height || contains(water, x, y, z)) {
                lbm_.flags[n] = TYPE_F;
                if(init_hydrostatic_) lbm_.rho[n] = units.rho_hydrostatic(gravity, (float32_t)z, (float32_t)reference_height);
            }
            if(on_solid_face(x, y, z) || contains(solid, x, y, z)) lbm_.flags[n] = TYPE_S;

            for(const CellFlow& inflow : inflows) {
                if(in_flow(inflow, x, y, z)) {
                    lbm_.flags[n] = TYPE_F;
                    set_normal_velocity(n, inflow);
                }
            }
            for(const CellFlow& outflow : outflows) {
                if(in_flow(outflow, x, y, z)) {
                    lbm_.flags[n] = TYPE_E;
                    set_normal_velocity(n, outflow);
                }
            }
        });
    }

private:
    struct Box { Position from, to; };                                ///< in metres
    struct Inflow { Face face; Speed speed; Length from_height, to_height; };
    struct Outflow { Face face; Speed speed; };

    struct CellBox { ///< a Box in whole cells
        CellSpan x, y, z;
        bool contains(uint32_t cx, uint32_t cy, uint32_t cz) const { return x.contains(cx) && y.contains(cy) && z.contains(cz); }
    };

    struct CellFlow { ///< an inflow or outflow in lattice units
        Face face;
        float32_t velocity; ///< along the face normal's axis, signed
        CellSpan height;    ///< cells along z
        uint32_t layer;     ///< cells inward from the face
    };

    LBM& lbm_;
    std::optional<Length> water_level_;
    std::vector<Box> water_boxes_;
    std::array<bool, 6> solid_faces_{}; ///< indexed by Face
    std::vector<Box> solid_blocks_;
    std::vector<Inflow> inflows_;
    std::vector<Outflow> outflows_;
    bool init_hydrostatic_ = false;

    static bool is_min_face(Face face) { return face == Face::X_MIN || face == Face::Y_MIN || face == Face::Z_MIN; }

    static bool contains(const std::vector<CellBox>& boxes, uint32_t x, uint32_t y, uint32_t z) {
        for(const CellBox& box : boxes) {
            if(box.contains(x, y, z)) return true;
        }
        return false;
    }

    CellBox cells_of(const Box& box) const {
        return { CellSpan::of(units.x(box.from.x.si()), units.x(box.to.x.si()), lbm_.get_Nx()),
                 CellSpan::of(units.x(box.from.y.si()), units.x(box.to.y.si()), lbm_.get_Ny()),
                 CellSpan::of(units.x(box.from.z.si()), units.x(box.to.z.si()), lbm_.get_Nz()) };
    }

    bool on_face(Face face, uint32_t x, uint32_t y, uint32_t z, uint32_t layer = 0u) const {
        return boundary_utils::is_on_face(x, y, z, lbm_.get_Nx(), lbm_.get_Ny(), lbm_.get_Nz(), face, layer);
    }

    bool on_solid_face(uint32_t x, uint32_t y, uint32_t z) const {
        for(std::size_t face = 0u; face < solid_faces_.size(); face++) {
            if(solid_faces_[face] && on_face(static_cast<Face>(face), x, y, z)) return true;
        }
        return false;
    }

    bool in_flow(const CellFlow& flow, uint32_t x, uint32_t y, uint32_t z) const {
        return on_face(flow.face, x, y, z, flow.layer) && flow.height.contains(z) && !on_solid_face(x, y, z);
    }

    void set_normal_velocity(uint64_t n, const CellFlow& flow) {
        switch(flow.face) {
            case Face::X_MIN: case Face::X_MAX: lbm_.u.x[n] = flow.velocity; break;
            case Face::Y_MIN: case Face::Y_MAX: lbm_.u.y[n] = flow.velocity; break;
            case Face::Z_MIN: case Face::Z_MAX: lbm_.u.z[n] = flow.velocity; break;
        }
    }
};
