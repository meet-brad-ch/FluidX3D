#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/unit_scale.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/domain/lattice.hpp"
#include "setup/domain/shape.hpp"
#include "lbm.hpp"
#include <array>
#include <initializer_list>
#include <optional>
#include <vector>

#ifndef SURFACE
#error "setup/surface/surface_builder.hpp needs SURFACE in defines.hpp"
#endif // SURFACE

/// @brief Free surface setup (SURFACE extension) in physical units: water, gas bubbles, solid walls and objects,
/// inflows, outflows and drains (Simulation::surface()).
///
/// Positions are in metres from the domain's origin corner and speeds in m/s, converted with the simulation's unit
/// scale. The water level and the inflows' heights become whole cells as CellSpan describes; shapes cover the cells
/// whose centers they contain, smoothly (PLIC) at a sphere's surface. apply() writes the setup to the grid in this
/// order: water, gas, solid faces and objects, inflows, outflows, drains.
/// @code
/// sim.surface()
///     .set_water_level(0.576_m)
///     .initialize_hydrostatic()
///     .set_solid_faces({Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Z_MIN})
///     .add_solid(Shape::box({0_m, 0_m, 0_m}, {0.96_m, 0.53_m, 0.38_m}))
///     .add_inflow(Face::Y_MIN, 1.36_mps, 0.38_m, 0.576_m)
///     .add_outflow(Face::Y_MAX, 0.68_mps)
///     .apply();
/// @endcode
class SurfaceBuilder {
public:
    /// @param lbm   the free surface LBM, created with its gravity
    /// @param unit_scale the simulation's unit scale
    SurfaceBuilder(LBM& lbm, const UnitScale& unit_scale) : lbm_(lbm), units_(unit_scale) {}

    /// Water in the whole domain below this height, flowing at this velocity.
    SurfaceBuilder& set_water_level(Length height, Velocity velocity = {}) {
        water_level_ = WaterLevel{ height, velocity };
        return *this;
    }

    /// Water in a shape (a column, a drop), moving at this velocity, in addition to the water level.
    SurfaceBuilder& add_water(const Shape& shape, Velocity velocity = {}) {
        water_.push_back({ shape, velocity });
        return *this;
    }

    /// Gas in a shape, such as a bubble in the water.
    SurfaceBuilder& add_gas(const Shape& shape) {
        gas_.push_back(shape);
        return *this;
    }

    /// Solid walls on all six faces: a closed box.
    SurfaceBuilder& set_solid_box() {
        solid_faces_.fill(true);
        return *this;
    }

    /// Solid walls on these faces.
    SurfaceBuilder& set_solid_faces(std::initializer_list<Face> faces) {
        for(const Face face : faces) solid_faces_[static_cast<std::size_t>(face)] = true;
        return *this;
    }

    /// A solid object.
    SurfaceBuilder& add_solid(const Shape& shape) {
        solids_.push_back(shape);
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

    /// Fluid reaching these cells leaves the domain: equilibrium boundaries (TYPE_E) at half the density, e.g. the
    /// sides above the water, where splashes would otherwise pile up.
    SurfaceBuilder& add_drain(const Shape& region) {
        drains_.push_back(region);
        return *this;
    }

    /// Initial density from the hydrostatic pressure of the simulation's gravity below the water level (set_water_level()).
    SurfaceBuilder& initialize_hydrostatic() {
        init_hydrostatic_ = true;
        return *this;
    }

    /// Writes the setup to the grid.
    void apply() {
        const uint3 N(lbm_.get_Nx(), lbm_.get_Ny(), lbm_.get_Nz());
        const float32_t cell_size = units_.cell_size().si();
        const uint32_t water_height = water_level_ ? CellSpan::of(0.0f, units_.length(water_level_->height), N.z).end : 0u;
        const float3 level_velocity = water_level_ ? lattice(water_level_->velocity) : float3(0.0f);
        const float32_t gravity = -lbm_.get_fz(); // the LBM's gravity along -z, in lattice units
        if(init_hydrostatic_ && !water_level_) print_warning("SurfaceBuilder: initialize_hydrostatic() needs a water level; left out");

        std::vector<Shape::Cells> water, gas, solids, drains;
        std::vector<float3> water_velocities;
        for(const Water& w : water_) {
            water.push_back(w.shape.in_cells(cell_size, N));
            water_velocities.push_back(lattice(w.velocity));
        }
        for(const Shape& shape : gas_) gas.push_back(shape.in_cells(cell_size, N));
        for(const Shape& shape : solids_) solids.push_back(shape.in_cells(cell_size, N));
        for(const Shape& shape : drains_) drains.push_back(shape.in_cells(cell_size, N));
        std::vector<CellFlow> inflows, outflows;
        for(const Inflow& inflow : inflows_) {
            const float32_t velocity = units_.velocity(inflow.speed);
            inflows.push_back({ inflow.face, is_min_face(inflow.face) ? velocity : -velocity, // into the domain
                                CellSpan::of(units_.length(inflow.from_height), units_.length(inflow.to_height), N.z), 1u });
        }
        for(const Outflow& outflow : outflows_) {
            const float32_t velocity = units_.velocity(outflow.speed);
            outflows.push_back({ outflow.face, is_min_face(outflow.face) ? -velocity : velocity, // out of the domain
                                 CellSpan::of(0.0f, (float32_t)N.z, N.z), 0u });
        }

        parallel_for(lbm_.get_N(), [&](uint64_t n) {
            uint32_t x = 0u, y = 0u, z = 0u;
            lbm_.coordinates(n, x, y, z);

            if(z < water_height) {
                lbm_.flags[n] = TYPE_F;
                set_velocity(n, level_velocity);
                if(init_hydrostatic_) lbm_.rho[n] = hydrostatic_density(gravity, (float32_t)z, (float32_t)water_height);
            }
            for(std::size_t i = 0u; i < water.size(); i++) { // the core fills in phi of fluid and gas cells
                const float32_t share = water[i].fill(x, y, z);
                if(share <= 0.0f) continue;
                lbm_.flags[n] = share < 1.0f ? TYPE_I : TYPE_F;
                if(share < 1.0f) lbm_.phi[n] = share;
                set_velocity(n, water_velocities[i]);
            }
            for(const Shape::Cells& bubble : gas) {
                const float32_t share = bubble.fill(x, y, z);
                if(share <= 0.0f) continue;
                lbm_.flags[n] = share < 1.0f ? TYPE_I : TYPE_G;
                if(share < 1.0f) lbm_.phi[n] = 1.0f - share;
            }
            if(on_solid_face(x, y, z) || contains(solids, x, y, z)) { // at rest
                lbm_.flags[n] = TYPE_S;
                set_velocity(n, float3(0.0f));
            }

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
            if(contains(drains, x, y, z)) {
                lbm_.flags[n] = TYPE_E;
                lbm_.rho[n] = 0.5f;
            }
        });
    }

private:
    struct WaterLevel { Length height; Velocity velocity; };
    struct Water { Shape shape; Velocity velocity; };
    struct Inflow { Face face; Speed speed; Length from_height, to_height; };
    struct Outflow { Face face; Speed speed; };

    struct CellFlow { ///< an inflow or outflow in lattice units
        Face face;
        float32_t velocity; ///< along the face normal's axis, signed
        CellSpan height;    ///< cells along z
        uint32_t layer;     ///< cells inward from the face
    };

    LBM& lbm_;
    UnitScale units_;
    std::optional<WaterLevel> water_level_;
    std::vector<Water> water_;
    std::vector<Shape> gas_;
    std::array<bool, 6> solid_faces_{}; ///< indexed by Face
    std::vector<Shape> solids_;
    std::vector<Inflow> inflows_;
    std::vector<Outflow> outflows_;
    std::vector<Shape> drains_;
    bool init_hydrostatic_ = false;

    static bool is_min_face(Face face) { return face == Face::X_MIN || face == Face::Y_MIN || face == Face::Z_MIN; }

    float3 lattice(const Velocity& u) const { return float3(units_.velocity(u.x), units_.velocity(u.y), units_.velocity(u.z)); }

    static bool contains(const std::vector<Shape::Cells>& shapes, uint32_t x, uint32_t y, uint32_t z) {
        for(const Shape::Cells& shape : shapes) {
            if(shape.contains(x, y, z)) return true;
        }
        return false;
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

    void set_velocity(uint64_t n, const float3& u) {
        lbm_.u.x[n] = u.x;
        lbm_.u.y[n] = u.y;
        lbm_.u.z[n] = u.z;
    }

    void set_normal_velocity(uint64_t n, const CellFlow& flow) {
        switch(flow.face) {
            case Face::X_MIN: case Face::X_MAX: lbm_.u.x[n] = flow.velocity; break;
            case Face::Y_MIN: case Face::Y_MAX: lbm_.u.y[n] = flow.velocity; break;
            case Face::Z_MIN: case Face::Z_MAX: lbm_.u.z[n] = flow.velocity; break;
        }
    }
};
