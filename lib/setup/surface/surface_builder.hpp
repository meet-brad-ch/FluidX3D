#pragma once

#include "setup/core/types.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/domain/lattice.hpp"
#include "setup/domain/shape.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include "lbm.hpp"
#include <array>
#include <initializer_list>
#include <optional>
#include <vector>

#ifndef SURFACE
#error "setup/surface/surface_builder.hpp needs SURFACE in defines.hpp"
#endif // SURFACE

/// @file surface_builder.hpp
/// @brief The free surface setup in physical units (SurfaceBuilder).

/// @brief Sets the free surface setup (SURFACE extension) in physical units: water, gas bubbles, solid walls and
/// objects, inflows, outflows and drains (Simulation::surface()).
///
/// A temporary builder (L2) that keeps a reference to the LBM, which must outlive it. apply() writes the core's flags
/// (TYPE_F, TYPE_I, TYPE_G, TYPE_S, TYPE_E), the fill level phi, the velocities u and the densities rho on the host.
/// Positions are in metres from the domain's origin corner and speeds in m/s; they become lattice values through the
/// simulation's converter. The water level and the inflows' heights become whole cells as CellSpan describes; shapes
/// cover the cells whose centres they contain, smoothly (PLIC) at a sphere's surface. apply() writes the setup in this
/// order: water, gas, solid faces and objects, inflows, outflows, drains.
/// @code
/// using namespace si::symbols;
/// sim.surface()
///     .set_water_level(0.576 * m)
///     .initialize_hydrostatic()
///     .set_solid_faces({ Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Z_MIN })
///     .add_solid(Shape::box({ 0.0 * m, 0.0 * m, 0.0 * m }, { 0.96 * m, 0.53 * m, 0.38 * m }))
///     .add_inflow(Face::Y_MIN, 1.36 * m / s, 0.38 * m, 0.576 * m)
///     .add_outflow(Face::Y_MAX, 0.68 * m / s)
///     .apply();
/// @endcode
class SurfaceBuilder {
public:
    /// @brief Makes a builder for a free surface LBM.
    /// @param lbm the free surface LBM, created with its gravity
    /// @param converter the simulation's conversions
    SurfaceBuilder(LBM& lbm, const si::LatticeConverter& converter) : lbm_(lbm), converter_(converter) {}

    /// @brief Fills the whole domain with water below a height.
    /// @param height the water level above z = 0
    /// @param velocity the water's start velocity
    /// @return this builder
    SurfaceBuilder& set_water_level(si::Length height, si::Velocity velocity = {}) {
        water_level_ = WaterLevel{ .height = height, .velocity = velocity };
        return *this;
    }

    /// @brief Adds water in a shape (a column, a drop), in addition to the water level.
    /// @param shape the water's region
    /// @param velocity the water's start velocity
    /// @return this builder
    SurfaceBuilder& add_water(const Shape& shape, si::Velocity velocity = {}) {
        water_.push_back(Water{ .shape = shape, .velocity = velocity });
        return *this;
    }

    /// @brief Adds gas in a shape, such as a bubble in the water.
    /// @param shape the gas's region
    /// @return this builder
    SurfaceBuilder& add_gas(const Shape& shape) {
        gas_.push_back(shape);
        return *this;
    }

    /// @brief Makes all six faces solid walls: a closed box.
    /// @return this builder
    SurfaceBuilder& set_solid_box() {
        solid_faces_.fill(true);
        return *this;
    }

    /// @brief Makes the given faces solid walls.
    /// @param faces the faces to make solid
    /// @return this builder
    SurfaceBuilder& set_solid_faces(std::initializer_list<Face> faces) {
        for(const Face face : faces) {
            solid_faces_[static_cast<std::size_t>(face)] = true;
        }
        return *this;
    }

    /// @brief Adds a solid object at rest.
    /// @param shape the object's region
    /// @return this builder
    SurfaceBuilder& add_solid(const Shape& shape) {
        solids_.push_back(shape);
        return *this;
    }

    /// @brief Adds water flowing in through an X or Y face at a speed, between two heights.
    ///
    /// The inflow is the cell layer one inward from the face, so the face itself can stay a wall; cells on solid
    /// faces are left out. A Z face is not supported: it ends the program with print_error().
    /// @param face the face, X or Y
    /// @param speed the inflow speed, into the domain
    /// @param from_height the bottom of the inflow above z = 0
    /// @param to_height the top of the inflow above z = 0
    /// @return this builder
    SurfaceBuilder& add_inflow(Face face, si::Speed speed, si::Length from_height, si::Length to_height) {
        const bool z_face{ face == Face::Z_MIN || face == Face::Z_MAX };
        if(z_face) {
            print_error("SurfaceBuilder::add_inflow(): an inflow needs an X or Y face, not a Z face");
        }
        inflows_.push_back(Inflow{ .face = face, .speed = speed, .from_height = from_height, .to_height = to_height });
        return *this;
    }

    /// @brief Adds an outflow through a face at a speed: its cells become equilibrium boundaries (TYPE_E), except
    /// those on solid faces.
    /// @param face the face of the outflow
    /// @param speed the outflow speed, out of the domain
    /// @return this builder
    SurfaceBuilder& add_outflow(Face face, si::Speed speed) {
        outflows_.push_back(Outflow{ .face = face, .speed = speed });
        return *this;
    }

    /// @brief Adds a drain: fluid that reaches these cells leaves the domain. They become equilibrium boundaries
    /// (TYPE_E) at half the density, for example the sides above the water, where splashes would otherwise pile up.
    /// @param region the drain's cells
    /// @return this builder
    SurfaceBuilder& add_drain(const Shape& region) {
        drains_.push_back(region);
        return *this;
    }

    /// @brief Sets the start density from the hydrostatic pressure of the simulation's gravity below the water level
    /// (set_water_level()). Without a water level, apply() ends the program with print_error().
    /// @return this builder
    SurfaceBuilder& initialize_hydrostatic() {
        init_hydrostatic_ = true;
        return *this;
    }

    /// @brief Writes the setup to the grid.
    ///
    /// Ends the program with print_error() when initialize_hydrostatic() is set without set_water_level().
    void apply() {
        if(init_hydrostatic_ && !water_level_) {
            print_error("SurfaceBuilder::apply(): initialize_hydrostatic() needs a water level (set_water_level())");
        }
        const LatticeSetup setup{ lattice_setup() };
        parallel_for(lbm_.get_N(), [&](uint64_t n) { write_cell(n, setup); });
    }

private:
    /// The water level.
    struct WaterLevel {
        si::Length height{ au::ZERO }; ///< the water level above z = 0
        si::Velocity velocity{};       ///< the water's start velocity
    };

    /// Water in a shape.
    struct Water {
        Shape shape{};           ///< the water's region
        si::Velocity velocity{}; ///< the water's start velocity
    };

    /// An inflow.
    struct Inflow {
        Face face = Face::Y_MIN;             ///< the face of the inflow
        si::Speed speed{ au::ZERO };         ///< the speed into the domain
        si::Length from_height{ au::ZERO };  ///< the bottom of the inflow above z = 0
        si::Length to_height{ au::ZERO };    ///< the top of the inflow above z = 0
    };

    /// An outflow.
    struct Outflow {
        Face face = Face::Y_MAX;     ///< the face of the outflow
        si::Speed speed{ au::ZERO }; ///< the speed out of the domain
    };

    /// An inflow or outflow in lattice units.
    struct CellFlow {
        Face face = Face::Y_MIN; ///< the face of the flow
        float32_t velocity{};    ///< the velocity along the face normal's axis in cells per time step, signed
        CellSpan height{};       ///< the flow's cells along z
        uint32_t layer{};        ///< the cells inward from the face (1 for an inflow, 0 for an outflow)
    };

    /// The setup in lattice units, made once before the cell loop.
    struct LatticeSetup {
        uint3 cells{};                          ///< the grid's cells along X, Y and Z
        uint32_t water_height{};                ///< the cells below the water level along z (0 without a water level)
        float3 level_velocity{ 0.0f };          ///< the water level's velocity in cells per time step
        float32_t gravity{};                    ///< the LBM's gravity along -z in lattice units
        std::vector<Shape::Cells> water{};      ///< the water shapes on the grid
        std::vector<float3> water_velocities{}; ///< their velocities in cells per time step
        std::vector<Shape::Cells> gas{};        ///< the gas shapes on the grid
        std::vector<Shape::Cells> solids{};     ///< the solid objects on the grid
        std::vector<Shape::Cells> drains{};     ///< the drains on the grid
        std::vector<CellFlow> inflows{};        ///< the inflows in lattice units
        std::vector<CellFlow> outflows{};       ///< the outflows in lattice units
    };

    LBM& lbm_;                                ///< the free surface LBM (not owned)
    si::LatticeConverter converter_;          ///< the simulation's conversions
    std::optional<WaterLevel> water_level_{}; ///< set_water_level(), or none
    std::vector<Water> water_{};              ///< add_water()
    std::vector<Shape> gas_{};                ///< add_gas()
    static constexpr std::size_t face_count{ 6u }; ///< the faces of the box
    std::array<bool, face_count> solid_faces_{}; ///< true for the solid faces, indexed by Face
    std::vector<Shape> solids_{};             ///< add_solid()
    std::vector<Inflow> inflows_{};           ///< add_inflow()
    std::vector<Outflow> outflows_{};         ///< add_outflow()
    std::vector<Shape> drains_{};             ///< add_drain()
    bool init_hydrostatic_ = false;           ///< true after initialize_hydrostatic()

    /// @brief Converts the setup to lattice units.
    /// @return the setup in lattice units
    LatticeSetup lattice_setup() const {
        LatticeSetup setup{};
        setup.cells = uint3(lbm_.get_Nx(), lbm_.get_Ny(), lbm_.get_Nz());
        const uint3& n{ setup.cells };
        if(water_level_.has_value()) {
            const WaterLevel& level{ water_level_.value() };
            const float level_cells{ static_cast<float>(converter_.cells(level.height)) };
            setup.water_height = CellSpan::of(0.0f, level_cells, n.z).end;
            setup.level_velocity = lattice(level.velocity);
        }
        setup.gravity = -lbm_.get_fz();
        const si::Length cell{ converter_.cell() };
        for(const Water& w : water_) {
            setup.water.push_back(w.shape.in_cells(cell, n));
            setup.water_velocities.push_back(lattice(w.velocity));
        }
        for(const Shape& shape : gas_) {
            setup.gas.push_back(shape.in_cells(cell, n));
        }
        for(const Shape& shape : solids_) {
            setup.solids.push_back(shape.in_cells(cell, n));
        }
        for(const Shape& shape : drains_) {
            setup.drains.push_back(shape.in_cells(cell, n));
        }
        for(const Inflow& inflow : inflows_) {
            setup.inflows.push_back(inflow_cells(inflow, n.z));
        }
        for(const Outflow& outflow : outflows_) {
            setup.outflows.push_back(outflow_cells(outflow, n.z));
        }
        return setup;
    }

    /// @brief Converts an inflow to lattice units: its velocity points into the domain.
    /// @param inflow the face, speed and heights of the inflow
    /// @param cells_z the grid's cells along z
    /// @return the inflow in lattice units, one cell inward from its face
    CellFlow inflow_cells(const Inflow& inflow, uint32_t cells_z) const {
        const float32_t speed{ converter_.velocity(inflow.speed) };
        const float32_t velocity{ is_min_face(inflow.face) ? speed : -speed }; // into the domain
        const float from_cells{ static_cast<float>(converter_.cells(inflow.from_height)) };
        const float to_cells{ static_cast<float>(converter_.cells(inflow.to_height)) };
        const CellSpan height{ CellSpan::of(from_cells, to_cells, cells_z) };
        const CellFlow flow{ .face = inflow.face, .velocity = velocity, .height = height, .layer = 1u };
        return flow;
    }

    /// @brief Converts an outflow to lattice units: its velocity points out of the domain, over the whole height.
    /// @param outflow the face and speed of the outflow
    /// @param cells_z the grid's cells along z
    /// @return the outflow in lattice units, on its face
    CellFlow outflow_cells(const Outflow& outflow, uint32_t cells_z) const {
        const float32_t speed{ converter_.velocity(outflow.speed) };
        const float32_t velocity{ is_min_face(outflow.face) ? -speed : speed }; // out of the domain
        const float top{ static_cast<float32_t>(cells_z) };
        const CellSpan height{ CellSpan::of(0.0f, top, cells_z) };
        const CellFlow flow{ .face = outflow.face, .velocity = velocity, .height = height, .layer = 0u };
        return flow;
    }

    /// @brief Writes one cell: water, gas, solids, flows and drains, in that order.
    /// @param n the cell's index
    /// @param setup the setup in lattice units
    void write_cell(uint64_t n, const LatticeSetup& setup) {
        uint32_t x{ 0u };
        uint32_t y{ 0u };
        uint32_t z{ 0u };
        lbm_.coordinates(n, x, y, z);
        write_water(n, x, y, z, setup);
        write_gas(n, x, y, z, setup);
        if(on_solid_face(x, y, z) || contains(setup.solids, x, y, z)) { // at rest
            lbm_.flags[n] = TYPE_S;
            set_velocity(n, float3(0.0f));
        }
        write_flows(n, x, y, z, setup);
        if(contains(setup.drains, x, y, z)) {
            lbm_.flags[n] = TYPE_E;
            lbm_.rho[n] = 0.5f;
        }
    }

    /// @brief Writes a cell below the water level and in the water shapes (the core fills in phi of the fluid and gas
    /// cells).
    /// @param n the cell's index
    /// @param x the cell along X
    /// @param y the cell along Y
    /// @param z the cell along Z
    /// @param setup the setup in lattice units
    void write_water(uint64_t n, uint32_t x, uint32_t y, uint32_t z, const LatticeSetup& setup) {
        if(z < setup.water_height) {
            lbm_.flags[n] = TYPE_F;
            set_velocity(n, setup.level_velocity);
            if(init_hydrostatic_) {
                const float height{ static_cast<float32_t>(z) };
                const float surface{ static_cast<float32_t>(setup.water_height) };
                lbm_.rho[n] = HydrostaticDensity::at(setup.gravity, height, surface);
            }
        }
        for(std::size_t i = 0u; i < setup.water.size(); i++) {
            const float32_t share{ setup.water[i].fill(x, y, z) };
            if(share <= 0.0f) {
                continue;
            }
            const bool interface{ share < 1.0f };
            lbm_.flags[n] = interface ? TYPE_I : TYPE_F;
            if(interface) {
                lbm_.phi[n] = share;
            }
            set_velocity(n, setup.water_velocities[i]);
        }
    }

    /// @brief Writes a cell in the gas shapes.
    /// @param n the cell's index
    /// @param x the cell along X
    /// @param y the cell along Y
    /// @param z the cell along Z
    /// @param setup the setup in lattice units
    void write_gas(uint64_t n, uint32_t x, uint32_t y, uint32_t z, const LatticeSetup& setup) {
        for(const Shape::Cells& bubble : setup.gas) {
            const float32_t share{ bubble.fill(x, y, z) };
            if(share <= 0.0f) {
                continue;
            }
            const bool interface{ share < 1.0f };
            lbm_.flags[n] = interface ? TYPE_I : TYPE_G;
            if(interface) {
                lbm_.phi[n] = 1.0f - share;
            }
        }
    }

    /// @brief Writes a cell of an inflow (fluid at the inflow velocity) or of an outflow (an equilibrium boundary).
    /// @param n the cell's index
    /// @param x the cell along X
    /// @param y the cell along Y
    /// @param z the cell along Z
    /// @param setup the setup in lattice units
    void write_flows(uint64_t n, uint32_t x, uint32_t y, uint32_t z, const LatticeSetup& setup) {
        for(const CellFlow& inflow : setup.inflows) {
            if(in_flow(inflow, x, y, z)) {
                lbm_.flags[n] = TYPE_F;
                set_normal_velocity(n, inflow);
            }
        }
        for(const CellFlow& outflow : setup.outflows) {
            if(in_flow(outflow, x, y, z)) {
                lbm_.flags[n] = TYPE_E;
                set_normal_velocity(n, outflow);
            }
        }
    }

    /// @brief Tells whether a face is at the minimum of its axis.
    /// @param face a face of the box
    /// @return true for X_MIN, Y_MIN and Z_MIN
    static bool is_min_face(Face face) { return face == Face::X_MIN || face == Face::Y_MIN || face == Face::Z_MIN; }

    /// @brief Converts a velocity to lattice units.
    /// @param u a velocity
    /// @return the velocity in cells per time step
    float3 lattice(const si::Velocity& u) const {
        const float x{ converter_.velocity(u.x) };
        const float y{ converter_.velocity(u.y) };
        const float z{ converter_.velocity(u.z) };
        const float3 velocity{ x, y, z };
        return velocity;
    }

    /// @brief Tells whether any of the shapes contains a cell.
    /// @param shapes the shapes on the grid
    /// @param x the cell along X
    /// @param y the cell along Y
    /// @param z the cell along Z
    /// @return true when a shape contains the cell's centre
    static bool contains(const std::vector<Shape::Cells>& shapes, uint32_t x, uint32_t y, uint32_t z) {
        for(const Shape::Cells& shape : shapes) {
            if(shape.contains(x, y, z)) {
                return true;
            }
        }
        return false;
    }

    /// @brief Tells whether a cell is on a layer of a face.
    /// @param face the face of the box
    /// @param x the cell along X
    /// @param y the cell along Y
    /// @param z the cell along Z
    /// @param layer the cells inward from the face
    /// @return true when the cell is on that layer of the face
    bool on_face(Face face, uint32_t x, uint32_t y, uint32_t z, uint32_t layer = 0u) const {
        const uint3 cells(lbm_.get_Nx(), lbm_.get_Ny(), lbm_.get_Nz());
        const FaceLayer face_layer(face, cells, layer);
        const bool on{ face_layer.contains(x, y, z) };
        return on;
    }

    /// @brief Tells whether a cell is on a solid face.
    /// @param x the cell along X
    /// @param y the cell along Y
    /// @param z the cell along Z
    /// @return true when the cell is on one of the solid faces
    bool on_solid_face(uint32_t x, uint32_t y, uint32_t z) const {
        for(std::size_t face = 0u; face < solid_faces_.size(); face++) {
            if(solid_faces_[face] && on_face(static_cast<Face>(face), x, y, z)) {
                return true;
            }
        }
        return false;
    }

    /// @brief Tells whether a cell belongs to a flow: on its layer, within its heights, not on a solid face.
    /// @param flow an inflow or outflow in lattice units
    /// @param x the cell along X
    /// @param y the cell along Y
    /// @param z the cell along Z
    /// @return true when the cell belongs to the flow
    bool in_flow(const CellFlow& flow, uint32_t x, uint32_t y, uint32_t z) const {
        const bool on_layer{ on_face(flow.face, x, y, z, flow.layer) };
        const bool within{ flow.height.contains(z) };
        const bool belongs{ on_layer && within && !on_solid_face(x, y, z) };
        return belongs;
    }

    /// @brief Sets a cell's lattice velocity.
    /// @param n the cell's index
    /// @param u the velocity in cells per time step
    void set_velocity(uint64_t n, const float3& u) {
        lbm_.u.x[n] = u.x;
        lbm_.u.y[n] = u.y;
        lbm_.u.z[n] = u.z;
    }

    /// @brief Sets a cell's velocity component along a flow's face normal.
    /// @param n the cell's index
    /// @param flow the flow in lattice units
    void set_normal_velocity(uint64_t n, const CellFlow& flow) {
        switch(flow.face) {
            case Face::X_MIN: case Face::X_MAX: lbm_.u.x[n] = flow.velocity; break;
            case Face::Y_MIN: case Face::Y_MAX: lbm_.u.y[n] = flow.velocity; break;
            case Face::Z_MIN: case Face::Z_MAX: lbm_.u.z[n] = flow.velocity; break;
        }
    }
};
