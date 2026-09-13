#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/unit_scale.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/domain/shape.hpp"
#include "lbm.hpp"
#include <array>
#include <functional>
#include <initializer_list>
#include <optional>
#include <vector>

/// @brief Domain boundaries, solid objects and the initial flow in physical units; apply() writes them to the grid
/// (Simulation::boundaries()).
///
/// Speeds, pressures and positions (metres from the domain's origin corner) are converted with the simulation's unit
/// scale. Faces are periodic unless set solid or open. Order per cell: solid faces, open faces (TYPE_E, which win at
/// the edges they share with solid faces), solid shapes, then the velocities and pressures of the non-solid cells and
/// the velocities of moving solids.
/// @code
/// sim.boundaries().set_solid_floor().set_open_boundaries().initialize_velocity_y(10.0_mps).apply();
/// sim.boundaries().add_solid(Shape::sphere(center, 1_cm), Solid::MEASURED).set_open_boundaries().initialize_velocity_x(1_mps).apply();
/// @endcode
class BoundaryBuilder {
public:
    using VelocityField = std::function<Velocity(Position)>; ///< a velocity at a point in metres from the origin corner
    using PressureField = std::function<Pressure(Position)>; ///< a pressure relative to the fluid's at rest
    using AccelerationField = std::function<AccelerationVector(Position)>; ///< a body force per mass at a point

    /// @param lbm   the LBM whose flags, velocities and densities are set
    /// @param unit_scale the simulation's unit scale
    BoundaryBuilder(LBM& lbm, const UnitScale& unit_scale) : lbm_(lbm), units_(unit_scale) {}

    /// Solid floor (z = 0).
    BoundaryBuilder& set_solid_floor() { return set_solid_faces({ Face::Z_MIN }); }

    /// Solid ceiling (z = Nz-1).
    BoundaryBuilder& set_solid_ceiling() { return set_solid_faces({ Face::Z_MAX }); }

    /// Solid walls on the four X and Y faces.
    BoundaryBuilder& set_solid_sides() { return set_solid_faces({ Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Y_MAX }); }

    /// Solid walls on all six faces: a closed box.
    BoundaryBuilder& set_solid_box() { return set_solid_sides().set_solid_floor().set_solid_ceiling(); }

    /// Solid walls on these faces.
    BoundaryBuilder& set_solid_faces(std::initializer_list<Face> faces) {
        for(const Face face : faces) solid_faces_[index(face)] = true;
        return *this;
    }

    /// Every face not set solid or periodic open (TYPE_E): alone all six faces (free flow), with set_solid_floor() a
    /// wind tunnel.
    BoundaryBuilder& set_open_boundaries() {
        open_ = true;
        return *this;
    }

    /// The two faces normal to this axis stay periodic, also with set_open_boundaries() (e.g. Z of a 2D domain).
    BoundaryBuilder& set_periodic(Axis axis) {
        periodic_[static_cast<std::size_t>(axis)] = true;
        return *this;
    }

    /// All walls solid; moving_face slides at this speed along +y (X and Z faces) or +x (Y faces).
    BoundaryBuilder& preset_lid_driven_cavity(Face moving_face, Speed speed) {
        set_solid_box();
        lid_ = Lid{ moving_face, speed };
        return *this;
    }

    /// A solid object at rest; Solid::MEASURED: the fluid's force on it is measured (Simulation::forces()).
    BoundaryBuilder& add_solid(const Shape& shape, Solid kind = Solid::FIXED) {
        solids_.push_back({ shape, kind == Solid::MEASURED ? (uchar)(TYPE_S | TYPE_X) : (uchar)TYPE_S, {} });
        return *this;
    }

    /// A solid object whose surface moves at this velocity, such as a turning cylinder (MOVING_BOUNDARIES).
    BoundaryBuilder& add_moving_solid(const Shape& shape, VelocityField wall_velocity) {
        solids_.push_back({ shape, TYPE_S, std::move(wall_velocity) });
        return *this;
    }

    /// Initial velocity along x in all non-solid cells.
    BoundaryBuilder& initialize_velocity_x(Speed u) {
        init_u_x_ = u;
        return *this;
    }

    /// Initial velocity along y in all non-solid cells.
    BoundaryBuilder& initialize_velocity_y(Speed u) {
        init_u_y_ = u;
        return *this;
    }

    /// Initial velocity along z in all non-solid cells.
    BoundaryBuilder& initialize_velocity_z(Speed u) {
        init_u_z_ = u;
        return *this;
    }

    /// Initial velocity in all non-solid cells.
    BoundaryBuilder& initialize_velocity(Velocity u) {
        init_u_x_ = u.x;
        init_u_y_ = u.y;
        init_u_z_ = u.z;
        return *this;
    }

    /// Initial velocity field in the non-solid cells, at each cell's center (after the uniform velocities).
    BoundaryBuilder& initialize_velocity(VelocityField velocity) {
        velocity_field_ = std::move(velocity);
        return *this;
    }

    /// Initial pressure field in the non-solid cells, relative to the fluid's at rest.
    BoundaryBuilder& initialize_pressure(PressureField pressure) {
        pressure_field_ = std::move(pressure);
        return *this;
    }

    /// A body force per mass in every cell, in addition to the simulation's uniform one (FORCE_FIELD), e.g. a pull toward a point.
    BoundaryBuilder& set_force_field(AccelerationField force) {
#ifndef FORCE_FIELD
        print_error("BoundaryBuilder::set_force_field() needs FORCE_FIELD in the EXTENSIONS");
#endif // FORCE_FIELD
        force_field_ = std::move(force);
        return *this;
    }

    /// @brief Atmospheric boundary layer u(z) = u_ref*(z/z_ref)^alpha in the non-solid cells above the floor.
    /// @param alpha power-law exponent: 0.10 sea, 0.143 open terrain, 0.20 suburbs, 0.25-0.40 urban
    BoundaryBuilder& set_wind_profile_power_law(Speed reference_speed, Length reference_height, float32_t alpha = 0.143f) {
        wind_ = WindProfile{ reference_speed, reference_height, alpha };
        return *this;
    }

    /// The face the wind comes from (default Y_MIN).
    BoundaryBuilder& set_wind_direction(Face direction) {
        wind_direction_ = direction;
        return *this;
    }

    /// Writes the boundaries, objects and initial flow to the grid.
    void apply() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();
        const float32_t cell_size = units_.cell_size().si();

        std::vector<Shape::Cells> solid_cells;
        for(const SolidShape& solid : solids_) solid_cells.push_back(solid.shape.in_cells(cell_size, uint3(Nx, Ny, Nz)));

        const float32_t lbm_init_u_x = init_u_x_ ? units_.velocity(*init_u_x_) : 0.0f;
        const float32_t lbm_init_u_y = init_u_y_ ? units_.velocity(*init_u_y_) : 0.0f;
        const float32_t lbm_init_u_z = init_u_z_ ? units_.velocity(*init_u_z_) : 0.0f;
        const float32_t lbm_lid_velocity = lid_ ? units_.velocity(lid_->speed) : 0.0f;
        const float32_t lbm_wind_ref_velocity = wind_ ? units_.velocity(wind_->reference_speed) : 0.0f;
        const float32_t lbm_wind_ref_height = wind_ ? units_.length(wind_->reference_height) : 1.0f;

        parallel_for(lbm_.get_N(), [&](uint64_t n) {
            uint32_t x = 0u, y = 0u, z = 0u;
            lbm_.coordinates(n, x, y, z);
            const Position center = { Length::from_si(((float32_t)x + 0.5f) * cell_size),
                                      Length::from_si(((float32_t)y + 0.5f) * cell_size),
                                      Length::from_si(((float32_t)z + 0.5f) * cell_size) };

            bool on_open_face = false;
            for(const Face face : all_faces) {
                if(!boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, face)) continue;
                if(solid_faces_[index(face)]) lbm_.flags[n] = TYPE_S;
                else if(open_ && !periodic_[axis_index(face)]) on_open_face = true;
            }
            if(on_open_face) lbm_.flags[n] = TYPE_E;

            for(std::size_t i = 0u; i < solids_.size(); i++) {
                if(!solid_cells[i].contains(x, y, z)) continue;
                lbm_.flags[n] = solids_[i].flag;
                if(solids_[i].wall_velocity) set_velocity(n, solids_[i].wall_velocity(center));
            }

            if(!(lbm_.flags[n] & TYPE_S)) {
                if(init_u_x_) lbm_.u.x[n] = lbm_init_u_x;
                if(init_u_y_) lbm_.u.y[n] = lbm_init_u_y;
                if(init_u_z_) lbm_.u.z[n] = lbm_init_u_z;
                if(velocity_field_) set_velocity(n, velocity_field_(center));
                if(pressure_field_) lbm_.rho[n] = 1.0f + 3.0f * units_.pressure(pressure_field_(center)); // p = c²*rho, c² = 1/3
            }

#ifdef FORCE_FIELD
            if(force_field_) { // per volume at the lattice density 1: the acceleration in lattice units
                const AccelerationVector a = force_field_(center);
                lbm_.F.x[n] = units_.acceleration(a.x);
                lbm_.F.y[n] = units_.acceleration(a.y);
                lbm_.F.z[n] = units_.acceleration(a.z);
            }
#endif // FORCE_FIELD

            if(wind_ && !(lbm_.flags[n] & TYPE_S)) {
                const float32_t height_ratio = ((float32_t)z + 0.5f) / lbm_wind_ref_height; // at the cell's center
                const float32_t wind_velocity = lbm_wind_ref_velocity * pow(height_ratio, wind_->alpha);
                switch(wind_direction_) {
                    case Face::X_MIN: lbm_.u.x[n] = wind_velocity; break;
                    case Face::X_MAX: lbm_.u.x[n] = -wind_velocity; break;
                    case Face::Y_MIN: lbm_.u.y[n] = wind_velocity; break;
                    case Face::Y_MAX: lbm_.u.y[n] = -wind_velocity; break;
                    case Face::Z_MIN: lbm_.u.z[n] = wind_velocity; break;
                    case Face::Z_MAX: lbm_.u.z[n] = -wind_velocity; break;
                }
            }

            if(lid_ && boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, lid_->face)) {
                if(lid_->face == Face::Y_MIN || lid_->face == Face::Y_MAX) {
                    lbm_.u.x[n] = lbm_lid_velocity;
                } else {
                    lbm_.u.y[n] = lbm_lid_velocity;
                }
            }
        });
    }

private:
    struct Lid { Face face; Speed speed; };                                        ///< the moving wall of a lid-driven cavity
    struct WindProfile { Speed reference_speed; Length reference_height; float32_t alpha; }; ///< power-law wind
    struct SolidShape { Shape shape; uchar flag; VelocityField wall_velocity; };    ///< wall_velocity empty: at rest

    static constexpr Face all_faces[] = { Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Y_MAX, Face::Z_MIN, Face::Z_MAX };
    static std::size_t index(Face face) { return static_cast<std::size_t>(face); }
    static std::size_t axis_index(Face face) { return index(face) / 2u; } ///< X faces 0, Y faces 1, Z faces 2

    void set_velocity(uint64_t n, const Velocity& u) {
        lbm_.u.x[n] = units_.velocity(u.x);
        lbm_.u.y[n] = units_.velocity(u.y);
        lbm_.u.z[n] = units_.velocity(u.z);
    }

    LBM& lbm_;
    UnitScale units_;

    std::array<bool, 6> solid_faces_{}; ///< indexed by Face
    std::array<bool, 3> periodic_{};    ///< indexed by Axis
    bool open_ = false;                 ///< the faces neither solid nor periodic

    std::vector<SolidShape> solids_;
    std::optional<Speed> init_u_x_, init_u_y_, init_u_z_;
    VelocityField velocity_field_;
    PressureField pressure_field_;
    AccelerationField force_field_;
    std::optional<Lid> lid_;
    std::optional<WindProfile> wind_;
    Face wind_direction_ = Face::Y_MIN;
};
