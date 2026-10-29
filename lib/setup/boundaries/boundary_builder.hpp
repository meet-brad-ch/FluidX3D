#pragma once

#include "setup/core/types.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/atmosphere/wind_target.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/domain/shape.hpp"
#include "setup/simulation/runner.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include "lbm.hpp"
#include <array>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <optional>
#include <utility>
#include <vector>

/// @file boundary_builder.hpp
/// @brief The domain's boundaries, solid objects and initial flow in physical units (BoundaryBuilder).

/// @brief Sets the domain's boundaries, solid objects and initial flow in physical units; apply() writes them to the
/// grid (Simulation::boundaries()).
///
/// A temporary builder (L2): Simulation::boundaries() makes a new one each time, and it keeps a reference to the LBM,
/// which must outlive it. apply() writes the core's flags, velocities u, densities rho and (FORCE_FIELD) forces F on
/// the host; the core copies them to the device at its first run. Speeds, pressures and positions (metres from the
/// domain's origin corner; cell i's centre is at (i + 1/2) cell sizes) become lattice values through the simulation's
/// converter. Faces are periodic unless set solid or open. The order per cell: solid faces, open faces (TYPE_E,
/// which win at the edges they share with solid faces), solid shapes, then the velocities and pressures of the
/// non-solid cells and the velocities of moving solids.
///
/// The fields (VelocityField, PressureField, AccelerationField, TimedVelocityField) are the user's own functions, so
/// they are std::function by design: apply() calls each set field once per cell, and a TimedVelocityField is called
/// for each cell of its solid at every time step (with a copy of the velocities from and to the device). This is the
/// cost of a field that the user writes. A uniform value (initialize_velocity_x(), the wind, the lid) is computed
/// once before the cell loop and costs no call.
/// @code
/// using namespace si::symbols;
/// sim.boundaries().set_solid_floor().set_open_boundaries().initialize_velocity_y(10.0 * m / s).apply();
/// sim.boundaries().add_solid(Shape::sphere(center, 1.0 * cm), Solid::MEASURED).set_open_boundaries().initialize_velocity_x(1.0 * m / s).apply();
/// @endcode
class BoundaryBuilder {
public:
    using VelocityField = std::function<si::Velocity(si::Position)>;                        ///< a velocity at a point in metres from the origin corner
    using TimedVelocityField = std::function<si::Velocity(si::Position, si::Duration)>;     ///< a velocity at a point and a simulated time since the start
    using PressureField = std::function<si::Pressure(si::Position)>;                        ///< a pressure relative to the fluid's at rest, at a point
    using AccelerationField = std::function<si::AccelerationVector(si::Position)>;          ///< a body force per mass at a point

    /// The default exponent of the power-law wind: open terrain.
    static constexpr double open_terrain_exponent = 0.143;

    /// @brief Makes a builder for an LBM.
    /// @param lbm the LBM whose flags, velocities and densities are set
    /// @param converter the simulation's conversions (with its density, for pressures)
    /// @param runner the simulation's runner, which drives the moving solids that change in time, or null
    BoundaryBuilder(LBM& lbm, const si::LatticeConverter& converter, Runner* runner = nullptr) : lbm_(lbm), converter_(converter), runner_(runner) {}

    /// @brief Makes the floor (z = 0) solid.
    /// @return this builder
    BoundaryBuilder& set_solid_floor() { return set_solid_faces({ Face::Z_MIN }); }

    /// @brief Makes the ceiling (z = Nz-1) solid.
    /// @return this builder
    BoundaryBuilder& set_solid_ceiling() { return set_solid_faces({ Face::Z_MAX }); }

    /// @brief Makes the four X and Y faces solid walls.
    /// @return this builder
    BoundaryBuilder& set_solid_sides() { return set_solid_faces({ Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Y_MAX }); }

    /// @brief Makes all six faces solid walls: a closed box.
    /// @return this builder
    BoundaryBuilder& set_solid_box() { return set_solid_sides().set_solid_floor().set_solid_ceiling(); }

    /// @brief Makes the given faces solid walls.
    /// @param faces the faces to make solid
    /// @return this builder
    BoundaryBuilder& set_solid_faces(std::initializer_list<Face> faces) {
        for(const Face face : faces) {
            solid_faces_[index(face)] = true;
        }
        return *this;
    }

    /// @brief Makes every face that is not solid or periodic open (TYPE_E): alone all six faces (a free flow), with
    /// set_solid_floor() a wind tunnel.
    /// @return this builder
    BoundaryBuilder& set_open_boundaries() {
        open_ = true;
        return *this;
    }

    /// @brief Keeps the two faces normal to an axis periodic, also with set_open_boundaries() (for example Z of a 2D
    /// domain).
    /// @param axis the axis whose two faces stay periodic
    /// @return this builder
    BoundaryBuilder& set_periodic(Axis axis) {
        periodic_[static_cast<std::size_t>(axis)] = true;
        return *this;
    }

    /// @brief Makes all walls solid, with one face sliding along +y (an X or Z face) or along +x (a Y face).
    /// @param moving_face the sliding face
    /// @param speed the face's sliding speed
    /// @return this builder
    BoundaryBuilder& preset_lid_driven_cavity(Face moving_face, si::Speed speed) {
        set_solid_box();
        lid_ = Lid{ .face = moving_face, .speed = speed };
        return *this;
    }

    /// @brief Adds a solid object at rest.
    /// @param shape the object's region
    /// @param kind Solid::MEASURED: the fluid's force on it is measured (Simulation::forces())
    /// @return this builder
    BoundaryBuilder& add_solid(const Shape& shape, Solid kind = Solid::FIXED) {
        const bool measured{ kind == Solid::MEASURED };
        const uchar flag{ measured ? static_cast<uchar>(TYPE_S | TYPE_X) : static_cast<uchar>(TYPE_S) };
        solids_.push_back(SolidShape{ .shape = shape, .flag = flag, .wall_velocity = {}, .timed_velocity = {} });
        return *this;
    }

    /// @brief Adds a solid object whose surface moves, such as a turning cylinder (MOVING_BOUNDARIES).
    /// @param shape the object's region
    /// @param wall_velocity the velocity of its surface at a point
    /// @return this builder
    BoundaryBuilder& add_moving_solid(const Shape& shape, VelocityField wall_velocity) {
        solids_.push_back(SolidShape{ .shape = shape, .flag = TYPE_S, .wall_velocity = std::move(wall_velocity), .timed_velocity = {} });
        return *this;
    }

    /// @brief Adds a solid object whose surface velocity changes with the simulated time, such as a vibrating
    /// membrane (MOVING_BOUNDARIES): the simulation sets it every time step.
    ///
    /// Ends the program with print_error() for a builder without the simulation's runner.
    /// @param shape the object's region
    /// @param wall_velocity the velocity of its surface at a point and a simulated time
    /// @return this builder
    BoundaryBuilder& add_moving_solid(const Shape& shape, TimedVelocityField wall_velocity) {
        if(runner_ == nullptr) {
            print_error("BoundaryBuilder: a moving solid that changes in time needs the simulation's runner: use Simulation::boundaries()");
        }
        solids_.push_back(SolidShape{ .shape = shape, .flag = TYPE_S, .wall_velocity = {}, .timed_velocity = std::move(wall_velocity) });
        return *this;
    }

    /// @brief Sets a uniform initial velocity along x in all non-solid cells.
    /// @param u the velocity component along x
    /// @return this builder
    BoundaryBuilder& initialize_velocity_x(si::Speed u) {
        init_u_x_ = u;
        return *this;
    }

    /// @brief Sets a uniform initial velocity along y in all non-solid cells.
    /// @param u the velocity component along y
    /// @return this builder
    BoundaryBuilder& initialize_velocity_y(si::Speed u) {
        init_u_y_ = u;
        return *this;
    }

    /// @brief Sets a uniform initial velocity along z in all non-solid cells.
    /// @param u the velocity component along z
    /// @return this builder
    BoundaryBuilder& initialize_velocity_z(si::Speed u) {
        init_u_z_ = u;
        return *this;
    }

    /// @brief Sets a uniform initial velocity in all non-solid cells.
    /// @param u the velocity vector
    /// @return this builder
    BoundaryBuilder& initialize_velocity(si::Velocity u) {
        init_u_x_ = u.x;
        init_u_y_ = u.y;
        init_u_z_ = u.z;
        return *this;
    }

    /// @brief Sets an initial velocity field in the non-solid cells, taken at each cell's centre (after the uniform
    /// velocities).
    /// @param velocity the velocity at a point
    /// @return this builder
    BoundaryBuilder& initialize_velocity(VelocityField velocity) {
        velocity_field_ = std::move(velocity);
        return *this;
    }

    /// @brief Sets an initial pressure field in the non-solid cells, relative to the fluid's at rest; it becomes the
    /// lattice density 1 + 3 p_lattice.
    /// @param pressure the pressure at a point
    /// @return this builder
    BoundaryBuilder& initialize_pressure(PressureField pressure) {
        pressure_field_ = std::move(pressure);
        return *this;
    }

    /// @brief Sets a body force per mass in every cell, in addition to the simulation's uniform one (FORCE_FIELD), for
    /// example a pull toward a point.
    ///
    /// Ends the program with print_error() without FORCE_FIELD in the build.
    /// @param force the force per mass at a point
    /// @return this builder
    BoundaryBuilder& set_force_field(AccelerationField force) {
#ifndef FORCE_FIELD
        print_error("BoundaryBuilder::set_force_field() needs FORCE_FIELD in the EXTENSIONS");
#endif // FORCE_FIELD
        force_field_ = std::move(force);
        return *this;
    }

    /// @brief Sets an atmospheric boundary layer wind u(z) = u_ref (z / z_ref)^alpha in the non-solid cells, with z
    /// the height of a cell's centre above z = 0.
    /// @param reference_speed the wind speed at the reference height
    /// @param reference_height the height at which the reference speed applies, more than 0
    /// @param alpha the power law's exponent (a ratio): 0.10 over the sea, 0.143 open terrain, 0.20 suburbs, 0.25-0.40 urban
    /// @return this builder
    BoundaryBuilder& set_wind_profile_power_law(si::Speed reference_speed, si::Length reference_height, double alpha = open_terrain_exponent) {
        wind_ = WindProfile{ .reference_speed = reference_speed, .reference_height = reference_height, .alpha = alpha };
        return *this;
    }

    /// @brief Sets the face the wind comes from (default Y_MIN: the wind blows toward +y).
    /// @param direction the face the wind comes from
    /// @return this builder
    BoundaryBuilder& set_wind_direction(Face direction) {
        wind_direction_ = direction;
        return *this;
    }

    /// @brief Writes the boundaries, objects and initial flow to the grid, and schedules the moving solids that
    /// change in time.
    void apply() {
        const uint3 cells{ lbm_.get_Nx(), lbm_.get_Ny(), lbm_.get_Nz() };
        const si::Length cell{ converter_.cell() };
        std::vector<Shape::Cells> solid_cells{};
        for(const SolidShape& solid : solids_) {
            Shape::Cells shape_cells{ solid.shape.in_cells(cell, cells) };
            solid_cells.push_back(std::move(shape_cells));
        }
        const LatticeStart start{ lattice_start() };
        parallel_for(lbm_.get_N(), [&](uint64_t n) { write_cell(n, cells, start, solid_cells); });
        for(std::size_t i = 0u; i < solids_.size(); i++) {
            if(solids_[i].timed_velocity) {
                schedule_moving_solid(solid_cells[i], solids_[i].timed_velocity);
            }
        }
    }

private:
    /// The moving wall of a lid-driven cavity.
    struct Lid {
        Face face = Face::Z_MAX;     ///< the sliding face
        si::Speed speed{ au::ZERO }; ///< its sliding speed
    };

    /// A power-law wind.
    struct WindProfile {
        si::Speed reference_speed{ au::ZERO };   ///< the wind speed at the reference height
        si::Length reference_height{ au::ZERO }; ///< the height at which the reference speed applies
        double alpha{};                          ///< the power law's exponent (a ratio)
    };

    /// A solid object; with both velocities empty it is at rest.
    struct SolidShape {
        Shape shape{};                     ///< the object's region
        uchar flag{};                      ///< its cell flag: TYPE_S, or TYPE_S|TYPE_X when measured
        VelocityField wall_velocity{};     ///< the velocity of its surface, or empty
        TimedVelocityField timed_velocity{}; ///< the velocity of its surface in time, or empty
    };

    /// The lattice values that apply() writes, made once before the cell loop, so that the loop reads no optional.
    struct LatticeStart {
        float u_x{};               ///< the uniform initial velocity along x in cells per time step (0 when not set)
        float u_y{};               ///< the uniform initial velocity along y in cells per time step (0 when not set)
        float u_z{};               ///< the uniform initial velocity along z in cells per time step (0 when not set)
        bool has_lid{ false };     ///< true for a lid-driven cavity
        Face lid_face{ Face::Z_MAX }; ///< the lid's sliding face (Z_MAX without a lid)
        FaceLayer lid_layer{ Face::Z_MIN, uint3(1u, 1u, 1u) }; ///< the lid's layer of cells (not read without a lid)
        float lid{};               ///< the lid's speed in cells per time step (0 without a lid)
        bool has_wind{ false };    ///< true when a power-law wind is set
        float wind_speed{};        ///< the wind's reference speed in cells per time step (0 without wind)
        float wind_height{ 1.0f }; ///< the wind's reference height in cells (1 without wind)
        float wind_exponent{};     ///< the power law's exponent as the core's float (0 without wind)
    };

    /// @brief Converts the uniform settings to lattice values.
    /// @return the lattice values
    LatticeStart lattice_start() const {
        LatticeStart start{};
        if(init_u_x_.has_value()) {
            start.u_x = converter_.velocity(init_u_x_.value());
        }
        if(init_u_y_.has_value()) {
            start.u_y = converter_.velocity(init_u_y_.value());
        }
        if(init_u_z_.has_value()) {
            start.u_z = converter_.velocity(init_u_z_.value());
        }
        if(lid_.has_value()) {
            const Lid& lid{ lid_.value() };
            start.has_lid = true;
            const uint3 grid{ lbm_.get_Nx(), lbm_.get_Ny(), lbm_.get_Nz() };
            start.lid_face = lid.face;
            start.lid_layer = FaceLayer(lid.face, grid);
            start.lid = converter_.velocity(lid.speed);
        }
        if(wind_.has_value()) {
            const WindProfile& wind{ wind_.value() };
            start.has_wind = true;
            start.wind_speed = converter_.velocity(wind.reference_speed);
            start.wind_height = static_cast<float>(converter_.cells(wind.reference_height));
            start.wind_exponent = static_cast<float>(wind.alpha);
        }
        return start;
    }

    /// @brief Gives a cell's centre in metres from the origin corner: (i + 1/2) cell sizes along each axis.
    /// @param converter the simulation's conversions
    /// @param x the cell along X, counted from 0
    /// @param y the cell along Y, counted from 0
    /// @param z the cell along Z, counted from 0
    /// @return the centre
    static si::Position cell_centre(const si::LatticeConverter& converter, uint32_t x, uint32_t y, uint32_t z) {
        const double half_cell{ 0.5 };
        const si::Length centre_x{ converter.si_length(static_cast<double>(x) + half_cell) };
        const si::Length centre_y{ converter.si_length(static_cast<double>(y) + half_cell) };
        const si::Length centre_z{ converter.si_length(static_cast<double>(z) + half_cell) };
        const si::Position centre{ centre_x, centre_y, centre_z };
        return centre;
    }

    /// @brief Writes one cell: the faces, the solids, the flow, the force, the wind and the lid.
    /// @param n the cell's index
    /// @param cells the grid's cells along X, Y and Z
    /// @param start the uniform lattice values
    /// @param solid_cells the solids on the grid, indexed as solids_
    void write_cell(uint64_t n, const uint3& cells, const LatticeStart& start, const std::vector<Shape::Cells>& solid_cells) {
        uint32_t x{ 0u };
        uint32_t y{ 0u };
        uint32_t z{ 0u };
        lbm_.coordinates(n, x, y, z);
        const si::Position centre{ cell_centre(converter_, x, y, z) };
        write_faces(n, x, y, z, cells);
        write_solids(n, x, y, z, centre, solid_cells);
        const uchar solid_bit{ static_cast<uchar>(lbm_.flags[n] & TYPE_S) };
        const bool fluid{ solid_bit == 0u };
        if(fluid) {
            write_flow(n, centre, start);
        }
        write_force(n, centre);
        if(start.has_wind && fluid) {
            write_wind(n, z, start);
        }
        if(start.has_lid && start.lid_layer.contains(x, y, z)) {
            write_lid(n, start);
        }
    }

    /// @brief Sets a cell on a solid face to TYPE_S, and a cell on an open face to TYPE_E.
    /// @param n the cell's index
    /// @param x the cell along X
    /// @param y the cell along Y
    /// @param z the cell along Z
    /// @param cells the grid's cells along X, Y and Z
    void write_faces(uint64_t n, uint32_t x, uint32_t y, uint32_t z, const uint3& cells) {
        bool on_open_face{ false };
        for(const Face face : all_faces) {
            const FaceLayer layer(face, cells);
            const bool on_face{ layer.contains(x, y, z) };
            if(!on_face) {
                continue;
            }
            if(solid_faces_[index(face)]) {
                lbm_.flags[n] = TYPE_S;
            } else if(open_ && !periodic_[axis_index(face)]) {
                on_open_face = true;
            }
        }
        if(on_open_face) {
            lbm_.flags[n] = TYPE_E;
        }
    }

    /// @brief Sets a cell inside a solid to the solid's flag and, for a moving solid, to its wall velocity (a solid
    /// that changes in time gets its velocity at 0 s).
    /// @param n the cell's index
    /// @param x the cell along X
    /// @param y the cell along Y
    /// @param z the cell along Z
    /// @param centre the cell's centre
    /// @param solid_cells the solids on the grid, indexed as solids_
    void write_solids(uint64_t n, uint32_t x, uint32_t y, uint32_t z, const si::Position& centre, const std::vector<Shape::Cells>& solid_cells) {
        const si::Duration start_time{ au::ZERO };
        for(std::size_t i = 0u; i < solids_.size(); i++) {
            if(!solid_cells[i].contains(x, y, z)) {
                continue;
            }
            lbm_.flags[n] = solids_[i].flag;
            if(solids_[i].wall_velocity) {
                set_velocity(n, solids_[i].wall_velocity(centre));
            }
            if(solids_[i].timed_velocity) {
                set_velocity(n, solids_[i].timed_velocity(centre, start_time));
            }
        }
    }

    /// @brief Sets a fluid cell's initial velocity and pressure.
    /// @param n the cell's index
    /// @param centre the cell's centre
    /// @param start the uniform lattice values
    void write_flow(uint64_t n, const si::Position& centre, const LatticeStart& start) {
        if(init_u_x_) {
            lbm_.u.x[n] = start.u_x;
        }
        if(init_u_y_) {
            lbm_.u.y[n] = start.u_y;
        }
        if(init_u_z_) {
            lbm_.u.z[n] = start.u_z;
        }
        if(velocity_field_) {
            set_velocity(n, velocity_field_(centre));
        }
        if(pressure_field_) {
            lbm_.rho[n] = converter_.lattice_density(pressure_field_(centre)); // p = c² rho, c² = 1/3
        }
    }

    /// @brief Sets a cell's body force from the force field (FORCE_FIELD): per volume at the lattice density 1, the
    /// acceleration in lattice units.
    /// @param n the cell's index
    /// @param centre the cell's centre
    void write_force(uint64_t n, const si::Position& centre) {
#ifdef FORCE_FIELD
        if(force_field_) {
            const si::AccelerationVector a{ force_field_(centre) };
            lbm_.F.x[n] = converter_.acceleration(a.x);
            lbm_.F.y[n] = converter_.acceleration(a.y);
            lbm_.F.z[n] = converter_.acceleration(a.z);
        }
#else
        (void)n;
        (void)centre;
#endif // FORCE_FIELD
    }

    /// @brief Sets a fluid cell's wind: the power law at the cell centre's height, along the wind direction.
    /// @param n the cell's index
    /// @param z the cell along Z
    /// @param start the uniform lattice values (the wind's reference speed, height and exponent)
    void write_wind(uint64_t n, uint32_t z, const LatticeStart& start) {
        const float height_ratio{ (static_cast<float32_t>(z) + 0.5f) / start.wind_height }; // at the cell's centre
        const float wind_velocity{ start.wind_speed * PowerLawWind::shape(height_ratio, start.wind_exponent) }; // the one power law (the CBL wind uses it too)
        switch(wind_direction_) {
            case Face::X_MIN: lbm_.u.x[n] = wind_velocity; break;
            case Face::X_MAX: lbm_.u.x[n] = -wind_velocity; break;
            case Face::Y_MIN: lbm_.u.y[n] = wind_velocity; break;
            case Face::Y_MAX: lbm_.u.y[n] = -wind_velocity; break;
            case Face::Z_MIN: lbm_.u.z[n] = wind_velocity; break;
            case Face::Z_MAX: lbm_.u.z[n] = -wind_velocity; break;
        }
    }

    /// @brief Sets a lid cell's sliding velocity: along +x on a Y face, along +y on an X or Z face.
    /// @param n the cell's index
    /// @param start the uniform lattice values (the lid's face and speed)
    void write_lid(uint64_t n, const LatticeStart& start) {
        const bool y_face{ start.lid_face == Face::Y_MIN || start.lid_face == Face::Y_MAX };
        if(y_face) {
            lbm_.u.x[n] = start.lid;
        } else {
            lbm_.u.y[n] = start.lid;
        }
    }

    /// @brief Schedules a task that sets the solid's cells to their velocity at the simulated time, every time step.
    ///
    /// The core marks the cells next to a moving wall when its velocity is not zero; that is done once, when the wall
    /// first moves (MOVING_BOUNDARIES).
    /// @param shape the solid's cells
    /// @param velocity the velocity of its surface at a point and a simulated time
    void schedule_moving_solid(const Shape::Cells& shape, const TimedVelocityField& velocity) const {
        std::vector<uint64_t> cells{};
        for(uint64_t n = 0ull; n < lbm_.get_N(); n++) {
            uint32_t x{ 0u };
            uint32_t y{ 0u };
            uint32_t z{ 0u };
            lbm_.coordinates(n, x, y, z);
            if(shape.contains(x, y, z)) {
                cells.push_back(n);
            }
        }
        runner_->every_step([&lbm = lbm_, converter = converter_, cells = std::move(cells), velocity, marked = false](si::Duration time) mutable {
            lbm.u.read_from_device();
            const bool moving{ set_wall_velocities(lbm, converter, cells, velocity, time) };
            lbm.u.write_to_device();
#ifdef MOVING_BOUNDARIES
            if(moving && !marked) {
                lbm.update_moving_boundaries();
                marked = true;
            }
#else
            (void)moving;
            (void)marked;
#endif // MOVING_BOUNDARIES
        });
    }

    /// @brief Sets the host velocities of a moving solid's cells to the wall velocity at a simulated time.
    /// @param lbm the LBM whose host velocities are set
    /// @param converter the simulation's conversions
    /// @param cells the indices of the solid's cells
    /// @param velocity the velocity of the solid's surface at a point and a simulated time
    /// @param time the simulated time since the start
    /// @return true when a cell's velocity is not zero
    static bool set_wall_velocities(LBM& lbm, const si::LatticeConverter& converter, const std::vector<uint64_t>& cells, const TimedVelocityField& velocity,
                                    si::Duration time) {
        bool moving{ false };
        const si::Speed still{ au::ZERO };
        for(const uint64_t n : cells) {
            uint32_t x{ 0u };
            uint32_t y{ 0u };
            uint32_t z{ 0u };
            lbm.coordinates(n, x, y, z);
            const si::Position centre{ cell_centre(converter, x, y, z) };
            const si::Velocity u{ velocity(centre, time) };
            lbm.u.x[n] = converter.velocity(u.x);
            lbm.u.y[n] = converter.velocity(u.y);
            lbm.u.z[n] = converter.velocity(u.z);
            const bool cell_moves{ u.x != still || u.y != still || u.z != still };
            moving = moving || cell_moves;
        }
        return moving;
    }

    static constexpr std::size_t face_count{ 6u }; ///< the faces of the box
    static constexpr std::array<Face, face_count> all_faces{ Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Y_MAX, Face::Z_MIN, Face::Z_MAX }; ///< every face

    /// @brief Gives a face's index in the per-face arrays.
    /// @param face a face of the box
    /// @return the index, 0 to 5 in the order of Face
    static std::size_t index(Face face) { return static_cast<std::size_t>(face); }

    /// @brief Gives the index of a face's axis.
    /// @param face a face of the box
    /// @return 0 for the X faces, 1 for the Y faces, 2 for the Z faces
    static std::size_t axis_index(Face face) { return index(face) / 2u; }

    /// @brief Sets a cell's lattice velocity.
    /// @param n the cell's index
    /// @param u the velocity
    void set_velocity(uint64_t n, const si::Velocity& u) {
        lbm_.u.x[n] = converter_.velocity(u.x);
        lbm_.u.y[n] = converter_.velocity(u.y);
        lbm_.u.z[n] = converter_.velocity(u.z);
    }

    LBM& lbm_;                       ///< the LBM whose grid is set (not owned)
    si::LatticeConverter converter_; ///< the simulation's conversions
    Runner* runner_{ nullptr };      ///< the simulation's runner, or null (not owned)

    std::array<bool, face_count> solid_faces_{}; ///< true for the solid faces, indexed by Face
    std::array<bool, 3> periodic_{};    ///< true for the axes whose faces stay periodic, indexed by Axis
    bool open_ = false;                 ///< true when the faces neither solid nor periodic are open

    std::vector<SolidShape> solids_{};        ///< the solid objects
    std::optional<si::Speed> init_u_x_{};     ///< the uniform initial velocity along x, or none
    std::optional<si::Speed> init_u_y_{};     ///< the uniform initial velocity along y, or none
    std::optional<si::Speed> init_u_z_{};     ///< the uniform initial velocity along z, or none
    VelocityField velocity_field_{};          ///< the initial velocity field, or empty
    PressureField pressure_field_{};          ///< the initial pressure field, or empty
    AccelerationField force_field_{};         ///< the body force field, or empty
    std::optional<Lid> lid_{};                ///< the lid of a lid-driven cavity, or none
    std::optional<WindProfile> wind_{};       ///< the wind profile, or none
    Face wind_direction_ = Face::Y_MIN;       ///< the face the wind comes from
};
