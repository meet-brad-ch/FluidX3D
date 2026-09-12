#pragma once

#include "core/types.hpp"
#include "core/boundary_utils.hpp"
#include "boundaries/boundary_flags.hpp"
#include "lbm.hpp"
#include "units.hpp"

extern Units units;  // Global units object from lbm.cpp

/**
 * @file boundary_builder.hpp
 * @brief Fluent boundary condition builder for FluidX3D simulations
 *
 * Provides a readable, SI-unit based API for setting up boundary conditions
 * and initializing velocity fields.
 *
 * @note Cell type flags TYPE_S, TYPE_E, TYPE_T are defined in defines.hpp
 * @note Face enum is defined in boundary_flags.hpp
 */

/**
 * @class BoundaryBuilder
 * @brief Fluent API for setting boundary conditions in SI units
 *
 * This class simplifies the common pattern of iterating over all cells
 * to set boundary conditions. It works with SI units and converts
 * automatically to LBM units.
 *
 * @par Example:
 * @code
 * BoundaryBuilder boundaries(lbm);
 * boundaries
 *     .set_solid_floor()                      // z=0 is solid
 *     .set_open_boundaries()                  // all other walls are open
 *     .set_inlet_velocity(Face::Y_MIN, 15.0f) // 15 m/s inlet
 *     .initialize_velocity_y(15.0f)           // init whole domain
 *     .apply();
 * @endcode
 *
 * @note Call apply() to execute all queued operations
 */
class BoundaryBuilder {
public:
    /**
     * @brief Construct BoundaryBuilder for an LBM simulation
     * @param lbm Reference to the LBM simulation object
     */
    explicit BoundaryBuilder(LBM& lbm) : lbm_(lbm) {}

    // ========================================================================
    // Common boundary presets
    // ========================================================================

    /**
     * @brief Set solid no-slip floor at z=0
     * @param flag Boundary type (default: TYPE_S)
     * @return Reference for method chaining
     */
    BoundaryBuilder& set_solid_floor(uchar flag = TYPE_S) {
        floor_flag_ = flag;
        apply_floor_ = true;
        return *this;
    }

    /**
     * @brief Set solid no-slip ceiling at z=Nz-1
     * @param flag Boundary type (default: TYPE_S)
     * @return Reference for method chaining
     */
    BoundaryBuilder& set_solid_ceiling(uchar flag = TYPE_S) {
        ceiling_flag_ = flag;
        apply_ceiling_ = true;
        return *this;
    }

    /**
     * @brief Set all side walls as solid (x and y boundaries)
     * @param flag Boundary type (default: TYPE_S)
     * @return Reference for method chaining
     */
    BoundaryBuilder& set_solid_walls(uchar flag = TYPE_S) {
        walls_flag_ = flag;
        apply_walls_ = true;
        return *this;
    }

    /**
     * @brief Set open (equilibrium) boundaries on all domain faces except floor
     *
     * This is typical for wind tunnel simulations where air flows through
     * the domain with a solid ground plane.
     *
     * @return Reference for method chaining
     */
    BoundaryBuilder& set_open_boundaries() {
        open_x_ = open_y_ = open_z_max_ = true;
        return *this;
    }

    /**
     * @brief Set all faces as open (equilibrium) boundaries
     * @return Reference for method chaining
     */
    BoundaryBuilder& set_all_open() {
        open_x_ = open_y_ = open_z_min_ = open_z_max_ = true;
        return *this;
    }

    // ========================================================================
    // High-level presets (common simulation configurations)
    // ========================================================================

    /**
     * @brief Configure wind tunnel boundary conditions
     *
     * Sets up a typical wind tunnel configuration:
     * - Solid floor (z=0)
     * - Open boundaries on all other faces
     * - Inlet velocity in specified flow direction
     * - Domain initialized with uniform velocity
     *
     * @param flow_axis Flow direction (X, Y, or Z)
     * @param velocity_mps Freestream velocity in m/s
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * BoundaryBuilder(lbm)
     *     .preset_wind_tunnel(Axis::Y, 15.0f)  // 15 m/s flow in Y direction
     *     .apply();
     * @endcode
     */
    BoundaryBuilder& preset_wind_tunnel(Axis flow_axis, float32_t velocity_mps) {
        set_solid_floor();
        set_open_boundaries();

        switch(flow_axis) {
            case Axis::X:
                has_inlet_ = true;
                inlet_face_ = Face::X_MIN;
                inlet_velocity_ = velocity_mps;
                init_u_x_ = velocity_mps;
                has_init_u_x_ = true;
                break;
            case Axis::Y:
                has_inlet_ = true;
                inlet_face_ = Face::Y_MIN;
                inlet_velocity_ = velocity_mps;
                init_u_y_ = velocity_mps;
                has_init_u_y_ = true;
                break;
            case Axis::Z:
                has_inlet_ = true;
                inlet_face_ = Face::Z_MIN;
                inlet_velocity_ = velocity_mps;
                init_u_z_ = velocity_mps;
                has_init_u_z_ = true;
                break;
        }
        return *this;
    }

    /**
     * @brief Configure channel flow boundary conditions
     *
     * Sets up a typical channel flow (pipe flow) configuration:
     * - Solid walls on sides perpendicular to flow
     * - Open inlet/outlet on flow direction faces
     * - Domain initialized with uniform velocity
     *
     * @param flow_axis Flow direction (X, Y, or Z)
     * @param velocity_mps Bulk velocity in m/s
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * BoundaryBuilder(lbm)
     *     .preset_channel_flow(Axis::Y, 1.0f)  // 1 m/s flow in Y direction
     *     .apply();
     * @endcode
     */
    BoundaryBuilder& preset_channel_flow(Axis flow_axis, float32_t velocity_mps) {
        switch(flow_axis) {
            case Axis::X:
                // Walls on Y and Z faces, open on X faces
                apply_walls_y_ = true;
                apply_walls_z_ = true;
                open_x_min_ = true;
                open_x_max_ = true;
                init_u_x_ = velocity_mps;
                has_init_u_x_ = true;
                break;
            case Axis::Y:
                // Walls on X and Z faces, open on Y faces
                apply_walls_x_ = true;
                apply_walls_z_ = true;
                open_y_min_ = true;
                open_y_max_ = true;
                init_u_y_ = velocity_mps;
                has_init_u_y_ = true;
                break;
            case Axis::Z:
                // Walls on X and Y faces, open on Z faces
                apply_walls_x_ = true;
                apply_walls_y_ = true;
                open_z_min_ = true;
                open_z_max_ = true;
                init_u_z_ = velocity_mps;
                has_init_u_z_ = true;
                break;
        }
        return *this;
    }

    /**
     * @brief Configure lid-driven cavity boundary conditions
     *
     * Sets up a classic lid-driven cavity:
     * - All walls solid (no-slip)
     * - One face moves with specified velocity
     *
     * @param moving_face Which face is the moving lid
     * @param velocity_mps Lid velocity in m/s
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * BoundaryBuilder(lbm)
     *     .preset_lid_driven_cavity(Face::Z_MAX, 1.0f)  // Top lid moves at 1 m/s
     *     .apply();
     * @endcode
     *
     * @note The lid moves in the positive direction of the first perpendicular axis.
     *       For Z_MAX, this means motion in +X direction.
     */
    BoundaryBuilder& preset_lid_driven_cavity(Face moving_face, float32_t velocity_mps) {
        // All walls solid
        set_solid_floor();
        set_solid_ceiling();
        set_solid_walls();

        // Store lid configuration
        has_moving_lid_ = true;
        lid_face_ = moving_face;
        lid_velocity_ = velocity_mps;

        return *this;
    }

    /**
     * @brief Configure open domain boundary conditions
     *
     * Sets up a fully open domain (equilibrium boundaries everywhere):
     * - All faces are open (TYPE_E)
     * - Domain initialized with uniform ambient velocity
     *
     * @param ambient_velocity_mps Ambient velocity in m/s
     * @param flow_axis Primary flow direction (X, Y, or Z)
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * BoundaryBuilder(lbm)
     *     .preset_open_domain(10.0f, Axis::Y)  // 10 m/s ambient flow in Y
     *     .apply();
     * @endcode
     */
    BoundaryBuilder& preset_open_domain(float32_t ambient_velocity_mps, Axis flow_axis) {
        set_all_open();

        switch(flow_axis) {
            case Axis::X:
                init_u_x_ = ambient_velocity_mps;
                has_init_u_x_ = true;
                break;
            case Axis::Y:
                init_u_y_ = ambient_velocity_mps;
                has_init_u_y_ = true;
                break;
            case Axis::Z:
                init_u_z_ = ambient_velocity_mps;
                has_init_u_z_ = true;
                break;
        }
        return *this;
    }

    // ========================================================================
    // Per-face configuration
    // ========================================================================

    /**
     * @brief Set a specific face as equilibrium boundary (open)
     * @param face Which face to configure
     * @return Reference for method chaining
     */
    BoundaryBuilder& set_open_face(Face face) {
        switch(face) {
            case Face::X_MIN: open_x_min_ = true; break;
            case Face::X_MAX: open_x_max_ = true; break;
            case Face::Y_MIN: open_y_min_ = true; break;
            case Face::Y_MAX: open_y_max_ = true; break;
            case Face::Z_MIN: open_z_min_ = true; break;
            case Face::Z_MAX: open_z_max_ = true; break;
        }
        return *this;
    }

    /**
     * @brief Set inlet velocity on a specific face (SI units)
     * @param face Which face is the inlet
     * @param si_velocity Velocity magnitude in m/s
     * @return Reference for method chaining
     *
     * @note The velocity direction is automatically set based on face normal
     */
    BoundaryBuilder& set_inlet_velocity(Face face, float32_t si_velocity) {
        inlet_face_ = face;
        inlet_velocity_ = si_velocity;
        has_inlet_ = true;
        return *this;
    }

    // ========================================================================
    // Velocity initialization
    // ========================================================================

    /**
     * @brief Initialize velocity in X direction for all fluid cells (SI units)
     * @param si_velocity Velocity in m/s
     * @return Reference for method chaining
     */
    BoundaryBuilder& initialize_velocity_x(float32_t si_velocity) {
        init_u_x_ = si_velocity;
        has_init_u_x_ = true;
        return *this;
    }

    /**
     * @brief Initialize velocity in Y direction for all fluid cells (SI units)
     * @param si_velocity Velocity in m/s
     * @return Reference for method chaining
     */
    BoundaryBuilder& initialize_velocity_y(float32_t si_velocity) {
        init_u_y_ = si_velocity;
        has_init_u_y_ = true;
        return *this;
    }

    /**
     * @brief Initialize velocity in Z direction for all fluid cells (SI units)
     * @param si_velocity Velocity in m/s
     * @return Reference for method chaining
     */
    BoundaryBuilder& initialize_velocity_z(float32_t si_velocity) {
        init_u_z_ = si_velocity;
        has_init_u_z_ = true;
        return *this;
    }

    /**
     * @brief Initialize 3D velocity vector for all fluid cells (SI units)
     * @param si_vx X velocity in m/s
     * @param si_vy Y velocity in m/s
     * @param si_vz Z velocity in m/s
     * @return Reference for method chaining
     */
    BoundaryBuilder& initialize_velocity(float32_t si_vx, float32_t si_vy, float32_t si_vz) {
        init_u_x_ = si_vx; has_init_u_x_ = true;
        init_u_y_ = si_vy; has_init_u_y_ = true;
        init_u_z_ = si_vz; has_init_u_z_ = true;
        return *this;
    }

    // ========================================================================
    // LBM-unit Velocity Initialization (for Re-based simulations)
    // ========================================================================

    /**
     * @brief Initialize velocity in X direction for all fluid cells (LBM units)
     *
     * Use this for Reynolds number based simulations where velocity is
     * already in LBM units (no SI conversion needed).
     *
     * @param lbm_velocity Velocity in LBM units
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * sim.configure_reynolds(100000.0f, 0.075f);
     * BoundaryBuilder(lbm)
     *     .set_all_open()
     *     .initialize_velocity_lbm_x(sim.get_lbm_reference_velocity())
     *     .apply();
     * @endcode
     */
    BoundaryBuilder& initialize_velocity_lbm_x(float32_t lbm_velocity) {
        init_u_x_ = lbm_velocity;
        has_init_u_x_ = true;
        is_lbm_units_x_ = true;
        return *this;
    }

    /**
     * @brief Initialize velocity in Y direction for all fluid cells (LBM units)
     * @param lbm_velocity Velocity in LBM units
     * @return Reference for method chaining
     */
    BoundaryBuilder& initialize_velocity_lbm_y(float32_t lbm_velocity) {
        init_u_y_ = lbm_velocity;
        has_init_u_y_ = true;
        is_lbm_units_y_ = true;
        return *this;
    }

    /**
     * @brief Initialize velocity in Z direction for all fluid cells (LBM units)
     * @param lbm_velocity Velocity in LBM units
     * @return Reference for method chaining
     */
    BoundaryBuilder& initialize_velocity_lbm_z(float32_t lbm_velocity) {
        init_u_z_ = lbm_velocity;
        has_init_u_z_ = true;
        is_lbm_units_z_ = true;
        return *this;
    }

    /**
     * @brief Initialize 3D velocity vector for all fluid cells (LBM units)
     * @param lbm_vx X velocity in LBM units
     * @param lbm_vy Y velocity in LBM units
     * @param lbm_vz Z velocity in LBM units
     * @return Reference for method chaining
     */
    BoundaryBuilder& initialize_velocity_lbm(float32_t lbm_vx, float32_t lbm_vy, float32_t lbm_vz) {
        init_u_x_ = lbm_vx; has_init_u_x_ = true; is_lbm_units_x_ = true;
        init_u_y_ = lbm_vy; has_init_u_y_ = true; is_lbm_units_y_ = true;
        init_u_z_ = lbm_vz; has_init_u_z_ = true; is_lbm_units_z_ = true;
        return *this;
    }

    // ========================================================================
    // Consolidated Velocity API (using Axis and UnitSystem enums)
    // ========================================================================

    /**
     * @brief Initialize velocity for a single axis with unit system specification
     *
     * This consolidated method replaces the 6 individual velocity methods
     * (initialize_velocity_x/y/z and initialize_velocity_lbm_x/y/z).
     *
     * @param axis Which axis to initialize (X, Y, or Z)
     * @param value Velocity value
     * @param unit_system Unit system (SI or LBM, default: SI)
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * BoundaryBuilder(lbm)
     *     .init_velocity(Axis::Y, 15.0f)              // 15 m/s in Y (SI)
     *     .init_velocity(Axis::X, 0.05f, UnitSystem::LBM)  // 0.05 in X (LBM)
     *     .apply();
     * @endcode
     */
    BoundaryBuilder& init_velocity(Axis axis, float32_t value, UnitSystem unit_system = UnitSystem::SI) {
        bool is_lbm = (unit_system == UnitSystem::LBM);
        switch(axis) {
            case Axis::X:
                init_u_x_ = value;
                has_init_u_x_ = true;
                is_lbm_units_x_ = is_lbm;
                break;
            case Axis::Y:
                init_u_y_ = value;
                has_init_u_y_ = true;
                is_lbm_units_y_ = is_lbm;
                break;
            case Axis::Z:
                init_u_z_ = value;
                has_init_u_z_ = true;
                is_lbm_units_z_ = is_lbm;
                break;
        }
        return *this;
    }

    /**
     * @brief Initialize 3D velocity vector with unit system specification
     *
     * @param velocity 3D velocity vector (vx, vy, vz)
     * @param unit_system Unit system (SI or LBM, default: SI)
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * BoundaryBuilder(lbm)
     *     .init_velocity(float3(0.0f, 15.0f, 0.0f))  // SI units
     *     .init_velocity(float3(0.0f, 0.05f, 0.0f), UnitSystem::LBM)  // LBM units
     *     .apply();
     * @endcode
     */
    BoundaryBuilder& init_velocity(float3 velocity, UnitSystem unit_system = UnitSystem::SI) {
        bool is_lbm = (unit_system == UnitSystem::LBM);
        init_u_x_ = velocity.x; has_init_u_x_ = true; is_lbm_units_x_ = is_lbm;
        init_u_y_ = velocity.y; has_init_u_y_ = true; is_lbm_units_y_ = is_lbm;
        init_u_z_ = velocity.z; has_init_u_z_ = true; is_lbm_units_z_ = is_lbm;
        return *this;
    }

    // ========================================================================
    // Volume Forces (Gravity, Wind)
    // ========================================================================

    /**
     * @brief Set gravitational acceleration (SI units, default downward -Z)
     * @param si_gravity Gravitational acceleration magnitude in m/s² (default: 9.81)
     * @return Reference for method chaining
     *
     * @note Gravity acts on the simulated fluid (the density of configure_units()).
     *       It is applied in the -Z direction by default; use set_gravity_direction() for custom directions.
     */
    BoundaryBuilder& set_gravity(float32_t si_gravity = 9.81f) {
        gravity_magnitude_ = si_gravity;
        gravity_direction_ = float3(0.0f, 0.0f, -1.0f);  // Default: -Z
        has_gravity_ = true;
        return *this;
    }

    /**
     * @brief Set custom gravity direction (normalized internally)
     * @param gx X component of gravity direction
     * @param gy Y component of gravity direction
     * @param gz Z component of gravity direction
     * @return Reference for method chaining
     *
     * @par Example:
     * @code
     * builder.set_gravity(9.81f).set_gravity_direction(0.0f, -1.0f, 0.0f);  // Gravity in -Y
     * @endcode
     */
    BoundaryBuilder& set_gravity_direction(float32_t gx, float32_t gy, float32_t gz) {
        float32_t len = sqrt(gx*gx + gy*gy + gz*gz);
        if (len > 0.0f) {
            gravity_direction_ = float3(gx/len, gy/len, gz/len);
        }
        return *this;
    }

    /**
     * @brief Initialize hydrostatic pressure/density gradient
     * @param si_fluid_height_m Height of fluid column in meters
     * @return Reference for method chaining
     *
     * @note Creates a density gradient that increases with depth.
     *       Requires set_gravity() to be called first.
     *
     * @par Example:
     * @code
     * builder.set_gravity(9.81f)            // gravity on the fluid
     *        .initialize_hydrostatic_pressure(10.0f)  // 10m water column
     *        .apply();
     * @endcode
     */
    BoundaryBuilder& initialize_hydrostatic_pressure(float32_t si_fluid_height_m) {
        hydrostatic_height_ = si_fluid_height_m;
        has_hydrostatic_ = true;
        return *this;
    }

    /**
     * @brief Set atmospheric boundary layer wind profile (power law)
     *
     * Applies velocity profile: u(z) = u_ref * (z / z_ref)^alpha
     *
     * @param si_reference_velocity Reference velocity at reference height (m/s)
     * @param si_reference_height_m Reference height in meters
     * @param alpha Power law exponent (default: 0.143 for neutral atmosphere)
     * @return Reference for method chaining
     *
     * @note Common alpha values:
     *       - 0.10: Smooth sea surface
     *       - 0.143 (1/7): Neutral atmosphere, open terrain
     *       - 0.20: Suburbs, scattered trees
     *       - 0.25-0.40: Urban areas, forests
     *
     * @par Example:
     * @code
     * builder.set_wind_profile_power_law(15.0f, 10.0f, 0.143f)  // 15 m/s at 10m height
     *        .apply();
     * @endcode
     */
    BoundaryBuilder& set_wind_profile_power_law(float32_t si_reference_velocity,
                                                 float32_t si_reference_height_m,
                                                 float32_t alpha = 0.143f) {
        wind_profile_velocity_ = si_reference_velocity;
        wind_profile_height_ = si_reference_height_m;
        wind_profile_alpha_ = alpha;
        has_wind_profile_ = true;
        return *this;
    }

    /**
     * @brief Set wind profile flow direction
     * @param direction Flow direction axis (default: Y_MIN to Y_MAX)
     * @return Reference for method chaining
     */
    BoundaryBuilder& set_wind_direction(Face direction) {
        wind_direction_ = direction;
        return *this;
    }

    // ========================================================================
    // Apply all settings
    // ========================================================================

    /**
     * @brief Apply all configured boundary conditions to the LBM grid
     *
     * This method iterates over all cells and applies the configured
     * boundary conditions and velocity initialization.
     */
    void apply() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        // Convert velocities to LBM units (skip conversion if already in LBM units)
        const float32_t lbm_init_u_x = has_init_u_x_ ? (is_lbm_units_x_ ? init_u_x_ : units.u(init_u_x_)) : 0.0f;
        const float32_t lbm_init_u_y = has_init_u_y_ ? (is_lbm_units_y_ ? init_u_y_ : units.u(init_u_y_)) : 0.0f;
        const float32_t lbm_init_u_z = has_init_u_z_ ? (is_lbm_units_z_ ? init_u_z_ : units.u(init_u_z_)) : 0.0f;
        const float32_t lbm_inlet_velocity = has_inlet_ ? units.u(inlet_velocity_) : 0.0f;
        const float32_t lbm_lid_velocity = has_moving_lid_ ? units.u(lid_velocity_) : 0.0f;

        // Apply gravity force via lbm.set_f()
        if (has_gravity_) {
            const float32_t lbm_f = units.g(gravity_magnitude_); // volume force rho*g with the LBM density 1
            lbm_.set_f(
                gravity_direction_.x * lbm_f,
                gravity_direction_.y * lbm_f,
                gravity_direction_.z * lbm_f
            );
        }

        // Pre-calculate hydrostatic parameters
        const float32_t lbm_hydro_f = has_hydrostatic_ ? units.g(gravity_magnitude_) : 0.0f;
        const float32_t lbm_hydro_height = has_hydrostatic_ ? units.x(hydrostatic_height_) : 0.0f;

        // Pre-calculate wind profile parameters
        const float32_t lbm_wind_ref_velocity = has_wind_profile_ ? units.u(wind_profile_velocity_) : 0.0f;
        const float32_t lbm_wind_ref_height = has_wind_profile_ ? units.x(wind_profile_height_) : 1.0f;

        parallel_for(lbm_.get_N(), [&](uint64_t n) {
            uint32_t x = 0u, y = 0u, z = 0u;
            lbm_.coordinates(n, x, y, z);

            // Apply floor boundary (z = 0)
            if (apply_floor_ && z == 0u) {
                lbm_.flags[n] = floor_flag_;
            }

            // Apply ceiling boundary (z = Nz-1)
            if (apply_ceiling_ && z == Nz - 1u) {
                lbm_.flags[n] = ceiling_flag_;
            }

            // Apply solid walls (x and y boundaries)
            if (apply_walls_) {
                if (x == 0u || x == Nx - 1u || y == 0u || y == Ny - 1u) {
                    lbm_.flags[n] = walls_flag_;
                }
            }

            // Apply per-axis walls (for channel flow preset)
            if (apply_walls_x_ && (x == 0u || x == Nx - 1u)) {
                lbm_.flags[n] = walls_flag_;
            }
            if (apply_walls_y_ && (y == 0u || y == Ny - 1u)) {
                lbm_.flags[n] = walls_flag_;
            }
            if (apply_walls_z_ && (z == 0u || z == Nz - 1u)) {
                lbm_.flags[n] = walls_flag_;
            }

            // Apply open (equilibrium) boundaries
            bool is_open_boundary = false;

            if (open_x_ || open_x_min_) {
                if (x == 0u) is_open_boundary = true;
            }
            if (open_x_ || open_x_max_) {
                if (x == Nx - 1u) is_open_boundary = true;
            }
            if (open_y_ || open_y_min_) {
                if (y == 0u) is_open_boundary = true;
            }
            if (open_y_ || open_y_max_) {
                if (y == Ny - 1u) is_open_boundary = true;
            }
            if (open_z_min_) {
                if (z == 0u) is_open_boundary = true;
            }
            if (open_z_max_) {
                if (z == Nz - 1u) is_open_boundary = true;
            }

            if (is_open_boundary) {
                lbm_.flags[n] = TYPE_E;
            }

            // Initialize velocity in fluid cells (not solid)
            if (!(lbm_.flags[n] & TYPE_S)) {
                if (has_init_u_x_) lbm_.u.x[n] = lbm_init_u_x;
                if (has_init_u_y_) lbm_.u.y[n] = lbm_init_u_y;
                if (has_init_u_z_) lbm_.u.z[n] = lbm_init_u_z;
            }

            // Apply inlet velocity on specified face
            if (has_inlet_ && !(lbm_.flags[n] & TYPE_S)) {
                bool is_inlet = false;
                switch (inlet_face_) {
                    case Face::X_MIN: is_inlet = (x == 0u); break;
                    case Face::X_MAX: is_inlet = (x == Nx - 1u); break;
                    case Face::Y_MIN: is_inlet = (y == 0u); break;
                    case Face::Y_MAX: is_inlet = (y == Ny - 1u); break;
                    case Face::Z_MIN: is_inlet = (z == 0u); break;
                    case Face::Z_MAX: is_inlet = (z == Nz - 1u); break;
                }
                if (is_inlet) {
                    // Set velocity in direction normal to face
                    switch (inlet_face_) {
                        case Face::X_MIN: lbm_.u.x[n] = lbm_inlet_velocity; break;
                        case Face::X_MAX: lbm_.u.x[n] = -lbm_inlet_velocity; break;
                        case Face::Y_MIN: lbm_.u.y[n] = lbm_inlet_velocity; break;
                        case Face::Y_MAX: lbm_.u.y[n] = -lbm_inlet_velocity; break;
                        case Face::Z_MIN: lbm_.u.z[n] = lbm_inlet_velocity; break;
                        case Face::Z_MAX: lbm_.u.z[n] = -lbm_inlet_velocity; break;
                    }
                }
            }

            // Initialize hydrostatic pressure (density gradient)
            if (has_hydrostatic_ && !(lbm_.flags[n] & TYPE_S)) {
                lbm_.rho[n] = units.rho_hydrostatic(lbm_hydro_f, (float32_t)z, lbm_hydro_height);
            }

            // Apply wind profile (atmospheric boundary layer)
            if (has_wind_profile_ && !(lbm_.flags[n] & TYPE_S) && z > 0u) {
                const float32_t height_ratio = (float32_t)z / lbm_wind_ref_height;
                const float32_t wind_velocity = lbm_wind_ref_velocity * pow(height_ratio, wind_profile_alpha_);

                // Apply velocity based on wind direction
                switch (wind_direction_) {
                    case Face::X_MIN: lbm_.u.x[n] = wind_velocity; break;
                    case Face::X_MAX: lbm_.u.x[n] = -wind_velocity; break;
                    case Face::Y_MIN: lbm_.u.y[n] = wind_velocity; break;
                    case Face::Y_MAX: lbm_.u.y[n] = -wind_velocity; break;
                    case Face::Z_MIN: lbm_.u.z[n] = wind_velocity; break;
                    case Face::Z_MAX: lbm_.u.z[n] = -wind_velocity; break;
                }
            }

            // Apply moving lid velocity (for lid-driven cavity preset)
            if (has_moving_lid_) {
                bool is_lid = false;
                switch (lid_face_) {
                    case Face::X_MIN: is_lid = (x == 0u); break;
                    case Face::X_MAX: is_lid = (x == Nx - 1u); break;
                    case Face::Y_MIN: is_lid = (y == 0u); break;
                    case Face::Y_MAX: is_lid = (y == Ny - 1u); break;
                    case Face::Z_MIN: is_lid = (z == 0u); break;
                    case Face::Z_MAX: is_lid = (z == Nz - 1u); break;
                }
                if (is_lid) {
                    // Lid moves in the first perpendicular direction
                    // X face → motion in Y, Y face → motion in X, Z face → motion in X
                    switch (lid_face_) {
                        case Face::X_MIN:
                        case Face::X_MAX:
                            lbm_.u.y[n] = lbm_lid_velocity;
                            break;
                        case Face::Y_MIN:
                        case Face::Y_MAX:
                            lbm_.u.x[n] = lbm_lid_velocity;
                            break;
                        case Face::Z_MIN:
                        case Face::Z_MAX:
                            lbm_.u.x[n] = lbm_lid_velocity;
                            break;
                    }
                }
            }
        });
    }

private:
    LBM& lbm_;

    // Floor/ceiling/wall flags
    uchar floor_flag_ = TYPE_S;
    uchar ceiling_flag_ = TYPE_S;
    uchar walls_flag_ = TYPE_S;
    bool apply_floor_ = false;
    bool apply_ceiling_ = false;
    bool apply_walls_ = false;

    // Per-axis wall flags (for channel flow preset)
    bool apply_walls_x_ = false;  // Walls at x=0 and x=Nx-1
    bool apply_walls_y_ = false;  // Walls at y=0 and y=Ny-1
    bool apply_walls_z_ = false;  // Walls at z=0 and z=Nz-1

    // Moving lid configuration (for lid-driven cavity preset)
    bool has_moving_lid_ = false;
    Face lid_face_ = Face::Z_MAX;
    float32_t lid_velocity_ = 0.0f;

    // Open boundary flags
    bool open_x_ = false;
    bool open_y_ = false;
    bool open_x_min_ = false;
    bool open_x_max_ = false;
    bool open_y_min_ = false;
    bool open_y_max_ = false;
    bool open_z_min_ = false;
    bool open_z_max_ = false;

    // Inlet configuration
    bool has_inlet_ = false;
    Face inlet_face_ = Face::Y_MIN;
    float32_t inlet_velocity_ = 0.0f;

    // Velocity initialization
    bool has_init_u_x_ = false;
    bool has_init_u_y_ = false;
    bool has_init_u_z_ = false;
    float32_t init_u_x_ = 0.0f;
    float32_t init_u_y_ = 0.0f;
    float32_t init_u_z_ = 0.0f;

    // LBM units flags (skip SI conversion if true)
    bool is_lbm_units_x_ = false;
    bool is_lbm_units_y_ = false;
    bool is_lbm_units_z_ = false;

    // Gravity configuration
    bool has_gravity_ = false;
    float32_t gravity_magnitude_ = 9.81f;
    float3 gravity_direction_ = float3(0.0f, 0.0f, -1.0f);

    // Hydrostatic pressure configuration
    bool has_hydrostatic_ = false;
    float32_t hydrostatic_height_ = 0.0f;

    // Wind profile configuration (atmospheric boundary layer)
    bool has_wind_profile_ = false;
    float32_t wind_profile_velocity_ = 0.0f;
    float32_t wind_profile_height_ = 10.0f;
    float32_t wind_profile_alpha_ = 0.143f;
    Face wind_direction_ = Face::Y_MIN;
};
