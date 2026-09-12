#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <optional>

extern Units units; // global units object from lbm.cpp

/// @brief Domain boundaries and the initial velocity in physical units; apply() writes them to the grid.
///
/// Speeds and heights are converted with the global units (SimulationSetup::configure_units() first).
/// Order per cell: floor, ceiling, walls, open faces (TYPE_E), then the velocities of the non-solid cells.
/// @code
/// BoundaryBuilder(lbm).set_solid_floor().set_open_boundaries().initialize_velocity_y(10.0_mps).apply();
/// @endcode
class BoundaryBuilder {
public:
    /// @param lbm the LBM whose flags and velocities are set
    explicit BoundaryBuilder(LBM& lbm) : lbm_(lbm) {}

    /// Solid floor (z = 0) with this flag.
    BoundaryBuilder& set_solid_floor(uchar flag = TYPE_S) {
        floor_flag_ = flag;
        apply_floor_ = true;
        return *this;
    }

    /// Solid ceiling (z = Nz-1) with this flag.
    BoundaryBuilder& set_solid_ceiling(uchar flag = TYPE_S) {
        ceiling_flag_ = flag;
        apply_ceiling_ = true;
        return *this;
    }

    /// Solid walls on the four x and y faces with this flag.
    BoundaryBuilder& set_solid_walls(uchar flag = TYPE_S) {
        walls_flag_ = flag;
        apply_walls_ = true;
        return *this;
    }

    /// All faces except the floor open (wind tunnel).
    BoundaryBuilder& set_open_boundaries() {
        open_x_ = open_y_ = open_z_max_ = true;
        return *this;
    }

    /// All six faces open.
    BoundaryBuilder& set_all_open() {
        open_x_ = open_y_ = open_z_min_ = open_z_max_ = true;
        return *this;
    }

    /// All walls solid; moving_face slides at this speed along its first tangential axis (X faces: +y, Y and Z faces: +x).
    BoundaryBuilder& preset_lid_driven_cavity(Face moving_face, Speed speed) {
        set_solid_floor();
        set_solid_ceiling();
        set_solid_walls();
        lid_ = Lid{ moving_face, speed };
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
    BoundaryBuilder& initialize_velocity(Speed ux, Speed uy, Speed uz) {
        init_u_x_ = ux;
        init_u_y_ = uy;
        init_u_z_ = uz;
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

    /// Writes the boundaries and velocities to the grid.
    void apply() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        const float32_t lbm_init_u_x = init_u_x_ ? units.u(init_u_x_->si()) : 0.0f;
        const float32_t lbm_init_u_y = init_u_y_ ? units.u(init_u_y_->si()) : 0.0f;
        const float32_t lbm_init_u_z = init_u_z_ ? units.u(init_u_z_->si()) : 0.0f;
        const float32_t lbm_lid_velocity = lid_ ? units.u(lid_->speed.si()) : 0.0f;
        const float32_t lbm_wind_ref_velocity = wind_ ? units.u(wind_->reference_speed.si()) : 0.0f;
        const float32_t lbm_wind_ref_height = wind_ ? units.x(wind_->reference_height.si()) : 1.0f;

        parallel_for(lbm_.get_N(), [&](uint64_t n) {
            uint32_t x = 0u, y = 0u, z = 0u;
            lbm_.coordinates(n, x, y, z);

            if (apply_floor_ && z == 0u) lbm_.flags[n] = floor_flag_;
            if (apply_ceiling_ && z == Nz - 1u) lbm_.flags[n] = ceiling_flag_;
            if (apply_walls_ && (x == 0u || x == Nx - 1u || y == 0u || y == Ny - 1u)) lbm_.flags[n] = walls_flag_;

            const bool is_open_boundary = (open_x_ && (x == 0u || x == Nx - 1u)) ||
                                          (open_y_ && (y == 0u || y == Ny - 1u)) ||
                                          (open_z_min_ && z == 0u) ||
                                          (open_z_max_ && z == Nz - 1u);
            if (is_open_boundary) lbm_.flags[n] = TYPE_E;

            if (!(lbm_.flags[n] & TYPE_S)) {
                if (init_u_x_) lbm_.u.x[n] = lbm_init_u_x;
                if (init_u_y_) lbm_.u.y[n] = lbm_init_u_y;
                if (init_u_z_) lbm_.u.z[n] = lbm_init_u_z;
            }

            if (wind_ && !(lbm_.flags[n] & TYPE_S) && z > 0u) {
                const float32_t height_ratio = (float32_t)z / lbm_wind_ref_height;
                const float32_t wind_velocity = lbm_wind_ref_velocity * pow(height_ratio, wind_->alpha);
                switch (wind_direction_) {
                    case Face::X_MIN: lbm_.u.x[n] = wind_velocity; break;
                    case Face::X_MAX: lbm_.u.x[n] = -wind_velocity; break;
                    case Face::Y_MIN: lbm_.u.y[n] = wind_velocity; break;
                    case Face::Y_MAX: lbm_.u.y[n] = -wind_velocity; break;
                    case Face::Z_MIN: lbm_.u.z[n] = wind_velocity; break;
                    case Face::Z_MAX: lbm_.u.z[n] = -wind_velocity; break;
                }
            }

            if (lid_ && boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, lid_->face)) {
                if (lid_->face == Face::X_MIN || lid_->face == Face::X_MAX) {
                    lbm_.u.y[n] = lbm_lid_velocity;
                } else {
                    lbm_.u.x[n] = lbm_lid_velocity;
                }
            }
        });
    }

private:
    struct Lid { Face face; Speed speed; };                                        ///< the moving wall of a lid-driven cavity
    struct WindProfile { Speed reference_speed; Length reference_height; float32_t alpha; }; ///< power-law wind

    LBM& lbm_;

    uchar floor_flag_ = TYPE_S;
    uchar ceiling_flag_ = TYPE_S;
    uchar walls_flag_ = TYPE_S;
    bool apply_floor_ = false;
    bool apply_ceiling_ = false;
    bool apply_walls_ = false;

    bool open_x_ = false; ///< both x faces
    bool open_y_ = false; ///< both y faces
    bool open_z_min_ = false;
    bool open_z_max_ = false;

    std::optional<Speed> init_u_x_, init_u_y_, init_u_z_;
    std::optional<Lid> lid_;
    std::optional<WindProfile> wind_;
    Face wind_direction_ = Face::Y_MIN;
};
