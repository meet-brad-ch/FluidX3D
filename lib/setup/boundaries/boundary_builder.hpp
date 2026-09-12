#pragma once

#include "core/types.hpp"
#include "core/boundary_utils.hpp"
#include "boundaries/boundary_flags.hpp"
#include "lbm.hpp"
#include "units.hpp"

extern Units units; // global units object from lbm.cpp

// Domain boundaries and initial velocity in SI units; apply() writes them to the grid.
// Order per cell: floor, ceiling, walls, open faces (TYPE_E), then velocities in non-solid cells.
class BoundaryBuilder {
public:
    explicit BoundaryBuilder(LBM& lbm) : lbm_(lbm) {}

    BoundaryBuilder& set_solid_floor(uchar flag = TYPE_S) { // z = 0
        floor_flag_ = flag;
        apply_floor_ = true;
        return *this;
    }

    BoundaryBuilder& set_solid_ceiling(uchar flag = TYPE_S) { // z = Nz-1
        ceiling_flag_ = flag;
        apply_ceiling_ = true;
        return *this;
    }

    BoundaryBuilder& set_solid_walls(uchar flag = TYPE_S) { // x and y faces
        walls_flag_ = flag;
        apply_walls_ = true;
        return *this;
    }

    // all faces except the floor open (wind tunnel)
    BoundaryBuilder& set_open_boundaries() {
        open_x_ = open_y_ = open_z_max_ = true;
        return *this;
    }

    BoundaryBuilder& set_all_open() {
        open_x_ = open_y_ = open_z_min_ = open_z_max_ = true;
        return *this;
    }

    // all walls solid; moving_face slides at velocity_mps along its first tangential axis (X faces: +y, Y and Z faces: +x)
    BoundaryBuilder& preset_lid_driven_cavity(Face moving_face, float32_t velocity_mps) {
        set_solid_floor();
        set_solid_ceiling();
        set_solid_walls();
        has_moving_lid_ = true;
        lid_face_ = moving_face;
        lid_velocity_ = velocity_mps;
        return *this;
    }

    // initial velocity in m/s in all non-solid cells
    BoundaryBuilder& initialize_velocity_x(float32_t si_velocity) {
        init_u_x_ = si_velocity;
        has_init_u_x_ = true;
        return *this;
    }

    BoundaryBuilder& initialize_velocity_y(float32_t si_velocity) {
        init_u_y_ = si_velocity;
        has_init_u_y_ = true;
        return *this;
    }

    BoundaryBuilder& initialize_velocity_z(float32_t si_velocity) {
        init_u_z_ = si_velocity;
        has_init_u_z_ = true;
        return *this;
    }

    BoundaryBuilder& initialize_velocity(float32_t si_vx, float32_t si_vy, float32_t si_vz) {
        init_u_x_ = si_vx; has_init_u_x_ = true;
        init_u_y_ = si_vy; has_init_u_y_ = true;
        init_u_z_ = si_vz; has_init_u_z_ = true;
        return *this;
    }

    // atmospheric boundary layer u(z) = u_ref*(z/z_ref)^alpha, in m/s and m
    // (alpha: 0.10 sea, 0.143 open terrain, 0.20 suburbs, 0.25-0.40 urban)
    BoundaryBuilder& set_wind_profile_power_law(float32_t si_reference_velocity,
                                                 float32_t si_reference_height_m,
                                                 float32_t alpha = 0.143f) {
        wind_profile_velocity_ = si_reference_velocity;
        wind_profile_height_ = si_reference_height_m;
        wind_profile_alpha_ = alpha;
        has_wind_profile_ = true;
        return *this;
    }

    // face the wind comes from (default Y_MIN)
    BoundaryBuilder& set_wind_direction(Face direction) {
        wind_direction_ = direction;
        return *this;
    }

    void apply() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        const float32_t lbm_init_u_x = has_init_u_x_ ? units.u(init_u_x_) : 0.0f;
        const float32_t lbm_init_u_y = has_init_u_y_ ? units.u(init_u_y_) : 0.0f;
        const float32_t lbm_init_u_z = has_init_u_z_ ? units.u(init_u_z_) : 0.0f;
        const float32_t lbm_lid_velocity = has_moving_lid_ ? units.u(lid_velocity_) : 0.0f;
        const float32_t lbm_wind_ref_velocity = has_wind_profile_ ? units.u(wind_profile_velocity_) : 0.0f;
        const float32_t lbm_wind_ref_height = has_wind_profile_ ? units.x(wind_profile_height_) : 1.0f;

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
                if (has_init_u_x_) lbm_.u.x[n] = lbm_init_u_x;
                if (has_init_u_y_) lbm_.u.y[n] = lbm_init_u_y;
                if (has_init_u_z_) lbm_.u.z[n] = lbm_init_u_z;
            }

            if (has_wind_profile_ && !(lbm_.flags[n] & TYPE_S) && z > 0u) {
                const float32_t height_ratio = (float32_t)z / lbm_wind_ref_height;
                const float32_t wind_velocity = lbm_wind_ref_velocity * pow(height_ratio, wind_profile_alpha_);
                switch (wind_direction_) {
                    case Face::X_MIN: lbm_.u.x[n] = wind_velocity; break;
                    case Face::X_MAX: lbm_.u.x[n] = -wind_velocity; break;
                    case Face::Y_MIN: lbm_.u.y[n] = wind_velocity; break;
                    case Face::Y_MAX: lbm_.u.y[n] = -wind_velocity; break;
                    case Face::Z_MIN: lbm_.u.z[n] = wind_velocity; break;
                    case Face::Z_MAX: lbm_.u.z[n] = -wind_velocity; break;
                }
            }

            if (has_moving_lid_ && boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, lid_face_)) {
                if (lid_face_ == Face::X_MIN || lid_face_ == Face::X_MAX) {
                    lbm_.u.y[n] = lbm_lid_velocity;
                } else {
                    lbm_.u.x[n] = lbm_lid_velocity;
                }
            }
        });
    }

private:
    LBM& lbm_;

    uchar floor_flag_ = TYPE_S;
    uchar ceiling_flag_ = TYPE_S;
    uchar walls_flag_ = TYPE_S;
    bool apply_floor_ = false;
    bool apply_ceiling_ = false;
    bool apply_walls_ = false;

    bool has_moving_lid_ = false;
    Face lid_face_ = Face::Z_MAX;
    float32_t lid_velocity_ = 0.0f; // m/s

    bool open_x_ = false; // both x faces
    bool open_y_ = false; // both y faces
    bool open_z_min_ = false;
    bool open_z_max_ = false;

    bool has_init_u_x_ = false;
    bool has_init_u_y_ = false;
    bool has_init_u_z_ = false;
    float32_t init_u_x_ = 0.0f; // m/s
    float32_t init_u_y_ = 0.0f;
    float32_t init_u_z_ = 0.0f;

    bool has_wind_profile_ = false;
    float32_t wind_profile_velocity_ = 0.0f; // m/s
    float32_t wind_profile_height_ = 10.0f;  // m
    float32_t wind_profile_alpha_ = 0.143f;
    Face wind_direction_ = Face::Y_MIN;
};
