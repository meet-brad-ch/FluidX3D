#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <cmath>

extern Units units; // global units object from lbm.cpp

// Oscillating wave maker on an X or Y inlet face (SURFACE and EQUILIBRIUM_BOUNDARIES extensions).
// Call initialize() once, then update(lbm.get_t()) in the run loop.
class WaveBoundary {
public:
    explicit WaveBoundary(LBM& lbm) : lbm_(lbm) {}

    /// The wave's amplitude and frequency; its peak velocity is 2*pi*frequency*amplitude.
    WaveBoundary& set_wave(Length amplitude, Frequency frequency) {
        frequency_hz_ = frequency.si();
        peak_velocity_mps_ = amplitude.si() * 2.0f * pif * frequency.si();
        return *this;
    }

    WaveBoundary& set_inlet_face(Face face) { // default Y_MIN
        inlet_face_ = face;
        return *this;
    }

    // vertical velocity as a fraction of the horizontal one (default 0.5; smaller for shallow water)
    WaveBoundary& set_vertical_factor(float32_t factor) {
        vertical_factor_ = factor;
        return *this;
    }

    // marks the interior of the inlet face as equilibrium boundary (TYPE_E)
    void initialize() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        u_wave_lbm_ = units.u(peak_velocity_mps_);
        omega_ = 2.0f * pif * frequency_hz_;
        dt_si_ = units.si_t(1ull); // SI seconds per LBM time step (units.t() would round to whole time steps)
        if (inlet_face_ == Face::Z_MIN || inlet_face_ == Face::Z_MAX) {
            print_warning("WaveBoundary: Z inlet faces are not supported; the wave is not driven");
        }

        parallel_for(lbm_.get_N(), [&](uint64_t n) {
            uint32_t x = 0, y = 0, z = 0;
            lbm_.coordinates(n, x, y, z);
            if (!boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, inlet_face_)) return;
            const bool inner_x = x > 0u && x < Nx - 1u;
            const bool inner_y = y > 0u && y < Ny - 1u;
            const bool inner_z = z > 0u && z < Nz - 1u;
            bool inner = false; // not on the edges the inlet face shares with other faces
            switch (inlet_face_) {
                case Face::X_MIN: case Face::X_MAX: inner = inner_y && inner_z; break;
                case Face::Y_MIN: case Face::Y_MAX: inner = inner_x && inner_z; break;
                case Face::Z_MIN: case Face::Z_MAX: inner = inner_x && inner_y; break;
            }
            if (inner) lbm_.flags[n] = TYPE_E;
        });

        initialized_ = true;
    }

    // set the inlet velocities for this LBM time step (reads and writes the velocity field on the device)
    void update(uint64_t timestep) {
        if (!initialized_) return;

        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        const float32_t t_si = (float32_t)timestep * dt_si_;
        const float32_t u_primary = u_wave_lbm_ * sinf(omega_ * t_si);
        const float32_t u_vertical = vertical_factor_ * u_wave_lbm_ * cosf(omega_ * t_si);

        lbm_.u.read_from_device();

        // interior cells of the inlet face: primary velocity along the face normal, plus the vertical component
        const bool is_min_face = inlet_face_ == Face::X_MIN || inlet_face_ == Face::Y_MIN;
        const float32_t u_normal = is_min_face ? u_primary : -u_primary;
        for (uint32_t z = 1u; z < Nz - 1u; z++) {
            if (inlet_face_ == Face::Y_MIN || inlet_face_ == Face::Y_MAX) {
                const uint32_t y = is_min_face ? 0u : Ny - 1u;
                for (uint32_t x = 1u; x < Nx - 1u; x++) {
                    const uint64_t n = lbm_.index(x, y, z);
                    lbm_.u.y[n] = u_normal;
                    lbm_.u.z[n] = u_vertical;
                }
            } else if (inlet_face_ == Face::X_MIN || inlet_face_ == Face::X_MAX) {
                const uint32_t x = is_min_face ? 0u : Nx - 1u;
                for (uint32_t y = 1u; y < Ny - 1u; y++) {
                    const uint64_t n = lbm_.index(x, y, z);
                    lbm_.u.x[n] = u_normal;
                    lbm_.u.z[n] = u_vertical;
                }
            }
        }

        lbm_.u.write_to_device();
    }

private:
    LBM& lbm_;

    float32_t frequency_hz_ = 0.5f;
    float32_t peak_velocity_mps_ = 0.0f;

    float32_t u_wave_lbm_ = 0.0f;
    float32_t omega_ = 0.0f; // rad/s
    float32_t dt_si_ = 0.0f; // s per LBM time step

    Face inlet_face_ = Face::Y_MIN;
    float32_t vertical_factor_ = 0.5f;

    bool initialized_ = false;
};
