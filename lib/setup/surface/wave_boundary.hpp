#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/unit_scale.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/simulation/runner.hpp"
#include "lbm.hpp"
#include <cmath>
#include <optional>

#ifndef SURFACE
#error "setup/surface/wave_boundary.hpp needs SURFACE in defines.hpp"
#endif // SURFACE

/// @brief Oscillating wave maker on an X or Y inlet face (SURFACE and EQUILIBRIUM_BOUNDARIES extensions), owned by
/// the simulation (Simulation::wave_maker()).
///
/// initialize() marks the inlet and drives it from then on, at every update interval of simulated time.
/// @code
/// sim.wave_maker().set_wave(0.21_m, 0.78_Hz).set_inlet_face(Face::Y_MIN).initialize();
/// @endcode
class WaveBoundary {
public:
    /// @param lbm        the free surface LBM
    /// @param unit_scale the simulation's unit scale
    /// @param runner     the simulation's runner, which calls update()
    WaveBoundary(LBM& lbm, const UnitScale& unit_scale, Runner& runner) : lbm_(lbm), units_(unit_scale), runner_(runner) {}

    /// @brief The wave's amplitude and frequency; its peak velocity is 2*pi*frequency*amplitude.
    /// @param amplitude the amplitude
    /// @param frequency the frequency
    /// @return this wave maker
    WaveBoundary& set_wave(Length amplitude, Frequency frequency) {
        frequency_ = frequency;
        peak_velocity_ = (amplitude * frequency) * (2.0f * pif);
        return *this;
    }

    /// @brief The face the waves come from (default Y_MIN).
    /// @param face the face
    /// @return this wave maker
    WaveBoundary& set_inlet_face(Face face) {
        inlet_face_ = face;
        return *this;
    }

    /// @brief The vertical velocity as a fraction of the horizontal one (default 0.5; smaller for shallow water).
    /// @param factor the fraction
    /// @return this wave maker
    WaveBoundary& set_vertical_factor(float32_t factor) {
        vertical_factor_ = factor;
        return *this;
    }

    /// @brief The simulated time between updates of the inlet velocities (default: a sixteenth of the wave's period).
    /// @param interval the interval
    /// @return this wave maker
    WaveBoundary& set_update_interval(Duration interval) {
        update_interval_ = interval;
        return *this;
    }

    /// Marks the interior of the inlet face as equilibrium boundary (TYPE_E) and schedules the updates.
    void initialize() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        u_wave_lbm_ = units_.velocity(peak_velocity_);
        omega_ = 2.0f * pif * frequency_.si();
        if(inlet_face_ == Face::Z_MIN || inlet_face_ == Face::Z_MAX) {
            print_warning("WaveBoundary: Z inlet faces are not supported; the wave is not driven");
        }

        parallel_for(lbm_.get_N(), [&](uint64_t n) {
            uint32_t x = 0, y = 0, z = 0;
            lbm_.coordinates(n, x, y, z);
            if(!boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, inlet_face_)) return;
            const bool inner_x = x > 0u && x < Nx - 1u;
            const bool inner_y = y > 0u && y < Ny - 1u;
            const bool inner_z = z > 0u && z < Nz - 1u;
            bool inner = false; // not on the edges the inlet face shares with other faces
            switch(inlet_face_) {
                case Face::X_MIN: case Face::X_MAX: inner = inner_y && inner_z; break;
                case Face::Y_MIN: case Face::Y_MAX: inner = inner_x && inner_z; break;
                case Face::Z_MIN: case Face::Z_MAX: inner = inner_x && inner_y; break;
            }
            if(inner) lbm_.flags[n] = TYPE_E;
        });

        initialized_ = true;
        const Duration interval = update_interval_ ? *update_interval_ : Duration::from_si(1.0f / (16.0f * frequency_.si()));
        runner_.every(interval, [this](Duration time) { update(time); });
    }

    /// @brief Sets the inlet velocities for a simulated time since the start (reads and writes the velocity field on
    /// the device).
    /// @param time the simulated time
    void update(Duration time) {
        if(!initialized_) return;

        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        const float32_t t_si = time.si();
        const float32_t u_primary = u_wave_lbm_ * sinf(omega_ * t_si);
        const float32_t u_vertical = vertical_factor_ * u_wave_lbm_ * cosf(omega_ * t_si);

        lbm_.u.read_from_device();

        // interior cells of the inlet face: primary velocity along the face normal, plus the vertical component
        const bool is_min_face = inlet_face_ == Face::X_MIN || inlet_face_ == Face::Y_MIN;
        const float32_t u_normal = is_min_face ? u_primary : -u_primary;
        for(uint32_t z = 1u; z < Nz - 1u; z++) {
            if(inlet_face_ == Face::Y_MIN || inlet_face_ == Face::Y_MAX) {
                const uint32_t y = is_min_face ? 0u : Ny - 1u;
                for(uint32_t x = 1u; x < Nx - 1u; x++) {
                    const uint64_t n = lbm_.index(x, y, z);
                    lbm_.u.y[n] = u_normal;
                    lbm_.u.z[n] = u_vertical;
                }
            } else if(inlet_face_ == Face::X_MIN || inlet_face_ == Face::X_MAX) {
                const uint32_t x = is_min_face ? 0u : Nx - 1u;
                for(uint32_t y = 1u; y < Ny - 1u; y++) {
                    const uint64_t n = lbm_.index(x, y, z);
                    lbm_.u.x[n] = u_normal;
                    lbm_.u.z[n] = u_vertical;
                }
            }
        }

        lbm_.u.write_to_device();
    }

private:
    LBM& lbm_;        ///< the free surface LBM
    UnitScale units_; ///< the simulation's unit scale
    Runner& runner_;  ///< the simulation's runner

    Frequency frequency_ = Frequency::from_si(0.5f); ///< the wave's frequency
    Speed peak_velocity_;                            ///< the wave's peak velocity
    std::optional<Duration> update_interval_;        ///< set_update_interval()

    float32_t u_wave_lbm_ = 0.0f; ///< the peak velocity in lattice units
    float32_t omega_ = 0.0f;      ///< the angular frequency in rad/s

    Face inlet_face_ = Face::Y_MIN;    ///< the face the waves come from
    float32_t vertical_factor_ = 0.5f; ///< the vertical velocity as a fraction of the horizontal one

    bool initialized_ = false; ///< whether initialize() was called
};
