#pragma once

#include "core/types.hpp"
#include "boundaries/boundary_flags.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <cmath>

extern Units units;  // Global units object from lbm.cpp

/**
 * @file wave_boundary.hpp
 * @brief Time-varying wave boundary conditions for FluidX3D simulations
 *
 * Provides a readable API for setting up oscillating wave boundary conditions
 * for breaking wave, wave flume, and coastal simulations.
 *
 * @note Requires SURFACE and EQUILIBRIUM_BOUNDARIES extensions
 */

/**
 * @class WaveBoundary
 * @brief Fluent API for wave generation boundary conditions
 *
 * This class simplifies setting up wave boundaries including:
 * - Sinusoidal wave motion at domain boundary
 * - Time-varying velocity updates
 * - SI-unit based configuration
 *
 * @par Example (breaking waves):
 * @code
 * WaveBoundary wave(lbm);
 * wave
 *     .set_wave_parameters_si(0.05f, 0.5f)  // 5cm amplitude, 0.5 Hz
 *     .set_inlet_face(Face::Y_MIN)
 *     .initialize();
 *
 * lbm.run(0u);
 * while (true) {
 *     wave.update(lbm.get_t());
 *     lbm.run(100u);
 * }
 * @endcode
 */
class WaveBoundary {
public:
    /**
     * @brief Construct WaveBoundary for an LBM simulation
     * @param lbm Reference to the LBM simulation object
     */
    explicit WaveBoundary(LBM& lbm) : lbm_(lbm) {}

    // ========================================================================
    // Wave Configuration (SI Units)
    // ========================================================================

    /**
     * @brief Set wave parameters in SI units
     * @param amplitude_m Wave amplitude in meters
     * @param frequency_hz Wave frequency in Hertz
     * @return Reference for method chaining
     *
     * @note Calculates peak velocity as: u = 2 * pi * frequency * amplitude
     */
    WaveBoundary& set_wave_parameters_si(float32_t amplitude_m, float32_t frequency_hz) {
        amplitude_m_ = amplitude_m;
        frequency_hz_ = frequency_hz;
        peak_velocity_mps_ = amplitude_m * 2.0f * pif * frequency_hz;
        return *this;
    }

    /**
     * @brief Set wave velocity directly in SI units
     * @param velocity_mps Peak wave velocity in m/s
     * @param frequency_hz Wave frequency in Hertz
     * @return Reference for method chaining
     */
    WaveBoundary& set_wave_velocity_si(float32_t velocity_mps, float32_t frequency_hz) {
        peak_velocity_mps_ = velocity_mps;
        frequency_hz_ = frequency_hz;
        amplitude_m_ = velocity_mps / (2.0f * pif * frequency_hz);
        return *this;
    }

    /**
     * @brief Set inlet face for wave generation
     * @param face Which face generates waves (default: Y_MIN)
     * @return Reference for method chaining
     */
    WaveBoundary& set_inlet_face(Face face) {
        inlet_face_ = face;
        return *this;
    }

    /**
     * @brief Set vertical oscillation factor
     * @param factor Ratio of vertical to horizontal velocity (default: 0.5)
     * @return Reference for method chaining
     *
     * @note For deep water waves, use 0.5; for shallow waves, use smaller values
     */
    WaveBoundary& set_vertical_factor(float32_t factor) {
        vertical_factor_ = factor;
        return *this;
    }

    /**
     * @brief Set phase offset for wave
     * @param phase_rad Phase offset in radians (default: 0)
     * @return Reference for method chaining
     */
    WaveBoundary& set_phase(float32_t phase_rad) {
        phase_offset_ = phase_rad;
        return *this;
    }

    // ========================================================================
    // Initialization
    // ========================================================================

    /**
     * @brief Initialize wave boundary condition
     *
     * Sets up equilibrium boundary cells at the inlet face.
     * Must be called after LBM is created but before simulation runs.
     */
    void initialize() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        // Convert SI parameters to LBM units
        u_wave_lbm_ = units.u(peak_velocity_mps_);
        omega_ = 2.0f * pif * frequency_hz_;
        dt_si_ = units.si_t(1ull);  // SI seconds per LBM timestep (units.t() would round to whole time steps)
        if (inlet_face_ == Face::Z_MIN || inlet_face_ == Face::Z_MAX) {
            print_warning("WaveBoundary: Z inlet faces are not supported; the wave is not driven");
        }

        // Set inlet cells as equilibrium boundary
        parallel_for(lbm_.get_N(), [&](uint64_t n) {
            uint32_t x = 0, y = 0, z = 0;
            lbm_.coordinates(n, x, y, z);

            bool is_inlet = false;
            switch (inlet_face_) {
                case Face::X_MIN: is_inlet = (x == 0u); break;
                case Face::X_MAX: is_inlet = (x == Nx - 1u); break;
                case Face::Y_MIN: is_inlet = (y == 0u); break;
                case Face::Y_MAX: is_inlet = (y == Ny - 1u); break;
                case Face::Z_MIN: is_inlet = (z == 0u); break;
                case Face::Z_MAX: is_inlet = (z == Nz - 1u); break;
            }

            // Set as equilibrium boundary, but not at corners/edges
            if (is_inlet) {
                bool at_edge = (x == 0u || x == Nx - 1u) ||
                               (y == 0u || y == Ny - 1u) ||
                               (z == 0u || z == Nz - 1u);
                bool at_inlet_face = false;
                switch (inlet_face_) {
                    case Face::X_MIN: case Face::X_MAX: at_inlet_face = true; break;
                    case Face::Y_MIN: case Face::Y_MAX: at_inlet_face = true; break;
                    case Face::Z_MIN: case Face::Z_MAX: at_inlet_face = true; break;
                }
                // Only exclude the 4 edges that are not the inlet face edges
                if (at_inlet_face) {
                    // For Y_MIN face, exclude x=0, x=Nx-1, z=0, z=Nz-1 edges
                    if (inlet_face_ == Face::Y_MIN || inlet_face_ == Face::Y_MAX) {
                        if (x > 0u && x < Nx - 1u && z > 0u && z < Nz - 1u) {
                            lbm_.flags[n] = TYPE_E;
                        }
                    } else if (inlet_face_ == Face::X_MIN || inlet_face_ == Face::X_MAX) {
                        if (y > 0u && y < Ny - 1u && z > 0u && z < Nz - 1u) {
                            lbm_.flags[n] = TYPE_E;
                        }
                    } else {
                        if (x > 0u && x < Nx - 1u && y > 0u && y < Ny - 1u) {
                            lbm_.flags[n] = TYPE_E;
                        }
                    }
                }
            }
        });

        initialized_ = true;
    }

    // ========================================================================
    // Runtime Update
    // ========================================================================

    /**
     * @brief Update wave velocities at current timestep
     * @param timestep Current LBM timestep
     *
     * Call this in the simulation loop to update boundary velocities.
     * Automatically handles device transfer.
     */
    void update(uint64_t timestep) {
        if (!initialized_) return;

        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        // Calculate current time in SI seconds
        const float32_t t_si = (float32_t)timestep * dt_si_;

        // Calculate wave velocities
        const float32_t u_primary = u_wave_lbm_ * sinf(omega_ * t_si + phase_offset_);
        const float32_t u_vertical = vertical_factor_ * u_wave_lbm_ * cosf(omega_ * t_si + phase_offset_);

        // Read velocity field from device
        lbm_.u.read_from_device();

        // Update the inlet face's interior cells: primary velocity along the face normal, plus the vertical component
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

        // Write updated velocities back to device
        lbm_.u.write_to_device();
    }

    // ========================================================================
    // Accessors
    // ========================================================================

    /**
     * @brief Check if wave boundary is initialized
     * @return True if initialize() has been called
     */
    bool is_initialized() const { return initialized_; }

    /**
     * @brief Get current wave velocity in LBM units
     * @param timestep Current LBM timestep
     * @return Primary wave velocity component
     */
    float32_t get_velocity_lbm(uint64_t timestep) const {
        const float32_t t_si = (float32_t)timestep * dt_si_;
        return u_wave_lbm_ * sinf(omega_ * t_si + phase_offset_);
    }

    /**
     * @brief Get peak wave velocity in SI units
     */
    float32_t get_peak_velocity_si() const { return peak_velocity_mps_; }

    /**
     * @brief Get wave frequency in Hertz
     */
    float32_t get_frequency_hz() const { return frequency_hz_; }

private:
    LBM& lbm_;

    // Wave parameters (SI)
    float32_t amplitude_m_ = 0.05f;
    float32_t frequency_hz_ = 0.5f;
    float32_t peak_velocity_mps_ = 0.0f;

    // Wave parameters (LBM)
    float32_t u_wave_lbm_ = 0.0f;
    float32_t omega_ = 0.0f;
    float32_t dt_si_ = 0.0f;

    // Configuration
    Face inlet_face_ = Face::Y_MIN;
    float32_t vertical_factor_ = 0.5f;
    float32_t phase_offset_ = 0.0f;

    bool initialized_ = false;
};
