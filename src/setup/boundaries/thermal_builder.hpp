#pragma once

#include "core/types.hpp"
#include "core/boundary_utils.hpp"
#include "boundaries/boundary_flags.hpp"
#include "boundaries/thermal_utils.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <vector>
#include <thread>

extern Units units;  // Global units object from lbm.cpp

/**
 * @file thermal_builder.hpp
 * @brief Fluent thermal boundary condition builder for FluidX3D simulations
 *
 * Provides a readable, SI-unit based API for setting up thermal boundary
 * conditions and configuring Boussinesq natural convection simulations.
 *
 * @note Requires TEMPERATURE extension enabled in defines.hpp
 */

/**
 * @class ThermalBuilder
 * @brief Fluent API for thermal boundary conditions
 *
 * This class simplifies setting up thermal simulations including:
 * - Hot and cold wall boundaries
 * - Thermal diffusivity and expansion coefficients
 * - Rayleigh number based configuration
 * - Boussinesq buoyancy approximation
 *
 * @par Example (Rayleigh-Benard convection):
 * @code
 * ThermalBuilder thermal(lbm);
 * thermal
 *     .configure_rayleigh(1e5f, 0.71f)         // Ra=100,000, Pr=0.71
 *     .set_hot_wall(Face::Z_MIN, 350.0f)        // Hot bottom (350K)
 *     .set_cold_wall(Face::Z_MAX, 300.0f)       // Cold top (300K)
 *     .enable_boussinesq_buoyancy()
 *     .apply();
 * @endcode
 *
 * @par Example (Natural convection cavity):
 * @code
 * ThermalBuilder thermal(lbm);
 * thermal
 *     .set_thermal_diffusivity(2.2e-5f)        // Air thermal diffusivity
 *     .set_thermal_expansion(0.0034f)          // Air at ~300K
 *     .set_hot_wall(Face::X_MIN, 320.0f)        // Hot left wall
 *     .set_cold_wall(Face::X_MAX, 280.0f)       // Cold right wall
 *     .set_gravity(-9.81f)
 *     .apply();
 * @endcode
 */
class ThermalBuilder {
public:
    /**
     * @brief Construct ThermalBuilder for an LBM simulation
     * @param lbm Reference to the LBM simulation object
     */
    explicit ThermalBuilder(LBM& lbm) : lbm_(lbm) {}

    // ========================================================================
    // Thermal Wall Boundaries (SI units - Kelvin)
    // ========================================================================

    /**
     * @brief Set a hot wall temperature boundary in Kelvin
     * @param face Which face is the hot wall
     * @param temperature_K Temperature in Kelvin
     * @return Reference for method chaining
     *
     * @note Uses TYPE_T flag for temperature boundary
     * @note Temperatures are converted to dimensionless LBM units internally
     */
    ThermalBuilder& set_hot_wall(Face face, float32_t temperature_K) {
        hot_face_ = face;
        hot_temperature_K_ = temperature_K;
        has_hot_wall_ = true;
        use_si_temperatures_ = true;
        return *this;
    }

    /**
     * @brief Set a cold wall temperature boundary in Kelvin
     * @param face Which face is the cold wall
     * @param temperature_K Temperature in Kelvin
     * @return Reference for method chaining
     *
     * @note Uses TYPE_T flag for temperature boundary
     * @note Temperatures are converted to dimensionless LBM units internally
     */
    ThermalBuilder& set_cold_wall(Face face, float32_t temperature_K) {
        cold_face_ = face;
        cold_temperature_K_ = temperature_K;
        has_cold_wall_ = true;
        use_si_temperatures_ = true;
        return *this;
    }

    /**
     * @brief Set temperature on a specific face in Kelvin
     * @param face Which face to set temperature
     * @param temperature_K Temperature in Kelvin
     * @return Reference for method chaining
     */
    ThermalBuilder& set_wall_temperature(Face face, float32_t temperature_K) {
        WallTemp wt;
        wt.face = face;
        wt.temperature_K = temperature_K;
        wall_temps_.push_back(wt);
        use_si_temperatures_ = true;
        return *this;
    }

    /**
     * @brief Set reference temperature for dimensionless conversion
     * @param T_ref_K Reference temperature in Kelvin (default: average of hot/cold)
     * @return Reference for method chaining
     *
     * The reference temperature is used to convert SI temperatures to
     * dimensionless LBM units: T_lbm = 1 + (T_K - T_ref) / delta_T
     */
    ThermalBuilder& set_reference_temperature(float32_t T_ref_K) {
        T_ref_K_ = T_ref_K;
        has_reference_temp_ = true;
        return *this;
    }

    // ========================================================================
    // Thermal Properties (LBM units)
    // ========================================================================

    /**
     * @brief Set thermal diffusivity (alpha) in LBM units
     * @param alpha Thermal diffusivity
     * @return Reference for method chaining
     *
     * @note In LBM, alpha is often set equal to nu for Pr=1
     */
    ThermalBuilder& set_thermal_diffusivity_lbm(float32_t alpha) {
        alpha_ = alpha;
        return *this;
    }

    /**
     * @brief Set thermal expansion coefficient (beta) in LBM units
     * @param beta Thermal expansion coefficient for Boussinesq approximation
     * @return Reference for method chaining
     *
     * @note Typical values: 0.0005 to 0.001 for stable simulations
     */
    ThermalBuilder& set_thermal_expansion_lbm(float32_t beta) {
        beta_ = beta;
        return *this;
    }

    // ========================================================================
    // Dimensionless Configuration (Rayleigh/Prandtl)
    // ========================================================================

    /**
     * @brief Configure thermal simulation using Rayleigh and Prandtl numbers
     * @param Ra Rayleigh number (dimensionless)
     * @param Pr Prandtl number (default: 0.71 for air)
     * @return Reference for method chaining
     *
     * Rayleigh number: Ra = g * beta * dT * L³ / (nu * alpha)
     * Prandtl number: Pr = nu / alpha
     *
     * This method calculates appropriate alpha and beta values for
     * the simulation based on the desired Ra and Pr numbers.
     *
     * @note The domain's Z-dimension is used as the characteristic length L
     *
     * @par Example:
     * @code
     * thermal.configure_rayleigh(1e5f, 0.71f)  // Ra=100,000, Pr=0.71 (air)
     *        .set_hot_wall(Face::Z_MIN, 1.75f)
     *        .set_cold_wall(Face::Z_MAX, 0.25f)
     *        .apply();
     * @endcode
     */
    ThermalBuilder& configure_rayleigh(float32_t Ra, float32_t Pr = 0.71f) {
        rayleigh_ = Ra;
        prandtl_ = Pr;
        use_rayleigh_config_ = true;
        return *this;
    }

    // ========================================================================
    // Buoyancy and Gravity
    // ========================================================================

    /**
     * @brief Enable Boussinesq buoyancy approximation
     * @return Reference for method chaining
     *
     * The Boussinesq approximation models buoyancy-driven flow where
     * density variations are small and only affect the body force term.
     *
     * @note Requires VOLUME_FORCE extension in defines.hpp
     */
    ThermalBuilder& enable_boussinesq_buoyancy() {
        enable_buoyancy_ = true;
        return *this;
    }

    /**
     * @brief Set gravity magnitude for buoyancy force (LBM units)
     * @param g_lbm Gravitational acceleration in LBM units
     * @return Reference for method chaining
     *
     * @note Use negative value for downward gravity (-Z direction)
     */
    ThermalBuilder& set_gravity_lbm(float32_t g_lbm) {
        gravity_lbm_ = g_lbm;
        has_gravity_ = true;
        return *this;
    }

    /**
     * @brief Set gravity direction axis
     * @param axis Gravity direction axis (default: Z)
     * @return Reference for method chaining
     */
    ThermalBuilder& set_gravity_axis(Axis axis) {
        gravity_axis_ = axis;
        return *this;
    }

    // ========================================================================
    // Initialization Helpers
    // ========================================================================

    /**
     * @brief Initialize with hydrostatic pressure distribution
     * @return Reference for method chaining
     *
     * Sets up initial density field with hydrostatic pressure gradient
     * based on gravity and domain height.
     */
    ThermalBuilder& initialize_hydrostatic_pressure() {
        init_hydrostatic_ = true;
        return *this;
    }

    /**
     * @brief Initialize with random velocity perturbations
     * @param magnitude Maximum perturbation magnitude in LBM units
     * @return Reference for method chaining
     *
     * Useful for triggering convection instability in simulations
     * like Rayleigh-Benard convection.
     */
    ThermalBuilder& initialize_random_perturbation(float32_t magnitude = 0.015f) {
        init_random_ = true;
        random_magnitude_ = magnitude;
        return *this;
    }

    // ========================================================================
    // Apply
    // ========================================================================

    /**
     * @brief Apply all configured thermal boundary conditions
     *
     * This method iterates over all cells and applies the configured
     * thermal boundaries, initializations, and buoyancy settings.
     */
    void apply() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        // Convert SI temperatures to LBM units
        float32_t hot_T_lbm = 1.5f;   // Default LBM hot temperature
        float32_t cold_T_lbm = 0.5f;  // Default LBM cold temperature
        float32_t delta_T_K = 1.0f;   // Temperature difference in Kelvin
        float32_t T_ref_K = 300.0f;   // Reference temperature

        if (use_si_temperatures_ && has_hot_wall_ && has_cold_wall_) {
            delta_T_K = thermal_utils::calc_delta_temperature(hot_temperature_K_, cold_temperature_K_);
            if (!has_reference_temp_) {
                T_ref_K = thermal_utils::calc_reference_temperature(hot_temperature_K_, cold_temperature_K_);
            } else {
                T_ref_K = T_ref_K_;
            }
            hot_T_lbm = thermal_utils::kelvin_to_lbm(hot_temperature_K_, T_ref_K, delta_T_K);
            cold_T_lbm = thermal_utils::kelvin_to_lbm(cold_temperature_K_, T_ref_K, delta_T_K);
        }

        // Calculate alpha and beta from Rayleigh/Prandtl if configured
        float32_t alpha = alpha_;
        float32_t beta = beta_;
        float32_t gravity = gravity_lbm_;

        if (use_rayleigh_config_) {
            // Ra = g * beta * dT * L³ / (nu * alpha)
            // Pr = nu / alpha => alpha = nu / Pr

            // Get reference length (domain height for Rayleigh-Benard)
            float32_t L = (float32_t)Nz;

            // Get viscosity from lbm (use typical value if not accessible)
            float32_t nu = 0.02f;

            // alpha = nu / Pr
            alpha = nu / prandtl_;

            // LBM temperature difference
            float32_t dT_lbm = hot_T_lbm - cold_T_lbm;
            float32_t g = 0.0005f;  // Typical LBM gravity

            // beta = Ra * nu * alpha / (g * dT * L³)
            beta = rayleigh_ * nu * alpha / (g * dT_lbm * L * L * L);

            gravity = g;
        }

        // Setup for random perturbation
        const uint32_t threads = (uint32_t)std::thread::hardware_concurrency();
        std::vector<uint32_t> seed(threads);
        for (uint32_t t = 0; t < threads; t++) seed[t] = 42u + t;

        parallel_for(lbm_.get_N(), threads, [&](uint64_t n, uint32_t t) {
            uint32_t x = 0, y = 0, z = 0;
            lbm_.coordinates(n, x, y, z);

            // Apply hot wall
            if (has_hot_wall_) {
                bool is_hot = boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, hot_face_, 1);
                if (is_hot) {
                    lbm_.T[n] = hot_T_lbm;
                    lbm_.flags[n] = TYPE_T;
                }
            }

            // Apply cold wall
            if (has_cold_wall_) {
                bool is_cold = boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, cold_face_, 1);
                if (is_cold) {
                    lbm_.T[n] = cold_T_lbm;
                    lbm_.flags[n] = TYPE_T;
                }
            }

            // Apply additional wall temperatures
            for (const auto& wt : wall_temps_) {
                if (boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, wt.face, 1)) {
                    float32_t T_lbm = thermal_utils::kelvin_to_lbm(wt.temperature_K, T_ref_K, delta_T_K);
                    lbm_.T[n] = T_lbm;
                    lbm_.flags[n] = TYPE_T;
                }
            }

            // Initialize hydrostatic pressure
            if (init_hydrostatic_) {
                uint32_t height_coord = z;
                if (gravity_axis_ == Axis::Y) height_coord = y;
                else if (gravity_axis_ == Axis::X) height_coord = x;

                uint32_t max_height = Nz;
                if (gravity_axis_ == Axis::Y) max_height = Ny;
                else if (gravity_axis_ == Axis::X) max_height = Nx;

                lbm_.rho[n] = units.rho_hydrostatic(gravity, (float32_t)height_coord, 0.5f * (float32_t)max_height);
            }

            // Initialize random perturbation
            if (init_random_ && lbm_.flags[n] != TYPE_S && lbm_.flags[n] != TYPE_T) {
                lbm_.u.x[n] = random_symmetric(seed[t], random_magnitude_);
                lbm_.u.y[n] = random_symmetric(seed[t], random_magnitude_);
                lbm_.u.z[n] = random_symmetric(seed[t], random_magnitude_);
            }
        });
    }

private:
    LBM& lbm_;

    // Hot/cold wall configuration (SI units - Kelvin)
    bool has_hot_wall_ = false;
    bool has_cold_wall_ = false;
    Face hot_face_ = Face::Z_MIN;
    Face cold_face_ = Face::Z_MAX;
    float32_t hot_temperature_K_ = 350.0f;   // Hot wall in Kelvin
    float32_t cold_temperature_K_ = 300.0f;  // Cold wall in Kelvin
    bool use_si_temperatures_ = false;

    // Reference temperature for dimensionless conversion
    float32_t T_ref_K_ = 300.0f;
    bool has_reference_temp_ = false;

    // Additional wall temperatures
    struct WallTemp {
        Face face;
        float32_t temperature_K;  // Temperature in Kelvin
    };
    std::vector<WallTemp> wall_temps_;

    // Thermal properties (LBM units)
    float32_t alpha_ = 1.0f;  // Thermal diffusivity
    float32_t beta_ = 1.0f;   // Thermal expansion

    // Rayleigh/Prandtl configuration
    bool use_rayleigh_config_ = false;
    float32_t rayleigh_ = 1e5f;
    float32_t prandtl_ = 0.71f;

    // Buoyancy and gravity
    bool enable_buoyancy_ = false;
    bool has_gravity_ = false;
    float32_t gravity_lbm_ = 0.0005f;
    Axis gravity_axis_ = Axis::Z;

    // Initialization options
    bool init_hydrostatic_ = false;
    bool init_random_ = false;
    float32_t random_magnitude_ = 0.015f;

    // Random number helper (simple LCG for thread-safety)
    static float32_t random_symmetric(uint32_t& seed, float32_t magnitude) {
        seed = seed * 1103515245u + 12345u;
        float32_t r = (float32_t)(seed & 0x7FFFFFFFu) / (float32_t)0x7FFFFFFFu;
        return magnitude * (2.0f * r - 1.0f);
    }
};
