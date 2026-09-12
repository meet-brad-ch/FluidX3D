#pragma once

#include "core/types.hpp"
#include "core/boundary_utils.hpp"
#include "boundaries/boundary_flags.hpp"
#include "boundaries/thermal_utils.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <vector>
#include <thread>

extern Units units; // global units object from lbm.cpp

// Hot and cold walls in Kelvin, hydrostatic and perturbed initial state (TEMPERATURE extension); apply() writes them to the grid.
// The walls are the cell layer one inward from the given face.
class ThermalBuilder {
public:
    explicit ThermalBuilder(LBM& lbm) : lbm_(lbm) {}

    ThermalBuilder& set_hot_wall(Face face, float32_t temperature_K) {
        hot_face_ = face;
        hot_temperature_K_ = temperature_K;
        has_hot_wall_ = true;
        return *this;
    }

    ThermalBuilder& set_cold_wall(Face face, float32_t temperature_K) {
        cold_face_ = face;
        cold_temperature_K_ = temperature_K;
        has_cold_wall_ = true;
        return *this;
    }

    // axis along which gravity acts, for the hydrostatic initialization (default Z)
    ThermalBuilder& set_gravity_axis(Axis axis) {
        gravity_axis_ = axis;
        return *this;
    }

    ThermalBuilder& initialize_hydrostatic_pressure() {
        init_hydrostatic_ = true;
        return *this;
    }

    // random initial velocity up to magnitude (LBM units) to trigger convection
    ThermalBuilder& initialize_random_perturbation(float32_t magnitude = 0.015f) {
        init_random_ = true;
        random_magnitude_ = magnitude;
        return *this;
    }

    void apply() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        float32_t hot_T_lbm = 1.5f;
        float32_t cold_T_lbm = 0.5f;
        if (has_hot_wall_ && has_cold_wall_) {
            const float32_t delta_T_K = thermal_utils::calc_delta_temperature(hot_temperature_K_, cold_temperature_K_);
            const float32_t T_ref_K = thermal_utils::calc_reference_temperature(hot_temperature_K_, cold_temperature_K_);
            hot_T_lbm = thermal_utils::kelvin_to_lbm(hot_temperature_K_, T_ref_K, delta_T_K);
            cold_T_lbm = thermal_utils::kelvin_to_lbm(cold_temperature_K_, T_ref_K, delta_T_K);
        }

        const uint32_t threads = (uint32_t)std::thread::hardware_concurrency();
        std::vector<uint32_t> seed(threads);
        for (uint32_t t = 0; t < threads; t++) seed[t] = 42u + t;

        parallel_for(lbm_.get_N(), threads, [&](uint64_t n, uint32_t t) {
            uint32_t x = 0, y = 0, z = 0;
            lbm_.coordinates(n, x, y, z);

            if (has_hot_wall_ && boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, hot_face_, 1)) {
                lbm_.T[n] = hot_T_lbm;
                lbm_.flags[n] = TYPE_T;
            }
            if (has_cold_wall_ && boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, cold_face_, 1)) {
                lbm_.T[n] = cold_T_lbm;
                lbm_.flags[n] = TYPE_T;
            }

            if (init_hydrostatic_) {
                const uint32_t height_coord = gravity_axis_ == Axis::X ? x : gravity_axis_ == Axis::Y ? y : z;
                const uint32_t max_height = gravity_axis_ == Axis::X ? Nx : gravity_axis_ == Axis::Y ? Ny : Nz;
                lbm_.rho[n] = units.rho_hydrostatic(hydrostatic_gravity_lbm_, (float32_t)height_coord, 0.5f * (float32_t)max_height);
            }

            if (init_random_ && !(lbm_.flags[n] & (TYPE_S|TYPE_T))) {
                lbm_.u.x[n] = random_symmetric(seed[t], random_magnitude_);
                lbm_.u.y[n] = random_symmetric(seed[t], random_magnitude_);
                lbm_.u.z[n] = random_symmetric(seed[t], random_magnitude_);
            }
        });
    }

private:
    LBM& lbm_;

    bool has_hot_wall_ = false;
    bool has_cold_wall_ = false;
    Face hot_face_ = Face::Z_MIN;
    Face cold_face_ = Face::Z_MAX;
    float32_t hot_temperature_K_ = 350.0f;
    float32_t cold_temperature_K_ = 300.0f;

    // fixed lattice gravity for the hydrostatic initialization; does not follow the LBM's volume force (review A6)
    static constexpr float32_t hydrostatic_gravity_lbm_ = 0.0005f;
    Axis gravity_axis_ = Axis::Z;

    bool init_hydrostatic_ = false;
    bool init_random_ = false;
    float32_t random_magnitude_ = 0.015f;

    static float32_t random_symmetric(uint32_t& seed, float32_t magnitude) { // LCG, one seed per thread
        seed = seed * 1103515245u + 12345u;
        float32_t r = (float32_t)(seed & 0x7FFFFFFFu) / (float32_t)0x7FFFFFFFu;
        return magnitude * (2.0f * r - 1.0f);
    }
};
