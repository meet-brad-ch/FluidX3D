#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/unit_scale.hpp"
#include "setup/core/temperature_scale.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/domain/lattice.hpp"
#include "lbm.hpp"
#include <optional>
#include <thread>
#include <vector>

#ifndef TEMPERATURE
#error "setup/boundaries/thermal_builder.hpp needs TEMPERATURE in defines.hpp"
#endif // TEMPERATURE

/// @brief Hot and cold walls, hydrostatic and perturbed start (TEMPERATURE extension); apply() writes them to the grid
/// (Simulation::thermal()).
///
/// Temperatures are converted with the scale of Simulation::set_temperatures(), speeds with the unit scale. The walls
/// are the cell layer one inward from the given face.
/// @code
/// sim.thermal().set_hot_wall(Face::Z_MIN, 330.0_K).set_cold_wall(Face::Z_MAX, 300.0_K).initialize_hydrostatic().apply();
/// @endcode
class ThermalBuilder {
public:
    /// @param lbm          a thermal LBM
    /// @param unit_scale        the simulation's unit scale
    /// @param temperatures the scale the LBM was created with
    /// @param gravity_axis the axis the simulation's gravity acts along, for the hydrostatic start
    ThermalBuilder(LBM& lbm, const UnitScale& unit_scale, const TemperatureScale& temperatures, Axis gravity_axis)
        : lbm_(lbm), units_(unit_scale), temperatures_(temperatures), gravity_axis_(gravity_axis) {}

    /// A wall at this temperature, one cell inward from the face.
    ThermalBuilder& set_hot_wall(Face face, Temperature temperature) {
        hot_ = Wall{ face, temperature };
        return *this;
    }

    /// A wall at this temperature, one cell inward from the face.
    ThermalBuilder& set_cold_wall(Face face, Temperature temperature) {
        cold_ = Wall{ face, temperature };
        return *this;
    }

    /// Initial density from the hydrostatic pressure of the simulation's gravity, relative to half the domain height.
    ThermalBuilder& initialize_hydrostatic() {
        init_hydrostatic_ = true;
        return *this;
    }

    /// Random initial velocity components up to this speed in the fluid cells, to trigger convection.
    ThermalBuilder& initialize_random_perturbation(Speed magnitude) {
        perturbation_ = magnitude;
        return *this;
    }

    /// Writes the walls and the initial state to the grid.
    void apply() {
        const uint32_t Nx = lbm_.get_Nx();
        const uint32_t Ny = lbm_.get_Ny();
        const uint32_t Nz = lbm_.get_Nz();

        const float32_t hot_T_lbm = hot_ ? temperatures_.lattice(hot_->temperature) : 1.0f;
        const float32_t cold_T_lbm = cold_ ? temperatures_.lattice(cold_->temperature) : 1.0f;
        const float32_t gravity = -(gravity_axis_ == Axis::X ? lbm_.get_fx() : gravity_axis_ == Axis::Y ? lbm_.get_fy() : lbm_.get_fz());
        const float32_t perturbation = perturbation_ ? units_.velocity(*perturbation_) : 0.0f;

        const uint32_t threads = (uint32_t)std::thread::hardware_concurrency();
        std::vector<uint32_t> seed(threads);
        for(uint32_t t = 0; t < threads; t++) seed[t] = 42u + t;

        parallel_for(lbm_.get_N(), threads, [&](uint64_t n, uint32_t t) {
            uint32_t x = 0, y = 0, z = 0;
            lbm_.coordinates(n, x, y, z);

            if(hot_ && boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, hot_->face, 1)) {
                lbm_.T[n] = hot_T_lbm;
                lbm_.flags[n] = TYPE_T;
            }
            if(cold_ && boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, cold_->face, 1)) {
                lbm_.T[n] = cold_T_lbm;
                lbm_.flags[n] = TYPE_T;
            }

            if(init_hydrostatic_) {
                const uint32_t height_coord = gravity_axis_ == Axis::X ? x : gravity_axis_ == Axis::Y ? y : z;
                const uint32_t max_height = gravity_axis_ == Axis::X ? Nx : gravity_axis_ == Axis::Y ? Ny : Nz;
                lbm_.rho[n] = hydrostatic_density(gravity, (float32_t)height_coord, 0.5f * (float32_t)max_height);
            }

            if(perturbation_ && !(lbm_.flags[n] & (TYPE_S|TYPE_T))) {
                lbm_.u.x[n] = random_symmetric(seed[t], perturbation);
                lbm_.u.y[n] = random_symmetric(seed[t], perturbation);
                lbm_.u.z[n] = random_symmetric(seed[t], perturbation);
            }
        });
    }

private:
    struct Wall { Face face; Temperature temperature; }; ///< a wall at a fixed temperature

    LBM& lbm_;
    UnitScale units_;
    TemperatureScale temperatures_;
    Axis gravity_axis_;
    std::optional<Wall> hot_, cold_;
    bool init_hydrostatic_ = false;
    std::optional<Speed> perturbation_;

    static float32_t random_symmetric(uint32_t& seed, float32_t magnitude) { // LCG, one seed per thread
        seed = seed * 1103515245u + 12345u;
        float32_t r = (float32_t)(seed & 0x7FFFFFFFu) / (float32_t)0x7FFFFFFFu;
        return magnitude * (2.0f * r - 1.0f);
    }
};
