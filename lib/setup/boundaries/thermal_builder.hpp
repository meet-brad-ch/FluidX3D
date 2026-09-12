#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/temperature_scale.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <optional>
#include <thread>
#include <vector>

#ifndef TEMPERATURE
#error "setup/boundaries/thermal_builder.hpp needs TEMPERATURE in defines.hpp"
#endif // TEMPERATURE

extern Units units; // global units object from lbm.cpp

/// @brief Hot and cold walls, hydrostatic and perturbed start (TEMPERATURE extension); apply() writes them to the grid.
///
/// Temperatures are converted with the scale the LBM was created with (SimulationSetup::configure_temperatures()),
/// speeds with the global units. The walls are the cell layer one inward from the given face.
/// @code
/// ThermalBuilder(lbm, sim.temperature_scale()).set_hot_wall(Face::Z_MIN, 330.0_K).set_cold_wall(Face::Z_MAX, 300.0_K).apply();
/// @endcode
class ThermalBuilder {
public:
    /// @param lbm          a thermal LBM (SimulationSetup::create_lbm_thermal())
    /// @param temperatures the scale the LBM was created with (SimulationSetup::temperature_scale())
    ThermalBuilder(LBM& lbm, const TemperatureScale& temperatures) : lbm_(lbm), temperatures_(temperatures) {}

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

    /// The axis gravity acts along, for the hydrostatic start (default Z).
    ThermalBuilder& set_gravity_axis(Axis axis) {
        gravity_axis_ = axis;
        return *this;
    }

    /// Initial density from the hydrostatic pressure of the LBM's gravity, relative to half the domain height.
    ThermalBuilder& initialize_hydrostatic_pressure() {
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
        const float32_t perturbation = perturbation_ ? units.u(perturbation_->si()) : 0.0f;

        const uint32_t threads = (uint32_t)std::thread::hardware_concurrency();
        std::vector<uint32_t> seed(threads);
        for (uint32_t t = 0; t < threads; t++) seed[t] = 42u + t;

        parallel_for(lbm_.get_N(), threads, [&](uint64_t n, uint32_t t) {
            uint32_t x = 0, y = 0, z = 0;
            lbm_.coordinates(n, x, y, z);

            if (hot_ && boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, hot_->face, 1)) {
                lbm_.T[n] = hot_T_lbm;
                lbm_.flags[n] = TYPE_T;
            }
            if (cold_ && boundary_utils::is_on_face(x, y, z, Nx, Ny, Nz, cold_->face, 1)) {
                lbm_.T[n] = cold_T_lbm;
                lbm_.flags[n] = TYPE_T;
            }

            if (init_hydrostatic_) {
                const uint32_t height_coord = gravity_axis_ == Axis::X ? x : gravity_axis_ == Axis::Y ? y : z;
                const uint32_t max_height = gravity_axis_ == Axis::X ? Nx : gravity_axis_ == Axis::Y ? Ny : Nz;
                lbm_.rho[n] = units.rho_hydrostatic(gravity, (float32_t)height_coord, 0.5f * (float32_t)max_height);
            }

            if (perturbation_ && !(lbm_.flags[n] & (TYPE_S|TYPE_T))) {
                lbm_.u.x[n] = random_symmetric(seed[t], perturbation);
                lbm_.u.y[n] = random_symmetric(seed[t], perturbation);
                lbm_.u.z[n] = random_symmetric(seed[t], perturbation);
            }
        });
    }

private:
    struct Wall { Face face; Temperature temperature; }; ///< a wall at a fixed temperature

    LBM& lbm_;
    TemperatureScale temperatures_;
    std::optional<Wall> hot_, cold_;
    Axis gravity_axis_ = Axis::Z;
    bool init_hydrostatic_ = false;
    std::optional<Speed> perturbation_;

    static float32_t random_symmetric(uint32_t& seed, float32_t magnitude) { // LCG, one seed per thread
        seed = seed * 1103515245u + 12345u;
        float32_t r = (float32_t)(seed & 0x7FFFFFFFu) / (float32_t)0x7FFFFFFFu;
        return magnitude * (2.0f * r - 1.0f);
    }
};
