#pragma once

#include "setup/core/types.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/domain/lattice.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"
#include "lbm.hpp"
#include <optional>

#ifndef TEMPERATURE
#error "setup/boundaries/thermal_builder.hpp needs TEMPERATURE in defines.hpp"
#endif // TEMPERATURE

/// @file thermal_builder.hpp
/// @brief Hot and cold walls and the start of a thermal simulation (ThermalBuilder).

/// @brief Sets hot and cold walls and a hydrostatic and perturbed start (TEMPERATURE extension); apply() writes them
/// to the grid (Simulation::thermal()).
///
/// A temporary builder (L2) that keeps a reference to the LBM, which must outlive it. apply() writes the core's
/// temperatures T, flags (TYPE_T for a wall), densities rho and velocities u on the host. Temperatures become lattice
/// temperatures through the simulation's converter, with the temperature scale of Simulation::set_temperatures()
/// (cold 0.5, hot 1.5). A wall is the layer of cells one cell inward from the given face.
/// @code
/// sim.thermal().set_hot_wall(Face::Z_MIN, si::kelvin_point(330.0)).set_cold_wall(Face::Z_MAX, si::kelvin_point(300.0)).initialize_hydrostatic().apply();
/// @endcode
class ThermalBuilder {
public:
    /// The seed of the random start velocities: with the cell index and the component it gives each value.
    static constexpr uint64_t random_seed = 42u;
    /// The number of velocity components per cell: the hash key of a component is 3 n + component.
    static constexpr uint64_t components = 3u;
    /// The shift that keeps the top 24 bits of a 64-bit hash: they make a value in [0, 1).
    static constexpr unsigned value_shift = 40u;
    /// The number of values of 24 bits (2^24).
    static constexpr double value_range = 16777216.0;

    /// @brief Makes a builder for a thermal LBM.
    /// @param lbm a thermal LBM
    /// @param converter the simulation's conversions, with its temperature scale
    /// @param gravity_axis the axis along which the simulation's gravity acts, for the hydrostatic start
    ThermalBuilder(LBM& lbm, const si::LatticeConverter& converter, Axis gravity_axis) : lbm_(lbm), converter_(converter), gravity_axis_(gravity_axis) {}

    /// @brief Sets the hot wall: the layer one cell inward from a face, at a fixed temperature.
    /// @param face the face of the box
    /// @param temperature the wall's temperature
    /// @return this builder
    ThermalBuilder& set_hot_wall(Face face, si::Temperature temperature) {
        hot_ = Wall{ .face = face, .temperature = temperature };
        return *this;
    }

    /// @brief Sets the cold wall: the layer one cell inward from a face, at a fixed temperature.
    /// @param face the face of the box
    /// @param temperature the wall's temperature
    /// @return this builder
    ThermalBuilder& set_cold_wall(Face face, si::Temperature temperature) {
        cold_ = Wall{ .face = face, .temperature = temperature };
        return *this;
    }

    /// @brief Sets the start density from the hydrostatic pressure of the simulation's gravity, with the density 1 at
    /// half the domain's height along the gravity axis.
    /// @return this builder
    ThermalBuilder& initialize_hydrostatic() {
        init_hydrostatic_ = true;
        return *this;
    }

    /// @brief Sets random start velocity components in the fluid cells, to trigger convection: each component
    /// uniform between -magnitude and +magnitude. Each value comes from a hash of the cell's index and the component
    /// (random_symmetric()), so the start field is the same for any thread count and any run.
    /// @param magnitude the largest component
    /// @return this builder
    ThermalBuilder& initialize_random_perturbation(si::Speed magnitude) {
        perturbation_ = magnitude;
        return *this;
    }

    /// @brief Writes the walls and the start state to the grid.
    /// @throws SetupError when the simulation's converter has no temperature scale (with a wall)
    void apply() {
        const LatticeStart start{ lattice_start() };
        parallel_for(lbm_.get_N(), [&](uint64_t n) { write_cell(n, start); });
    }

private:
    /// A wall at a fixed temperature.
    struct Wall {
        Face face = Face::Z_MIN;                     ///< the face the wall is one cell inward from
        si::Temperature temperature{ si::kelvin_point(0.0) }; ///< the wall's temperature
    };

    /// The lattice values that apply() writes, made once before the cell loop, so that the loop reads no optional.
    struct LatticeStart {
        bool has_hot{ false };        ///< true when a hot wall is set
        FaceLayer hot_layer{ Face::Z_MIN, uint3(1u, 1u, 1u) }; ///< the hot wall's layer, one cell inward from its face (not read without a hot wall)
        float hot{ 1.0f };            ///< the hot wall's lattice temperature (1 without a hot wall)
        bool has_cold{ false };       ///< true when a cold wall is set
        FaceLayer cold_layer{ Face::Z_MIN, uint3(1u, 1u, 1u) }; ///< the cold wall's layer, one cell inward from its face (not read without a cold wall)
        float cold{ 1.0f };           ///< the cold wall's lattice temperature (1 without a cold wall)
        bool perturbed{ false };      ///< true after initialize_random_perturbation()
        float gravity{};           ///< the core's force per volume toward lower heights along the gravity axis
        float perturbation{};      ///< the largest random velocity component in cells per time step
        uint3 cells{};             ///< the grid's cells along X, Y and Z
    };

    LBM& lbm_;                               ///< the thermal LBM (not owned)
    si::LatticeConverter converter_;         ///< the simulation's conversions, with its temperature scale
    Axis gravity_axis_{ Axis::Z };           ///< the axis along which gravity acts
    std::optional<Wall> hot_{};              ///< the hot wall, or none
    std::optional<Wall> cold_{};             ///< the cold wall, or none
    bool init_hydrostatic_ = false;          ///< true after initialize_hydrostatic()
    std::optional<si::Speed> perturbation_{}; ///< initialize_random_perturbation(), or none

    /// @brief Converts the settings to lattice values.
    /// @return the lattice values
    /// @throws SetupError when a wall is set and the converter has no temperature scale
    LatticeStart lattice_start() const {
        LatticeStart start{};
        const uint3 grid{ lbm_.get_Nx(), lbm_.get_Ny(), lbm_.get_Nz() };
        if(hot_.has_value()) {
            const Wall& hot{ hot_.value() };
            start.has_hot = true;
            start.hot_layer = FaceLayer(hot.face, grid, 1u);
            start.hot = converter_.temperature(hot.temperature);
        }
        if(cold_.has_value()) {
            const Wall& cold{ cold_.value() };
            start.has_cold = true;
            start.cold_layer = FaceLayer(cold.face, grid, 1u);
            start.cold = converter_.temperature(cold.temperature);
        }
        const float force{ along_gravity(lbm_.get_fx(), lbm_.get_fy(), lbm_.get_fz()) };
        start.gravity = -force;
        if(perturbation_.has_value()) {
            start.perturbed = true;
            start.perturbation = converter_.velocity(perturbation_.value());
        }
        start.cells = grid;
        return start;
    }

    /// @brief Gives the component of a force along the gravity axis.
    /// @param x the component along X, in lattice units
    /// @param y the component along Y, in lattice units
    /// @param z the component along Z, in lattice units
    /// @return the component along gravity_axis_
    float along_gravity(float x, float y, float z) const {
        float component{ z };
        if(gravity_axis_ == Axis::X) {
            component = x;
        } else if(gravity_axis_ == Axis::Y) {
            component = y;
        }
        return component;
    }

    /// @brief Gives the component of a cell coordinate or a cell count along the gravity axis.
    /// @param x the value along X, in cells
    /// @param y the value along Y, in cells
    /// @param z the value along Z, in cells
    /// @return the value along gravity_axis_
    uint32_t along_gravity(uint32_t x, uint32_t y, uint32_t z) const {
        uint32_t component{ z };
        if(gravity_axis_ == Axis::X) {
            component = x;
        } else if(gravity_axis_ == Axis::Y) {
            component = y;
        }
        return component;
    }

    /// @brief Writes one cell: the walls, the hydrostatic density and the random velocity.
    /// @param n the cell's index
    /// @param start the lattice values
    void write_cell(uint64_t n, const LatticeStart& start) {
        uint32_t x{ 0u };
        uint32_t y{ 0u };
        uint32_t z{ 0u };
        lbm_.coordinates(n, x, y, z);
        const uint3& c{ start.cells };
        if(start.has_hot && start.hot_layer.contains(x, y, z)) {
            lbm_.T[n] = start.hot;
            lbm_.flags[n] = TYPE_T;
        }
        if(start.has_cold && start.cold_layer.contains(x, y, z)) {
            lbm_.T[n] = start.cold;
            lbm_.flags[n] = TYPE_T;
        }
        if(init_hydrostatic_) {
            const uint32_t height_cells{ along_gravity(x, y, z) };
            const uint32_t extent_cells{ along_gravity(c.x, c.y, c.z) };
            const float height{ static_cast<float32_t>(height_cells) };
            const float surface{ 0.5f * static_cast<float32_t>(extent_cells) };
            lbm_.rho[n] = HydrostaticDensity::at(start.gravity, height, surface);
        }
        const uchar wall_bits{ static_cast<uchar>(lbm_.flags[n] & (TYPE_S | TYPE_T)) };
        const bool fluid{ wall_bits == 0u };
        if(start.perturbed && fluid) {
            lbm_.u.x[n] = random_symmetric(n, 0u, start.perturbation);
            lbm_.u.y[n] = random_symmetric(n, 1u, start.perturbation);
            lbm_.u.z[n] = random_symmetric(n, 2u, start.perturbation);
        }
    }

    /// @brief Gives the random start value of one velocity component of one cell: the same value for the same cell
    /// and component in every run, whatever the thread that writes it.
    /// @param n the cell's index
    /// @param component the velocity component, 0 (x), 1 (y) or 2 (z)
    /// @param magnitude the largest magnitude, in lattice units
    /// @return a number from -magnitude (included) to magnitude (excluded)
    static float32_t random_symmetric(uint64_t n, uint64_t component, float32_t magnitude) {
        const uint64_t key{ n * components + component };
        const uint64_t cell_hash{ mix(key) };
        const uint64_t hash{ mix(cell_hash ^ random_seed) };
        const uint64_t top_bits{ hash >> value_shift };
        const double r{ static_cast<double>(top_bits) / value_range };
        const double symmetric{ 2.0 * r - 1.0 };
        const float32_t value{ magnitude * static_cast<float32_t>(symmetric) };
        return value;
    }

    /// @brief Mixes a 64-bit key into a hash: the SplitMix64 finalizer (Steele, Lea and Flood 2014), the same function
    /// as StartNoise's block noise.
    /// @param k the key
    /// @return the hash
    static uint64_t mix(uint64_t k) {
        const uint64_t k1{ k + 0x9E3779B97F4A7C15ull };
        const uint64_t shifted1{ k1 >> 30 };
        const uint64_t mixed1{ k1 ^ shifted1 };
        const uint64_t k2{ mixed1 * 0xBF58476D1CE4E5B9ull };
        const uint64_t shifted2{ k2 >> 27 };
        const uint64_t mixed2{ k2 ^ shifted2 };
        const uint64_t k3{ mixed2 * 0x94D049BB133111EBull };
        const uint64_t shifted3{ k3 >> 31 };
        const uint64_t result{ k3 ^ shifted3 };
        return result;
    }
};
