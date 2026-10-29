#pragma once

#include "setup/core/types.hpp"
#include "setup/core/boundary_utils.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/simulation/runner.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"
#include "lbm.hpp"
#include <cmath>
#include <optional>

#ifndef SURFACE
#error "setup/surface/wave_boundary.hpp needs SURFACE in defines.hpp"
#endif // SURFACE

/// @file wave_boundary.hpp
/// @brief An oscillating wave maker on an inlet face (WaveBoundary).

/// @brief Drives an oscillating wave maker on an X or Y inlet face (SURFACE and EQUILIBRIUM_BOUNDARIES extensions),
/// owned by the simulation (Simulation::wave_maker()).
///
/// Owned by Simulation, which makes it with the LBM; it keeps references to the LBM and the runner. initialize() marks
/// the inner cells of the inlet face as equilibrium boundaries (TYPE_E) on the host and schedules update(), which then
/// sets their velocities on the device at every update interval of simulated time: u_normal = u_peak sin(2 pi f t)
/// along the face normal, u_z = vertical_factor u_peak cos(2 pi f t).
/// @code
/// using namespace si::symbols;
/// sim.wave_maker().set_wave(0.21 * m, 0.78 * Hz).set_inlet_face(Face::Y_MIN).initialize();
/// @endcode
class WaveBoundary {
public:
    /// pi, for the wave's phase in double.
    static constexpr double pi_double = 3.14159265358979323846;
    /// The default frequency of the wave.
    static constexpr si::Frequency default_frequency{ au::hertz(0.5) };
    /// The default update interval is the wave's period over this number.
    static constexpr double updates_per_period = 16.0;

    /// @brief Makes the wave maker of an LBM.
    /// @param lbm the free surface LBM
    /// @param converter the simulation's conversions
    /// @param runner the simulation's runner, which calls update()
    WaveBoundary(LBM& lbm, const si::LatticeConverter& converter, Runner& runner) : lbm_(lbm), converter_(converter), runner_(runner) {}

    /// @brief Sets the wave's amplitude and frequency; its peak velocity is 2 pi frequency amplitude.
    /// @param amplitude the wave's amplitude
    /// @param frequency the wave's frequency, more than 0 (the default update interval divides by it)
    /// @return this wave maker
    WaveBoundary& set_wave(si::Length amplitude, si::Frequency frequency) {
        frequency_ = frequency;
        const si::Speed amplitude_rate{ amplitude * frequency };
        peak_velocity_ = amplitude_rate * (2.0 * pi_double);
        return *this;
    }

    /// @brief Sets the face the waves come from (default Y_MIN). A Z face is not supported: initialize() ends the
    /// program with print_error() for it.
    /// @param face the inlet face
    /// @return this wave maker
    WaveBoundary& set_inlet_face(Face face) {
        inlet_face_ = face;
        return *this;
    }

    /// @brief Sets the vertical velocity as a part of the horizontal one (default 0.5; smaller for shallow water).
    /// @param factor the vertical velocity's peak over the horizontal one's (a ratio)
    /// @return this wave maker
    WaveBoundary& set_vertical_factor(float32_t factor) {
        vertical_factor_ = factor;
        return *this;
    }

    /// @brief Sets the simulated time between updates of the inlet velocities (default: a sixteenth of the wave's
    /// period).
    /// @param interval the simulated time between two updates
    /// @return this wave maker
    WaveBoundary& set_update_interval(si::Duration interval) {
        update_interval_ = interval;
        return *this;
    }

    /// @brief Marks the inner cells of the inlet face as equilibrium boundaries (TYPE_E) and schedules the updates.
    ///
    /// Ends the program with print_error() for a Z inlet face (the wave cannot be driven there).
    void initialize() {
        const bool z_face{ inlet_face_ == Face::Z_MIN || inlet_face_ == Face::Z_MAX };
        if(z_face) {
            print_error("WaveBoundary::initialize(): the inlet face must be an X or Y face, not a Z face");
        }
        u_wave_lbm_ = converter_.velocity(peak_velocity_);
        const uint3 cells{ lbm_.get_Nx(), lbm_.get_Ny(), lbm_.get_Nz() };
        parallel_for(lbm_.get_N(), [&](uint64_t n) { mark_inlet_cell(n, cells); });
        initialized_ = true;
        const si::Duration period_part{ 1.0 / (frequency_ * updates_per_period) };
        const si::Duration interval{ update_interval_.value_or(period_part) };
        runner_.every(interval, [this](si::Duration time) { update(time); });
    }

    /// @brief Sets the inlet velocities for a simulated time since the start. It reads and writes the velocity field
    /// on the device. The runner calls it after initialize().
    ///
    /// Ends the program with print_error() when it is called before initialize().
    /// @param time the simulated time since the start
    void update(si::Duration time) {
        if(!initialized_) {
            print_error("WaveBoundary::update(): call initialize() first");
        }
        const double cycles{ au::as_raw_number(frequency_ * time) };
        const double phase{ 2.0 * pi_double * cycles };
        const float32_t u_primary{ u_wave_lbm_ * static_cast<float32_t>(std::sin(phase)) };
        const float32_t u_vertical{ vertical_factor_ * u_wave_lbm_ * static_cast<float32_t>(std::cos(phase)) };
        const bool min_face{ inlet_face_ == Face::X_MIN || inlet_face_ == Face::Y_MIN };
        const float32_t u_normal{ min_face ? u_primary : -u_primary };
        lbm_.u.read_from_device();
        // the inner cells of the inlet face: the primary velocity along the face normal, plus the vertical component
        for(uint32_t z = 1u; z < lbm_.get_Nz() - 1u; z++) {
            write_inlet_row(z, u_normal, u_vertical, min_face);
        }
        lbm_.u.write_to_device();
    }

private:
    LBM& lbm_;                       ///< the free surface LBM (not owned)
    si::LatticeConverter converter_; ///< the simulation's conversions
    Runner& runner_;                 ///< the simulation's runner (not owned)

    si::Frequency frequency_{ default_frequency }; ///< the wave's frequency
    si::Speed peak_velocity_{ au::ZERO };                        ///< the wave's peak velocity
    std::optional<si::Duration> update_interval_{};              ///< set_update_interval(), or none

    float32_t u_wave_lbm_ = 0.0f; ///< the peak velocity in cells per time step

    Face inlet_face_ = Face::Y_MIN;    ///< the face the waves come from
    float32_t vertical_factor_ = 0.5f; ///< the vertical velocity's peak over the horizontal one's (a ratio)

    bool initialized_ = false; ///< true after initialize()

    /// @brief Tells whether a cell index is inside an axis, not at either end.
    /// @param index the cell along the axis, counted from 0
    /// @param count the cells along the axis
    /// @return true for 0 < index < count - 1
    static bool inner_index(uint32_t index, uint32_t count) {
        const bool inner{ index > 0u && index < count - 1u };
        return inner;
    }

    /// @brief Marks a cell as an equilibrium boundary when it is on the inlet face and not on its edges with the
    /// other faces.
    /// @param n the cell's index
    /// @param cells the grid's cells along X, Y and Z
    void mark_inlet_cell(uint64_t n, const uint3& cells) {
        uint32_t x{ 0u };
        uint32_t y{ 0u };
        uint32_t z{ 0u };
        lbm_.coordinates(n, x, y, z);
        const FaceLayer inlet_layer(inlet_face_, cells);
        const bool on_inlet{ inlet_layer.contains(x, y, z) };
        if(!on_inlet) {
            return;
        }
        const bool inner_x{ inner_index(x, cells.x) };
        const bool inner_y{ inner_index(y, cells.y) };
        const bool inner_z{ inner_index(z, cells.z) };
        // not on the edges the inlet face shares with other faces: inner along the two axes in the face
        const std::size_t normal_axis{ static_cast<std::size_t>(inlet_face_) / 2u }; // X faces 0, Y faces 1, Z faces 2
        const bool inner_along_x{ normal_axis == 0u || inner_x };
        const bool inner_along_y{ normal_axis == 1u || inner_y };
        const bool inner_along_z{ normal_axis == 2u || inner_z };
        const bool inner{ inner_along_x && inner_along_y && inner_along_z };
        if(inner) {
            lbm_.flags[n] = TYPE_E;
        }
    }

    /// @brief Sets the velocities of the inner inlet cells in one layer of z (an X or Y inlet face only).
    /// @param z the layer along Z, from 1 to Nz - 2
    /// @param u_normal the velocity along the face normal in cells per time step
    /// @param u_vertical the vertical velocity in cells per time step
    /// @param min_face true for an inlet at the minimum of its axis
    void write_inlet_row(uint32_t z, float32_t u_normal, float32_t u_vertical, bool min_face) {
        const bool y_face{ inlet_face_ == Face::Y_MIN || inlet_face_ == Face::Y_MAX };
        const bool x_face{ inlet_face_ == Face::X_MIN || inlet_face_ == Face::X_MAX };
        if(y_face) {
            write_y_inlet_row(z, u_normal, u_vertical, min_face);
        } else if(x_face) {
            write_x_inlet_row(z, u_normal, u_vertical, min_face);
        }
    }

    /// @brief Sets the velocities of the inner cells of a Y inlet face in one layer of z.
    /// @param z the layer along Z, from 1 to Nz - 2
    /// @param u_normal the velocity along +y (the face normal) in cells per time step
    /// @param u_vertical the vertical velocity in cells per time step
    /// @param min_face true for the inlet at y = 0, false for the inlet at y = Ny - 1
    void write_y_inlet_row(uint32_t z, float32_t u_normal, float32_t u_vertical, bool min_face) {
        const uint32_t nx{ lbm_.get_Nx() };
        const uint32_t last_y{ lbm_.get_Ny() - 1u };
        const uint32_t y{ min_face ? 0u : last_y };
        for(uint32_t x = 1u; x < nx - 1u; x++) {
            const uint64_t n{ lbm_.index(x, y, z) };
            lbm_.u.y[n] = u_normal;
            lbm_.u.z[n] = u_vertical;
        }
    }

    /// @brief Sets the velocities of the inner cells of an X inlet face in one layer of z.
    /// @param z the layer along Z, from 1 to Nz - 2
    /// @param u_normal the velocity along +x (the face normal) in cells per time step
    /// @param u_vertical the vertical velocity in cells per time step
    /// @param min_face true for the inlet at x = 0, false for the inlet at x = Nx - 1
    void write_x_inlet_row(uint32_t z, float32_t u_normal, float32_t u_vertical, bool min_face) {
        const uint32_t ny{ lbm_.get_Ny() };
        const uint32_t last_x{ lbm_.get_Nx() - 1u };
        const uint32_t x{ min_face ? 0u : last_x };
        for(uint32_t y = 1u; y < ny - 1u; y++) {
            const uint64_t n{ lbm_.index(x, y, z) };
            lbm_.u.x[n] = u_normal;
            lbm_.u.z[n] = u_vertical;
        }
    }
};
