#pragma once

#include "setup/core/types.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include "lbm.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>

/// @file field_reader.hpp
/// @brief A snapshot of the simulation's fields in physical units (FieldReader).

/// @brief A snapshot of the simulation's fields in physical units (Simulation::fields()): the velocity, pressure and
/// density of every cell, or of the cell at a point.
///
/// A temporary reader that keeps a reference to the LBM, which must outlive it. Making it reads the velocity, the
/// density and the flags from the device once the simulation has started (before that the device holds nothing yet
/// and the host holds the setup). The values become physical through the simulation's converter (which needs its
/// density). Positions are in metres from the domain's origin corner: cell i spans i to i+1 cell sizes, its centre is
/// at i + 1/2. Pressures are relative to the fluid at rest.
/// @code
/// using namespace si::symbols;
/// const FieldReader fields = sim.fields();
/// const std::uint64_t count{ fields.cell_count() };
/// for(std::uint64_t n{ 0u }; n < count; ++n) {
///     const FieldReader::Cell c{ fields.cell(n) };
///     if(!c.solid) {
///         energy += c.velocity.magnitude().in(m / s);
///     }
/// }
/// const si::Speed u = fields.velocity_at({ 0.5 * m, 1.0 * m, 0.2 * m }).magnitude();
/// @endcode
class FieldReader {
public:
    /// One cell's state in physical units.
    struct Cell {
        si::Position center{};                ///< the cell's centre in metres from the origin corner
        si::Velocity velocity{};              ///< the flow velocity (a solid cell: its wall's velocity)
        si::Pressure pressure{ au::ZERO };    ///< the pressure relative to the fluid at rest
        si::Density density{ au::ZERO };      ///< the density
        bool solid = false;                   ///< true for a solid cell (TYPE_S)
    };

    /// @brief Makes the snapshot.
    /// @param lbm the LBM whose fields are read
    /// @param converter the simulation's conversions, with its density
    /// @param from_device true to read the velocity, density and flags from the device (once the simulation has started)
    FieldReader(LBM& lbm, const si::LatticeConverter& converter, bool from_device) : lbm_(lbm), converter_(converter) {
        if(!from_device) {
            return;
        }
        lbm_.u.read_from_device();
        lbm_.rho.read_from_device();
        lbm_.flags.read_from_device();
    }

    /// @brief Gives the state of the cell that contains a point.
    /// @param point a point in metres from the origin corner
    /// @return the cell containing the point; for a point outside the domain, the nearest cell along each axis
    /// @throws SetupError when the converter has no density (from LatticeConverter::si_pressure())
    Cell cell_at(const si::Position& point) const {
        const uint32_t x{ clamp_index(converter_.cells(point.x), lbm_.get_Nx()) };
        const uint32_t y{ clamp_index(converter_.cells(point.y), lbm_.get_Ny()) };
        const uint32_t z{ clamp_index(converter_.cells(point.z), lbm_.get_Nz()) };
        const uint64_t n{ lbm_.index(x, y, z) };
        const Cell state{ state_of(n, x, y, z) };
        return state;
    }

    /// @brief Gives the velocity of the cell that contains a point.
    /// @param point a point in metres from the origin corner
    /// @return the cell's velocity
    /// @throws SetupError when the converter has no density
    si::Velocity velocity_at(const si::Position& point) const {
        const Cell state{ cell_at(point) };
        return state.velocity;
    }

    /// @brief Gives the pressure of the cell that contains a point.
    /// @param point a point in metres from the origin corner
    /// @return the cell's pressure relative to the fluid at rest
    /// @throws SetupError when the converter has no density
    si::Pressure pressure_at(const si::Position& point) const {
        const Cell state{ cell_at(point) };
        return state.pressure;
    }

    /// @brief Gives the density of the cell that contains a point.
    /// @param point a point in metres from the origin corner
    /// @return the cell's density
    /// @throws SetupError when the converter has no density
    si::Density density_at(const si::Position& point) const {
        const Cell state{ cell_at(point) };
        return state.density;
    }

    /// @brief Gives the number of cells of the domain, the bound of a loop over cell().
    /// @return Nx Ny Nz
    std::uint64_t cell_count() const {
        const std::uint64_t count{ lbm_.get_N() };
        return count;
    }

    /// @brief Gives the state of one cell, read on the host. Inline, so that a counted loop over the cells has no
    /// call through a function pointer and keeps no copy of the field.
    /// @param n the cell's index, from 0 to cell_count() - 1 (n = x + (y + z Ny) Nx, LBM::index())
    /// @return the cell's state
    /// @throws SetupError when the converter has no density
    Cell cell(std::uint64_t n) const {
        uint32_t x{ 0u };
        uint32_t y{ 0u };
        uint32_t z{ 0u };
        lbm_.coordinates(n, x, y, z);
        const Cell state{ state_of(n, x, y, z) };
        return state;
    }

private:
    LBM& lbm_;                       ///< the LBM whose fields are read (not owned)
    si::LatticeConverter converter_; ///< the simulation's conversions

    /// @brief Gives the index of the cell that contains a coordinate, clamped to the axis.
    /// @param cells a coordinate in cells from the origin corner
    /// @param count the cells along the axis, at least 1
    /// @return floor(cells), clamped to 0 to count - 1
    static uint32_t clamp_index(double cells, uint32_t count) {
        const double last{ static_cast<double>(count - 1u) };
        const double index{ std::clamp(std::floor(cells), 0.0, last) };
        const uint32_t whole{ static_cast<uint32_t>(index) };
        return whole;
    }

    /// @brief Gives a cell's state in physical units.
    /// @param n the cell's index
    /// @param x the cell along X
    /// @param y the cell along Y
    /// @param z the cell along Z
    /// @return the state
    /// @throws SetupError when the converter has no density
    Cell state_of(uint64_t n, uint32_t x, uint32_t y, uint32_t z) const {
        const double half_cell{ 0.5 };
        const double rho{ static_cast<double>(lbm_.rho[n]) };
        const double p_lattice{ (rho - 1.0) / 3.0 }; // the lattice pressure is (rho - 1)/3
        const si::Length centre_x{ converter_.si_length(static_cast<double>(x) + half_cell) };
        const si::Length centre_y{ converter_.si_length(static_cast<double>(y) + half_cell) };
        const si::Length centre_z{ converter_.si_length(static_cast<double>(z) + half_cell) };
        const si::Speed u_x{ converter_.si_velocity(static_cast<double>(lbm_.u.x[n])) };
        const si::Speed u_y{ converter_.si_velocity(static_cast<double>(lbm_.u.y[n])) };
        const si::Speed u_z{ converter_.si_velocity(static_cast<double>(lbm_.u.z[n])) };
        Cell state{};
        state.center = si::Position{ centre_x, centre_y, centre_z };
        state.velocity = si::Velocity{ u_x, u_y, u_z };
        state.pressure = converter_.si_pressure(p_lattice);
        state.density = converter_.si_density(rho);
        state.solid = (lbm_.flags[n] & TYPE_S) != 0u;
        return state;
    }
};
