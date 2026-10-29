#pragma once

#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"

#include <cstdint>

/// @brief The Boussinesq buoyancy of an atmosphere in the core's form: the lattice force is the carrier fz (a tiny
/// constant volume force down z) plus -fz beta (T - T_ref), so a cell warmer than its level's reference rises with
/// g theta'/theta_0.
///
/// The core multiplies the buoyancy by the volume force (`kernel.cpp`: fzn -= fz*def_beta*(T_buoy-T_ref)), so the
/// force needs a carrier: a constant fz small enough to leave the density almost uniform (the hydrostatic ramp
/// rho = 1 + 3 fz (z - the middle) stays within density_limit over the column), and beta = g dt^2/(dx theta_0 |fz|) per
/// lattice temperature unit. The carrier is a stated lattice constant (1e-6 by default), not derived: what matters is
/// |fz| beta, which does not depend on it.
/// @code
/// using namespace si::symbols;
/// const BoussinesqBuoyancy b(9.81 * m / (s * s), si::kelvin_point(300.0));
/// const float fz = b.lattice_force(), beta = b.lattice_expansion(converter);
/// @endcode
class BoussinesqBuoyancy {
public:
    /// The carrier's default size in lattice units (the CBL example's |fz|).
    static constexpr double default_carrier = 1.0E-6;
    /// The largest relative density range over the column that the carrier may cause (1 %, a ratio).
    static constexpr double density_limit = 0.01;
    /// The factor of the carrier's hydrostatic density ramp: the lattice density changes by 3 times the carrier per
    /// level (the lattice pressure is (ρ - 1)/3).
    static constexpr double ramp_per_level = 3.0;

    /// @brief Makes the buoyancy of an atmosphere with a reference potential temperature.
    /// @param gravity the gravitational acceleration, more than 0
    /// @param reference theta_0: the potential temperature that the buoyancy is relative to, more than 0 K
    /// @param carrier the carrier's size in lattice units, more than 0 (the force acts down z)
    /// @throws SetupError when the gravity, theta_0 or the carrier is not more than 0
    BoussinesqBuoyancy(si::Acceleration gravity, si::Temperature reference, double carrier = default_carrier);

    /// @brief Gives the volume force that the core takes (the fz of LBM's constructor): the carrier, down z.
    /// @return -carrier in lattice units
    float lattice_force() const {
        const double down{ -carrier_ };
        const float force{ static_cast<float>(down) };
        return force;
    }

    /// @brief Gives the thermal expansion coefficient that the core takes (def_beta), so that fz beta (T - T_ref)
    /// is the lattice acceleration g theta'/theta_0.
    ///
    /// The lattice temperature unit is the kernel's 1 K (LatticeConverter::kernel_temperature_unit); the converter
    /// must have this unit.
    /// @param converter the simulation's conversions (its cell, its time step and its temperature unit of 1 K)
    /// @return g dt²/(dx theta_0 carrier) per kelvin, in lattice units
    /// @throws SetupError when the converter's temperature unit is not 1 K
    float lattice_expansion(const si::LatticeConverter& converter) const;

    /// @brief Gives an upper bound of the relative density range that the carrier's hydrostatic balance makes over a
    /// column: 3 carrier per level, counted over all levels (the range between the end cells' centres is
    /// 3 carrier (levels - 1)).
    /// @param levels the column's levels (its cells along z, with the floor and the lid)
    /// @return 3 times the carrier times the levels (a ratio)
    double density_ramp(std::uint32_t levels) const {
        const double level_count{ static_cast<double>(levels) };
        const double per_level{ ramp_per_level * carrier_ };
        const double ramp{ per_level * level_count };
        return ramp;
    }

    /// @brief Refuses a column so tall that the carrier's density ramp is more than density_limit.
    /// @param levels the column's levels (its cells along z, with the floor and the lid)
    /// @throws SetupError when density_ramp(levels) is more than density_limit (the message suggests a smaller
    /// carrier)
    void validate(std::uint32_t levels) const;

    /// @return the gravitational acceleration
    si::Acceleration gravity() const { return gravity_; }
    /// @return theta_0, the potential temperature that the buoyancy is relative to
    si::Temperature reference() const { return reference_; }
    /// @return the carrier's size in lattice units (|fz|)
    double carrier() const { return carrier_; }

private:
    si::Acceleration gravity_{ au::ZERO };            ///< the gravitational acceleration g
    si::Temperature reference_{ si::kelvin_point(0.0) }; ///< theta_0, the reference potential temperature
    double carrier_{};                                ///< the carrier's size |fz| in lattice units
};
