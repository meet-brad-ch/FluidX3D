#pragma once

#include "setup/core/quantity.hpp"
#include <cstdint>

/// @brief Scale between SI units and lattice units, as a value: 1 cell is cell_size() long, 1 time step lasts
/// time_step(), and the lattice density 1 is density().
///
/// Built from one reference length, speed and density in both unit systems (the same definition as the core's
/// Units::set_m_kg_s, which Simulation sets from this). The builders convert with it.
class UnitScale {
public:
    /// @brief The scale on which a length, a speed and a density have given lattice values.
    /// @param si_length the reference length
    /// @param lbm_length the reference length in cells
    /// @param si_speed the reference speed
    /// @param lbm_speed the reference speed in cells per time step
    /// @param si_density the fluid's density, which is the lattice density 1
    /// @return the scale
    static UnitScale from_reference(Length si_length, float lbm_length, Speed si_speed, float lbm_speed, Density si_density);

    /// @return the size of one cell
    Length cell_size() const { return Length::from_si(unit_m_); }

    /// @return the duration of one time step
    Duration time_step() const { return Duration::from_si(unit_s_); }

    /// @return the mass of one cell at the lattice density 1
    Mass mass_unit() const { return Mass::from_si(unit_kg_); }

    /// @return the density that is the lattice density 1
    Density density() const { return Density::from_si(unit_kg_ / (unit_m_ * unit_m_ * unit_m_)); }

    /// @name SI to lattice units
    /// @{

    /// @param x a length
    /// @return the length in cells
    float length(Length x) const;

    /// @param u a speed
    /// @return the speed in cells per time step
    float velocity(Speed u) const;

    /// @param a an acceleration
    /// @return the acceleration in lattice units; for gravity also the volume force rho*g (lattice density 1)
    float acceleration(Acceleration a) const;

    /// @param nu a kinematic viscosity, or a thermal diffusivity
    /// @return the viscosity in lattice units
    float viscosity(KinematicViscosity nu) const;

    /// @param sigma a surface tension
    /// @return the surface tension in lattice units
    float surface_tension(SurfaceTension sigma) const;

    /// @param p a pressure difference from the fluid at rest
    /// @return the pressure in lattice units (the lattice pressure is (rho-1)/3)
    float pressure(Pressure p) const;

    /// @param t a duration
    /// @return the duration in time steps, rounded to the nearest step
    std::uint64_t time_steps(Duration t) const;
    /// @}

    /// @name lattice units to SI
    /// @{

    /// @param cells a length in cells
    /// @return the length
    Length si_length(float cells) const;

    /// @param u a speed in cells per time step
    /// @return the speed
    Speed si_velocity(float u) const;

    /// @param steps a number of time steps
    /// @return their duration
    Duration si_time(std::uint64_t steps) const;

    /// @param f a force in lattice units
    /// @return the force
    Force si_force(float f) const;

    /// @param p a pressure in lattice units, (rho-1)/3
    /// @return the pressure difference from the fluid at rest
    Pressure si_pressure(float p) const;

    /// @param rho a density in lattice units
    /// @return the density
    Density si_density(float rho) const;

    /// @param sigma a surface tension in lattice units
    /// @return the surface tension
    SurfaceTension si_surface_tension(float sigma) const;
    /// @}

private:
    float unit_m_ = 1.0f;  ///< m per cell
    float unit_kg_ = 1.0f; ///< kg per lattice mass unit
    float unit_s_ = 1.0f;  ///< s per time step
};
