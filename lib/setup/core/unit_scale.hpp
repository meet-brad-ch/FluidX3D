#pragma once

#include "setup/core/quantity.hpp"
#include <cstdint>

// Scale between SI units and lattice units, as a value: 1 cell is cell_size() long, 1 time step lasts time_step(),
// and the lattice density 1 is density(). Built from one reference length, speed and density in both unit systems
// (the same definition as the core's Units::set_m_kg_s, which the simulation layer is given from this).
class UnitScale {
public:
    // si_length is lbm_length cells long, si_speed is lbm_speed in lattice units, si_density is the lattice density 1
    static UnitScale from_reference(Length si_length, float lbm_length, Speed si_speed, float lbm_speed, Density si_density);

    Length cell_size() const { return Length::from_si(unit_m_); }
    Duration time_step() const { return Duration::from_si(unit_s_); }
    Mass mass_unit() const { return Mass::from_si(unit_kg_); }   // mass of one cell at lattice density 1
    Density density() const { return Density::from_si(unit_kg_ / (unit_m_ * unit_m_ * unit_m_)); }

    // SI -> lattice units
    float length(Length x) const;                       // cells
    float velocity(Speed u) const;
    float acceleration(Acceleration a) const;           // for gravity also the volume force rho*g (lattice density 1)
    float viscosity(KinematicViscosity nu) const;       // also thermal diffusivity
    float surface_tension(SurfaceTension sigma) const;
    std::uint64_t time_steps(Duration t) const;         // rounded to the nearest step, at least 0

    // lattice units -> SI
    Length si_length(float cells) const;
    Speed si_velocity(float u) const;
    Duration si_time(std::uint64_t steps) const;
    Force si_force(float f) const;

private:
    float unit_m_ = 1.0f;  // m per cell
    float unit_kg_ = 1.0f; // kg per lattice mass unit
    float unit_s_ = 1.0f;  // s per time step
};
