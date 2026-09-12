#include "core/unit_scale.hpp"

#include <algorithm>

UnitScale UnitScale::from_reference(Length si_length, float lbm_length, Speed si_speed, float lbm_speed, Density si_density) {
    // same order of operations as Units::set_m_kg_s(x, u, rho=1, si_x, si_u, si_rho), so both give identical floats
    UnitScale scale;
    scale.unit_m_ = si_length.si() / lbm_length;
    scale.unit_kg_ = si_density.si() / 1.0f * (scale.unit_m_ * scale.unit_m_ * scale.unit_m_);
    scale.unit_s_ = lbm_speed / si_speed.si() * scale.unit_m_;
    return scale;
}

float UnitScale::length(Length x) const { return x.si() / unit_m_; }
float UnitScale::velocity(Speed u) const { return u.si() * unit_s_ / unit_m_; }
float UnitScale::acceleration(Acceleration a) const { return a.si() / unit_m_ * (unit_s_ * unit_s_); }
float UnitScale::viscosity(KinematicViscosity nu) const { return nu.si() * unit_s_ / (unit_m_ * unit_m_); }
float UnitScale::surface_tension(SurfaceTension sigma) const { return sigma.si() * (unit_s_ * unit_s_) / unit_kg_; }

std::uint64_t UnitScale::time_steps(Duration t) const {
    return static_cast<std::uint64_t>(std::max(t.si() / unit_s_ + 0.5f, 0.5f)); // as the core's to_ulong()
}

Length UnitScale::si_length(float cells) const { return Length::from_si(cells * unit_m_); }
Speed UnitScale::si_velocity(float u) const { return Speed::from_si(u * unit_m_ / unit_s_); }
Duration UnitScale::si_time(std::uint64_t steps) const { return Duration::from_si(static_cast<float>(steps) * unit_s_); }
Force UnitScale::si_force(float f) const { return Force::from_si(f * unit_kg_ * unit_m_ / (unit_s_ * unit_s_)); }
