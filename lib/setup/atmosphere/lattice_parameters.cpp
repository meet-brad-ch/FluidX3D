#include "setup/atmosphere/lattice_parameters.hpp"
#include "setup/core/setup_error.hpp"

#include <cstddef>
#include <cstdio>

LatticeParameters LatticeParameters::of(const CblDescription& description, const TransportBuild& build) {
    const si::LatticeConverter c{ description.converter() };
    const CblNumerics& n{ description.numerics() };
    const si::Length cell{ c.cell() };
    const si::Duration step{ c.step() };
    const si::Speed lattice_speed{ c.lattice_speed() };
    const float lattice_nu{ n.momentum.lattice_value(c) };
    const Diffusivity scalar{ n.scalar_for(build) };
    const float lattice_diffusivity{ scalar.lattice_value(c) };
    const float core_alpha{ ScalarTransportRules::core_alpha(lattice_diffusivity, build) };
    float tau_t{ 0.0f };
    if(!build.finite_volumes) {
        tau_t = ScalarTransportRules::relaxation_time(lattice_diffusivity);
    }
    const float lattice_inv_prandtl{ n.prandtl.lattice_inverse() };
    const si::Temperature theta_0{ c.theta_0() };
    const BoussinesqBuoyancy buoyancy(n.gravity, theta_0, n.carrier);
    const float lattice_fz{ buoyancy.lattice_force() };
    const float lattice_beta{ buoyancy.lattice_expansion(c) };
    const SurfaceForcing& surface{ description.surface() };
    const float lattice_heat_rate{ c.heat_rate(surface.flux) };
    float lattice_moisture_rate{ 0.0f };
    if(description.moist()) {
        const si::MoistureFlux moisture_flux{ surface.moisture_flux() };
        lattice_moisture_rate = c.moisture_rate(moisture_flux);
    }
    float lattice_nudge{ 0.0f };
    if(description.wind()) {
        const si::Duration nudge_tau{ description.wind()->nudge_tau };
        lattice_nudge = c.per_step(nudge_tau);
    }
    const LatticeParameters p{
        .dx = cell,
        .dt = step,
        .speed = lattice_speed,
        .nu = lattice_nu,
        .diffusivity = lattice_diffusivity,
        .alpha = core_alpha,
        .relaxation_time = tau_t,
        .inv_prandtl = lattice_inv_prandtl,
        .fz = lattice_fz,
        .beta = lattice_beta,
        .heat_rate = lattice_heat_rate,
        .moisture_rate = lattice_moisture_rate,
        .nudge_per_step = lattice_nudge,
        .build = build,
    };
    return p;
}

std::string LatticeParameters::describe() const {
    const double dx_m{ dx.in(au::meters) };
    const double dt_s{ dt.in(au::seconds) };
    const double speed_m_s{ speed.in(au::meters / au::second) };
    const double nu_double{ static_cast<double>(nu) };
    const double diffusivity_double{ static_cast<double>(diffusivity) };
    const double alpha_double{ static_cast<double>(alpha) };
    const bool finite_volumes{ build.finite_volumes };
    const char* const finite_volumes_text{ finite_volumes ? ", finite volumes" : "" };
    const double inv_prandtl_double{ static_cast<double>(inv_prandtl) };
    const double fz_double{ static_cast<double>(fz) };
    const double beta_double{ static_cast<double>(beta) };
    const double heat_rate_double{ static_cast<double>(heat_rate) };
    const bool moisture_set{ moisture_rate > 0.0f };
    const char* const moisture_text{ moisture_set ? ", moisture rate set" : "" };
    constexpr std::size_t text_capacity{ 320u };
    char text[text_capacity]{};
    const int length = std::snprintf(text, sizeof(text), "lattice: dx %.2f m, dt %.3f s (%.2f m/s), nu %.4g, D %.4g (alpha %.4g%s), 1/Pr_t %.3g, fz %.3g, beta %.4f, heat rate %.4e%s",
                  dx_m, dt_s, speed_m_s, nu_double, diffusivity_double, alpha_double, finite_volumes_text, inv_prandtl_double, fz_double, beta_double,
                  heat_rate_double, moisture_text);
    const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
    if(cut) {
        throw SetupError("LatticeParameters::describe(): the text does not fit its buffer");
    }
    const std::string description{ text };
    return description;
}
