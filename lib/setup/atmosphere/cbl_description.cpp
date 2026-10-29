#include "setup/atmosphere/cbl_description.hpp"
#include "setup/core/setup_error.hpp"

#include <cmath>
#include <cstdio>

namespace {

/// @brief The values of CblDescription's defaults: Sullivan and Patton 2011's dry convective boundary layer
/// (J. Atmos. Sci. 68), at 10 m cells.
struct Sp2011 {
    static constexpr si::Length inversion = au::meters(974.0);                        ///< the inversion's base, zi at the start
    static constexpr si::TemperatureDifference jump = au::kelvins(8.0);               ///< the rise of theta across the inversion
    static constexpr si::Length jump_depth = au::meters(100.0);                       ///< the depth over which the jump rises
    static constexpr si::LapseRate lapse = (au::kelvins / au::meter)(0.003);          ///< the gradient of theta above the jump (3 K/km)
    static constexpr double theta_0_kelvin = 300.0;                                   ///< the mixed layer's potential temperature theta_0, in K
    static constexpr si::Length cell = au::meters(10.0);                              ///< the edge of one cubic cell
    static constexpr si::TemperatureDifference noise_amplitude = au::kelvins(0.1);    ///< the start noise's largest perturbation
    static constexpr si::Length noise_top = au::meters(300.0);                        ///< the height below which the cells get start noise
    static constexpr unsigned noise_seed = 2u;                                        ///< the start noise's random seed
};

/// The scalar diffusivity floor of the finite-volume transport (SCALAR_FV), in cells squared per time step: the
/// finite volumes have no diffusion of their own, and 1e-4 keeps the inversion sharp.
constexpr double finite_volume_floor = 1.0E-4;
/// The scalar diffusivity floor of the D3Q7 lattice transport, in cells squared per time step: next to TYPE_S walls
/// in a buoyant flow the lattice needs tauT 0.556, D 0.014 (tests/physics/heat_conservation.cpp).
constexpr double lattice_floor = 0.014;

/// Half (a ratio): the patch's radius from its diameter, and the box's middle from its cell counts.
constexpr double half = 0.5;

/// @brief Makes the default start state.
/// @return Sullivan and Patton 2011's idealized, dry sounding with theta_0 at 300 K
Sounding sp2011_sounding() {
    const si::Temperature theta_0{ si::kelvin_point(Sp2011::theta_0_kelvin) };
    const IdealizedProfile profile{
        .inversion_base = Sp2011::inversion,
        .jump = Sp2011::jump,
        .jump_depth = Sp2011::jump_depth,
        .lapse = Sp2011::lapse,
    };
    Sounding sounding{ Sounding::idealized(theta_0, profile) };
    return sounding;
}

/// @brief Makes the default start noise.
/// @return one draw per fluid cell below 300 m, at most 0.1 K, with seed 2
StartNoise sp2011_noise() {
    StartNoise noise{ StartNoise::per_cell(Sp2011::noise_amplitude, Sp2011::noise_top, Sp2011::noise_seed) };
    return noise;
}

} // namespace

LidRule LidRule::at(si::Length ground_to_lid) {
    if(!(ground_to_lid > au::ZERO)) {
        throw SetupError("LidRule::at(): the lid's height must be more than 0 m");
    }
    const LidRule rule(ground_to_lid, 0.0);
    return rule;
}

LidRule LidRule::in_inversion_heights(double k) {
    if(!(k > 1.0)) {
        throw SetupError("LidRule::in_inversion_heights(): the lid must be above the inversion (k more than 1)");
    }
    const LidRule rule(std::nullopt, k);
    return rule;
}

si::Length LidRule::height(si::Length zi) const {
    si::Length lid{ au::ZERO };
    if(height_) {
        lid = *height_;
    } else {
        lid = zi * k_;
    }
    return lid;
}

std::string LidRule::describe() const {
    constexpr std::size_t text_capacity{ 80u };
    char text[text_capacity]{};
    int length{ 0 };
    if(height_) {
        const double height_m{ height_->in(au::meters) };
        length = std::snprintf(text, sizeof(text), "the lid at %.0f m", height_m);
    } else {
        const double k{ k_ };
        length = std::snprintf(text, sizeof(text), "the lid at %.2f zi", k);
    }
    const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
    if(cut) {
        throw SetupError("LidRule::describe(): the text does not fit its buffer");
    }
    const std::string description{ text };
    return description;
}

si::MoistureFlux SurfaceForcing::moisture_flux() const {
    if(!(bowen > 0.0)) {
        throw SetupError("SurfaceForcing: the Bowen ratio must be more than 0");
    }
    // E = H cp / (Lv B): K m/s * J/(kg K) / (J/kg of water) is kg of water per kg of air * m/s
    const auto flux_cp = flux * si::PhysicalConstants::dry_air_heat_capacity;
    const auto latent_bowen = si::PhysicalConstants::latent_heat * bowen;
    const si::MoistureFlux evaporation = flux_cp / latent_bowen;
    return evaporation;
}

Diffusivity CblNumerics::scalar_for(const TransportBuild& build) const {
    std::optional<Diffusivity> chosen{ scalar };
    if(!chosen) {
        const bool finite_volumes{ build.finite_volumes };
        const double floor_value{ finite_volumes ? finite_volume_floor : lattice_floor };
        const si::LatticeDiffusivity floor_diffusivity(floor_value);
        chosen = Diffusivity::lattice(floor_diffusivity);
    }
    const Diffusivity diffusivity{ *chosen };
    return diffusivity;
}

CblDescription::CblDescription()
    : sounding_(sp2011_sounding()), inversion_(Sp2011::inversion), cell_(Sp2011::cell), sponge_(SpongeProfile::none()), noise_(sp2011_noise()) {}

CblDescription CblDescription::with_sounding(Sounding sounding, si::Length inversion) const {
    if(!(inversion > au::ZERO)) {
        throw SetupError("CblDescription: the inversion's height must be more than 0 m");
    }
    CblDescription d = *this;
    d.sounding_ = std::move(sounding);
    d.inversion_ = inversion;
    return d;
}

CblDescription CblDescription::with_cell(si::Length cell) const {
    if(!(cell > au::ZERO)) {
        throw SetupError("CblDescription: the cell size must be more than 0 m");
    }
    CblDescription d = *this;
    d.cell_ = cell;
    return d;
}

CblDescription CblDescription::with_extent(ColumnExtent extent) const {
    CblDescription d = *this;
    d.extent_ = extent;
    return d;
}

CblDescription CblDescription::with_surface(SurfaceForcing surface) const {
    CblDescription d = *this;
    d.surface_ = surface;
    return d;
}

CblDescription CblDescription::with_wind(std::optional<WindForcing> wind) const {
    CblDescription d = *this;
    d.wind_ = std::move(wind);
    return d;
}

CblDescription CblDescription::with_sponge(SpongeProfile sponge) const {
    CblDescription d = *this;
    d.sponge_ = sponge;
    return d;
}

CblDescription CblDescription::with_numerics(CblNumerics numerics) const {
    CblDescription d = *this;
    d.numerics_ = numerics;
    return d;
}

CblDescription CblDescription::with_schedule(RunSchedule schedule) const {
    CblDescription d = *this;
    d.schedule_ = schedule;
    return d;
}

CblDescription CblDescription::with_noise(StartNoise noise) const {
    CblDescription d = *this;
    d.noise_ = noise;
    return d;
}

CblDescription CblDescription::with_moisture(bool moist) const {
    CblDescription d = *this;
    d.moist_ = moist;
    return d;
}

CblDescription CblDescription::with_domains(std::optional<uint3> domains) const {
    CblDescription d = *this;
    d.domains_ = domains;
    return d;
}

void CblDescription::validate() const {
    (void)sounding_.reference(); // a tabulated sounding without theta_0 throws here
    validate_times();
    const AtmosphereGrid g{ grid() };
    validate_column(g);
    validate_patch(g);
    validate_forcing();
    const bool has_water{ sounding_.has_water() };
    if(moist_ && !has_water) {
        throw SetupError("CblDescription: a moist run needs total water in the sounding");
    }
    const bool split_along_z{ domains_ && domains_->z != 1u };
    if(split_along_z) {
        throw SetupError("CblDescription: the levels stay whole in each domain (no split along z)");
    }
}

void CblDescription::validate_times() const {
    const std::uint64_t interval{ interval_steps() };
    if(interval == 0u) {
        throw SetupError("CblDescription: the interval is shorter than half a time step");
    }
    const std::uint64_t spinup{ spinup_steps() };
    const std::uint64_t record{ record_steps() };
    const bool spinup_whole{ spinup % interval == 0u };
    const bool record_whole{ record % interval == 0u };
    if(!spinup_whole || !record_whole) {
        throw SetupError("CblDescription: the spin-up and the recorded time must be whole intervals");
    }
}

void CblDescription::validate_column(const AtmosphereGrid& g) const {
    const si::Length max_cell{ inversion_ * max_cell_over_zi };
    if(!(cell_ <= max_cell)) {
        const double inversion_m{ inversion_.in(au::meters) };
        const std::string inversion_text{ std::to_string(inversion_m) };
        const std::string message{ "CblDescription: the cell must be at most zi / 10 (zi " + inversion_text + " m)" };
        throw SetupError(message);
    }
    const si::Length lid{ g.levels().lid_height() };
    if(!(lid > inversion_)) {
        throw SetupError("CblDescription: the lid must be above the inversion");
    }
    if(sponge_.active()) {
        const si::LatticeConverter run_converter{ converter() };
        const float rate_at_lid{ sponge_.lattice_rate(lid, lid, run_converter) };
        if(!(rate_at_lid > 0.0f)) {
            throw SetupError("CblDescription: the sponge must start below the lid");
        }
    }
}

void CblDescription::validate_patch(const AtmosphereGrid& g) const {
    if(!surface_.patch.active()) {
        return;
    }
    const si::Length r{ surface_.patch.diameter() * half };
    const double ny_double{ static_cast<double>(g.ny()) };
    const double half_ny{ half * ny_double };
    const si::Length middle_north{ cell_ * half_ny };
    const si::Length north{ middle_north + surface_.patch.north_of_middle() };
    const double nx_double{ static_cast<double>(g.nx()) };
    const double half_nx{ half * nx_double };
    const si::Length east_half{ cell_ * half_nx };
    const si::Length north_extent{ cell_ * ny_double };
    const si::Length south_edge{ north - r };
    const si::Length north_edge{ north + r };
    const bool too_wide{ r > east_half };
    const bool past_south{ south_edge < au::ZERO };
    const bool past_north{ north_edge > north_extent };
    if(too_wide || past_south || past_north) {
        throw SetupError("CblDescription: the heated patch is not inside the box");
    }
}

void CblDescription::validate_forcing() const {
    if(!(surface_.flux > au::ZERO)) {
        throw SetupError("CblDescription: the floor's heat flux must be more than 0 K m/s");
    }
    if(wind_) {
        const si::Duration nudge_tau{ wind_->nudge_tau };
        if(!(nudge_tau > au::ZERO)) {
            throw SetupError("CblDescription: the nudging's time scale must be more than 0 s");
        }
    }
    const si::Temperature theta_0{ sounding_.reference() };
    const BoussinesqBuoyancy buoyancy(numerics_.gravity, theta_0, numerics_.carrier); // throws for gravity or carrier not above 0
    const StartNoise::Sequence noise_values{ noise_.sequence(cell_) };                   // throws for a block that the cells do not divide
}

AtmosphereGrid CblDescription::grid() const {
    const si::Length lid{ extent_.lid.height(inversion_) };
    const AtmosphereGrid cells{ AtmosphereGrid::of(extent_.east, extent_.north, lid, cell_) };
    return cells;
}

si::Duration CblDescription::time_step() const {
    const si::Duration step{ numerics_.time_step.step(cell_) };
    return step;
}

si::LatticeConverter CblDescription::converter() const {
    const si::Duration step{ time_step() };
    const si::Temperature theta_0{ sounding_.reference() };
    const si::LatticeConverter run_converter(cell_, step, theta_0);
    return run_converter;
}

// the steps of the core's time step (the converter's float step), so AtmosphereSimulation (the middle layer's hub)
// counts the same steps
std::uint64_t CblDescription::spinup_steps() const {
    const si::Duration step{ time_step() };
    const si::LatticeConverter core_steps(cell_, step);
    const std::uint64_t steps{ core_steps.steps(schedule_.spinup) };
    return steps;
}

std::uint64_t CblDescription::record_steps() const {
    const si::Duration step{ time_step() };
    const si::LatticeConverter core_steps(cell_, step);
    const std::uint64_t steps{ core_steps.steps(schedule_.record) };
    return steps;
}

std::uint64_t CblDescription::interval_steps() const {
    const si::Duration step{ time_step() };
    const si::LatticeConverter core_steps(cell_, step);
    const std::uint64_t steps{ core_steps.steps(schedule_.interval) };
    return steps;
}

RunEstimate CblDescription::estimate(const LatticeMemory& lattice, std::optional<si::LatticeUpdateRate> speed) const {
    const AtmosphereGrid cells{ grid() };
    const GridSize size{ cells.size() };
    const uint3 split{ domains_.value_or(uint3(1u, 1u, 1u)) };
    const std::uint64_t spinup{ spinup_steps() };
    const std::uint64_t record{ record_steps() };
    const std::uint64_t steps{ spinup + record };
    const RunEstimate run{ RunEstimate::of(size, split, lattice, steps, speed) };
    return run;
}

void CblDescription::visit(DescriptionVisitor& visitor) const {
    visitor.sounding(sounding_, inversion_);
    const AtmosphereGrid cells{ grid() };
    visitor.column(extent_, cells);
    visitor.surface(surface_);
    visitor.wind(wind_);
    visitor.sponge(sponge_);
    const si::Duration step{ time_step() };
    visitor.numerics(numerics_, step);
    visitor.schedule(schedule_);
    visitor.noise(noise_);
    visitor.moisture(moist_);
}
