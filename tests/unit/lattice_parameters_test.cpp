#include "setup/atmosphere/lattice_parameters.hpp"
#include "golden_capture.hpp"

#include <gtest/gtest.h>

namespace {

using namespace si::symbols;

/// @return the CBL example's build: finite volumes with the diffusivity fix, moisture, level forcing, Smagorinsky
TransportBuild example_build() {
    TransportBuild b{};
    b.diffusivity_fix = true;
    b.finite_volumes = true;
    b.moisture = true;
    b.level_forcing = true;
    b.subgrid = true;
    return b;
}

/// @return the example's quick preset (the time step its literal 0.04 s)
CblDescription quick() {
    CblNumerics numerics{};
    numerics.time_step = TimeStepRule::fixed(0.04 * s);
    CblDescription d = CblDescription();
    d = d.with_numerics(numerics);
    const si::Temperature theta_0 = si::kelvin_point(300.0);
    const IdealizedProfile profile = IdealizedProfile{ .inversion_base = 180.0 * m, .jump = 1.0 * K, .jump_depth = 0.0 * m, .lapse = 5.0 * K / km };
    const Sounding sounding = Sounding::idealized(theta_0, profile);
    d = d.with_sounding(sounding, 180.0 * m);
    d = d.with_cell(2.5 * m);
    const LidRule lid = LidRule::at(245.0 * m);
    const ColumnExtent extent = ColumnExtent{ .east = 320.0 * m, .north = 320.0 * m, .lid = lid };
    d = d.with_extent(extent);
    const HeatedPatch patch = HeatedPatch::disc(40.0 * m, 0.0 * m, PatchMode::HEATED_LAYERS, 4u);
    const SurfaceForcing forcing = SurfaceForcing{ .flux = 0.262 * K * m / s, .patch = patch, .bowen = 0.5 };
    d = d.with_surface(forcing);
    const StartNoise noise = StartNoise::per_cell(0.1 * K, 30.0 * m, 2u);
    d = d.with_noise(noise);
    const RunSchedule schedule = RunSchedule{ .spinup = 60.0 * s, .record = 40.0 * s, .interval = 10.0 * s };
    d = d.with_schedule(schedule);
    return d;
}

/// @param d a description
/// @param g its capture
/// @param name for the messages
void expect_the_capture(const CblDescription& d, const GoldenCapture& g, const std::string& name) {
    const TransportBuild build = example_build();
    const LatticeParameters p = LatticeParameters::of(d, build);
    const double dx = p.dx.in(m);
    const double expected_dx = g.setting("dx");
    EXPECT_EQ(dx, expected_dx) << name;
    const double dt = p.dt.in(s);
    const double expected_dt = g.setting("dt");
    EXPECT_EQ(dt, expected_dt) << name;
    const double speed = p.speed.in(m / s);
    const double expected_speed = g.setting("speed");
    EXPECT_EQ(speed, expected_speed) << name;
    const float expected_nu = (float)g.setting("nu");
    EXPECT_EQ(p.nu, expected_nu) << name;
    const float expected_alpha = (float)g.setting("alpha");
    EXPECT_EQ(p.alpha, expected_alpha) << name;
    const float expected_inv_prandtl = (float)g.setting("inv_prandtl_t");
    EXPECT_EQ(p.inv_prandtl, expected_inv_prandtl) << name;
    const float expected_fz = (float)g.setting("fz");
    EXPECT_EQ(p.fz, expected_fz) << name;
    const float expected_beta = (float)g.setting("beta");
    EXPECT_EQ(p.beta, expected_beta) << name;
    const float expected_heat_rate = (float)g.setting("heat_rate");
    EXPECT_EQ(p.heat_rate, expected_heat_rate) << name;
    if(g.setting("moist") != 0.0) {
        const double moisture_flux = d.surface().moisture_flux().in(kg_w / kg * m / s);
        const double expected_moisture_flux = g.setting("moisture_flux");
        EXPECT_EQ(moisture_flux, expected_moisture_flux) << name;
        const float expected_moisture_rate = (float)g.setting("moisture_rate");
        EXPECT_EQ(p.moisture_rate, expected_moisture_rate) << name;
    }
    if(g.setting("wind_speed") > 0.0) {
        const float expected_nudge = (float)g.setting("nudge_per_step");
        EXPECT_EQ(p.nudge_per_step, expected_nudge) << name;
    }
}

// the verified examples' lattice numbers bit for bit: the defaults (sp2011) and the quick preset
TEST(LatticeParameters, AreTheVerifiedExamples) {
    const CblDescription sp2011 = CblDescription();
    const GoldenCapture sp2011_capture = GoldenCapture("sp2011");
    expect_the_capture(sp2011, sp2011_capture, "sp2011");
    const CblDescription quick_case = quick();
    const GoldenCapture quick_capture = GoldenCapture("quick");
    expect_the_capture(quick_case, quick_capture, "quick");
    const TransportBuild build = example_build();
    const LatticeParameters p = LatticeParameters::of(quick_case, build);
    EXPECT_EQ(p.moisture_rate, 0.0f); // a dry run
    EXPECT_EQ(p.nudge_per_step, 0.0f); // no wind
    EXPECT_EQ(p.relaxation_time, 0.0f); // the finite volumes have no relaxation time
}

// the D3Q7 lattice: the floor 0.014, tauT 0.556, and without the fix the core gets 2 D
TEST(LatticeParameters, LatticeTransport) {
    TransportBuild d3q7{};
    d3q7.diffusivity_fix = true;
    const CblDescription defaults = CblDescription();
    const LatticeParameters fixed = LatticeParameters::of(defaults, d3q7);
    EXPECT_EQ(fixed.diffusivity, 0.014f);
    EXPECT_EQ(fixed.alpha, 0.014f);
    EXPECT_FLOAT_EQ(fixed.relaxation_time, 0.556f);
    const TransportBuild no_fix{};
    const LatticeParameters unfixed = LatticeParameters::of(defaults, no_fix);
    const float expected_alpha = 2.0f * 0.014f;
    EXPECT_EQ(unfixed.alpha, expected_alpha);
    const std::string described = fixed.describe();
    const std::size_t at = described.find("beta");
    EXPECT_NE(at, std::string::npos);
}

/// @param g the extra moist capture
/// @param csv its sounding
/// @param wind whether it is the windy case
/// @return the example's sounding preset as a description: 10 m cells, zi 1880 m, the lid at 2 zi and the sponge from
/// 1.6 zi (450 s), the capture's flux, a 400 m surface-layer patch, moisture, 300 s + 200 s by 20 s; with wind 5 m/s at
/// 100 m from the north (0.25, 600 s) and the patch 2000 m north of the middle
CblDescription sounding_case(const GoldenCapture& g, const std::string& csv, bool wind) {
    const si::Length cell = 10.0 * m;          // the preset's cell (the description's default)
    const double lid_in_zi = 2.0;
    const double sponge_in_zi = 1.6;
    const si::Length zi = g.setting("zi") * m;
    const si::Length east = cell * g.setting("Nx");
    const si::Length north = cell * g.setting("Ny");
    CblDescription d = CblDescription();
    const Sounding tabulated = Sounding::from_csv(csv);
    const Sounding sounding = tabulated.with_reference_from_mixed_layer(zi);
    d = d.with_sounding(sounding, zi);
    const LidRule lid = LidRule::in_inversion_heights(lid_in_zi);
    const ColumnExtent extent = ColumnExtent{ .east = east, .north = north, .lid = lid };
    d = d.with_extent(extent);
    const si::KinematicHeatFlux flux = g.setting("flux") * K * m / s;
    const si::Length patch_north = g.setting("patch_north") * m;
    const HeatedPatch patch = HeatedPatch::disc(400.0 * m, patch_north, PatchMode::SURFACE_LAYER);
    const SurfaceForcing forcing = SurfaceForcing{ .flux = flux, .patch = patch, .bowen = 0.5 };
    d = d.with_surface(forcing);
    const si::Length sponge_from = zi * sponge_in_zi;
    const SpongeProfile sponge = SpongeProfile::quadratic(sponge_from, 450.0 * s);
    d = d.with_sponge(sponge);
    const RunSchedule schedule = RunSchedule{ .spinup = 300.0 * s, .record = 200.0 * s, .interval = 20.0 * s };
    d = d.with_schedule(schedule);
    d = d.with_moisture(true);
    if(wind) {
        const PowerLawWind target = PowerLawWind(5.0 * m / s, 100.0 * m, 0.25, 0.0 * deg);
        const WindForcing forcing_wind = WindForcing{ .wind = target, .nudge_tau = 600.0 * s };
        d = d.with_wind(forcing_wind);
    }
    return d;
}

// the extra moist cases (outside the repository) (FLUIDX3D_EXTRA_CBL, FLUIDX3D_EXTRA_CBL_WIND, FLUIDX3D_EXTRA_SOUNDING): the
// description's grid, steps and lattice numbers against their captures
TEST(LatticeParameters, PrivateMoistCasesAreTheirCaptures) {
    const std::optional<std::string> csv = GoldenCapture::environment("FLUIDX3D_EXTRA_SOUNDING");
    if(!csv) {
        GTEST_SKIP() << "FLUIDX3D_EXTRA_SOUNDING is not set";
    }
    int cases = 0;
    for(const char* variable : { "FLUIDX3D_EXTRA_CBL", "FLUIDX3D_EXTRA_CBL_WIND" }) {
        const std::optional<std::string> folder = GoldenCapture::environment(variable);
        if(!folder) {
            continue;
        }
        const GoldenCapture g = GoldenCapture::at(*folder);
        const bool windy = g.setting("wind_speed") > 0.0;
        const CblDescription d = sounding_case(g, *csv, windy);
        d.validate();
        const double nz = (double)d.grid().levels().levels();
        const double expected_nz = g.setting("Nz");
        EXPECT_EQ(nz, expected_nz) << variable;
        const double spinup_steps = (double)d.spinup_steps();
        const double expected_spinup_steps = g.setting("steps_spinup");
        EXPECT_EQ(spinup_steps, expected_spinup_steps) << variable;
        const double record_steps = (double)d.record_steps();
        const double expected_record_steps = g.setting("steps_record");
        EXPECT_EQ(record_steps, expected_record_steps) << variable;
        const double theta_0 = si::in_kelvin(d.sounding().reference());
        const double expected_theta_0 = g.setting("theta_0");
        EXPECT_EQ(theta_0, expected_theta_0) << variable;
        expect_the_capture(d, g, variable);
        cases++;
    }
    if(cases == 0) {
        GTEST_SKIP() << "FLUIDX3D_EXTRA_CBL and FLUIDX3D_EXTRA_CBL_WIND are not set";
    }
}

} // namespace
