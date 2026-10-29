#include "setup/atmosphere/cbl_description.hpp"
#include "setup/core/setup_error.hpp"
#include "golden_capture.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace {

using namespace si::symbols;

/// @brief The example's quick preset as a description: 320 m square at 2.5 m cells, the lid at 245 m, 300 K to
/// 180 m, +1 K, 5 K/km above, 0.262 K m/s, a 40 m patch of 4 heated layers, noise below 30 m, 60 s + 40 s by 10 s,
/// the time step 0.04 s (the example's literal, not its rule).
/// @return the description
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

/// @brief Expects that a description gives the grid, the step, the step counts, the flux, theta_0 and zi of a golden
/// capture's settings.
/// @param d a description
/// @param name the golden case's folder name in tests/golden/cbl
void expect_the_capture(const CblDescription& d, const char* name) {
    const GoldenCapture g(name);
    d.validate();
    const AtmosphereGrid grid = d.grid();
    const double nx = (double)grid.nx();
    const double expected_nx = g.setting("Nx");
    EXPECT_EQ(nx, expected_nx) << name;
    const double ny = (double)grid.ny();
    const double expected_ny = g.setting("Ny");
    EXPECT_EQ(ny, expected_ny) << name;
    const double nz = (double)grid.levels().levels();
    const double expected_nz = g.setting("Nz");
    EXPECT_EQ(nz, expected_nz) << name;
    const double dx = d.cell().in(m);
    const double expected_dx = g.setting("dx");
    EXPECT_EQ(dx, expected_dx) << name;
    const double dt = d.converter().step().in(s);
    const double expected_dt = g.setting("dt");
    EXPECT_EQ(dt, expected_dt) << name; // the core's step
    const double interval_steps = (double)d.interval_steps();
    const double expected_interval_steps = g.setting("steps_interval");
    EXPECT_EQ(interval_steps, expected_interval_steps) << name;
    const double spinup_steps = (double)d.spinup_steps();
    const double expected_spinup_steps = g.setting("steps_spinup");
    EXPECT_EQ(spinup_steps, expected_spinup_steps) << name;
    const double record_steps = (double)d.record_steps();
    const double expected_record_steps = g.setting("steps_record");
    EXPECT_EQ(record_steps, expected_record_steps) << name;
    const double flux = d.surface().flux.in(K * m / s);
    const double expected_flux = g.setting("flux");
    EXPECT_EQ(flux, expected_flux) << name;
    const double theta_0 = si::in_kelvin(d.sounding().reference());
    const double expected_theta_0 = g.setting("theta_0");
    EXPECT_EQ(theta_0, expected_theta_0) << name;
    const double zi = d.inversion().in(m);
    const double expected_zi = g.setting("zi");
    EXPECT_EQ(zi, expected_zi) << name;
}

// the defaults are Sullivan and Patton 2011's dry CBL at 10 m: the verified sp2011 capture's grid, step and times
TEST(CblDescription, DefaultsAreSullivanPatton) {
    const CblDescription d;
    expect_the_capture(d, "sp2011");
    EXPECT_FALSE(d.surface().patch.active());
    EXPECT_FALSE(d.wind().has_value());
    EXPECT_FALSE(d.sponge().active());
    EXPECT_FALSE(d.moist());
}

// the quick preset: the verified quick capture's grid, step (0.04 s at 2.5 m) and times
TEST(CblDescription, QuickIsItsCapture) {
    const CblDescription d = quick();
    expect_the_capture(d, "quick");
}

// the lid rules: a height, or k zi (the example's lround(2 zi / cell) + 2 levels)
TEST(CblDescription, LidRules) {
    const double fixed_lid = LidRule::at(2048.0 * m).height(974.0 * m).in(m);
    EXPECT_EQ(fixed_lid, 2048.0);
    const double two_zi_lid = LidRule::in_inversion_heights(2.0).height(1880.0 * m).in(m);
    EXPECT_EQ(two_zi_lid, 3760.0);
    const si::Temperature theta_0 = si::kelvin_point(300.0);
    const IdealizedProfile profile = IdealizedProfile{ .inversion_base = 1880.0 * m, .jump = 1.0 * K, .jump_depth = 0.0 * m, .lapse = 5.0 * K / km };
    const Sounding sounding = Sounding::idealized(theta_0, profile);
    const LidRule lid = LidRule::in_inversion_heights(2.0);
    const ColumnExtent extent = ColumnExtent{ .east = 1920.0 * m, .north = 1920.0 * m, .lid = lid };
    CblDescription moist = CblDescription();
    moist = moist.with_sounding(sounding, 1880.0 * m);
    moist = moist.with_extent(extent);
    const std::uint32_t levels = moist.grid().levels().levels();
    EXPECT_EQ(levels, 378u); // the extra moist case's Nz
    EXPECT_THROW(LidRule::at(0.0 * m), SetupError);
    EXPECT_THROW(LidRule::in_inversion_heights(1.0), SetupError);
}

// a description text that does not fit its buffer is an error, not a cut text (the 301 digits of 1e300 m)
TEST(CblDescription, DescribeRefusesACutText) {
    const std::string lid_text = LidRule::at(2048.0 * m).describe();
    EXPECT_EQ(lid_text, "the lid at 2048 m");
    const LidRule far_lid = LidRule::at(1.0E300 * m);
    EXPECT_THROW(far_lid.describe(), SetupError);
    const SpongeProfile far_sponge = SpongeProfile::quadratic(1.0E300 * m, 600.0 * s);
    EXPECT_THROW(far_sponge.describe(), SetupError);
}

/// @brief Expects that a description fails its validation with a message that names a part.
/// @param d the description
/// @param part the text the message holds
void expect_fault(const CblDescription& d, const std::string& part) {
    try {
        d.validate();
        ADD_FAILURE() << "no error, expected: " << part;
    } catch(const SetupError& e) {
        const std::string message = std::string(e.what());
        const std::size_t at = message.find(part);
        EXPECT_NE(at, std::string::npos) << e.what();
    }
}

/// @brief Makes the quick preset with another heated patch.
/// @param patch a heated patch
/// @return the quick preset with that patch (0.262 K m/s, the Bowen ratio 0.5)
CblDescription quick_with_patch(const HeatedPatch& patch) {
    const SurfaceForcing forcing = SurfaceForcing{ .flux = 0.262 * K * m / s, .patch = patch, .bowen = 0.5 };
    const CblDescription d = quick().with_surface(forcing);
    return d;
}

// each fault of a description, named
TEST(CblDescription, ValidateNamesEachFault) {
    EXPECT_NO_THROW(quick().validate());
    const RunSchedule part_interval = RunSchedule{ .spinup = 65.0 * s, .record = 40.0 * s, .interval = 10.0 * s };
    const CblDescription not_whole_intervals = quick().with_schedule(part_interval);
    expect_fault(not_whole_intervals, "whole intervals");
    const RunSchedule coarse_schedule = RunSchedule{ .spinup = 60.0 * s, .record = 40.0 * s, .interval = 20.0 * s };
    const CblDescription coarse = quick().with_cell(20.0 * m);
    const CblDescription coarse_cells = coarse.with_schedule(coarse_schedule);
    expect_fault(coarse_cells, "zi / 10");
    const LidRule low_lid = LidRule::at(150.0 * m);
    const ColumnExtent low_extent = ColumnExtent{ .east = 320.0 * m, .north = 320.0 * m, .lid = low_lid };
    const CblDescription lid_below = quick().with_extent(low_extent);
    expect_fault(lid_below, "above the inversion");
    const SpongeProfile high_sponge = SpongeProfile::quadratic(400.0 * m, 450.0 * s);
    const CblDescription sponge_above = quick().with_sponge(high_sponge);
    expect_fault(sponge_above, "sponge");
    const HeatedPatch wide_patch = HeatedPatch::disc(400.0 * m, 0.0 * m, PatchMode::SURFACE_LAYER);
    const CblDescription too_wide = quick_with_patch(wide_patch);
    expect_fault(too_wide, "not inside the box");
    const HeatedPatch off_patch = HeatedPatch::disc(40.0 * m, 150.0 * m, PatchMode::SURFACE_LAYER);
    const CblDescription off_centre = quick_with_patch(off_patch);
    expect_fault(off_centre, "not inside the box");
    const CblDescription dry_moist = quick().with_moisture(true);
    expect_fault(dry_moist, "total water");
    const uint3 z_split = uint3(1u, 1u, 2u);
    const CblDescription split_z = quick().with_domains(z_split);
    expect_fault(split_z, "no split along z");
    const si::Temperature t = si::kelvin_point(300.0);
    const std::vector<TemperaturePoint> points{ { 0.0 * m, t }, { 500.0 * m, t } };
    const TemperatureProfile thl = TemperatureProfile::from_points(points);
    const Sounding no_reference = Sounding::tabulated(thl, std::nullopt, std::nullopt);
    const CblDescription without_theta_0 = quick().with_sounding(no_reference, 180.0 * m);
    expect_fault(without_theta_0, "theta_0");
    const SpongeProfile low_sponge = SpongeProfile::quadratic(200.0 * m, 450.0 * s);
    const CblDescription sponge_below = quick().with_sponge(low_sponge);
    EXPECT_NO_THROW(sponge_below.validate());
    EXPECT_THROW(CblDescription().with_cell(0.0 * m), SetupError);
    const Sounding default_sounding = CblDescription().sounding();
    EXPECT_THROW(CblDescription().with_sounding(default_sounding, 0.0 * m), SetupError);
}

// the forcing and the numerics that the run divides by or scales with, refused before the build
TEST(CblDescription, ValidateRefusesForcingAndNumericsNotAboveZero) {
    const PowerLawWind target = PowerLawWind(5.0 * m / s, 100.0 * m, 0.25, 0.0 * deg);
    const WindForcing no_time_scale = WindForcing{ .wind = target, .nudge_tau = 0.0 * s };
    const CblDescription instant_nudging = quick().with_wind(no_time_scale);
    expect_fault(instant_nudging, "nudging");
    const StartNoise part_cells = StartNoise::blocks(0.1 * K, 30.0 * m, 2u, 3.0 * m);
    const CblDescription noise_in_part_cells = quick().with_noise(part_cells);
    expect_fault(noise_in_part_cells, "divide the block");
    CblNumerics no_gravity_numerics = quick().numerics();
    no_gravity_numerics.gravity = 0.0 * m / (s * s);
    const CblDescription no_gravity = quick().with_numerics(no_gravity_numerics);
    expect_fault(no_gravity, "gravity");
    CblNumerics no_carrier_numerics = quick().numerics();
    no_carrier_numerics.carrier = 0.0;
    const CblDescription no_carrier = quick().with_numerics(no_carrier_numerics);
    expect_fault(no_carrier, "carrier");
    const SurfaceForcing cold_floor = SurfaceForcing{ .flux = 0.0 * K * m / s, .patch = HeatedPatch::none(), .bowen = 0.5 };
    const CblDescription no_heating = quick().with_surface(cold_floor);
    expect_fault(no_heating, "heat flux");
}

// the scalar diffusivity: the given one, or the build's floor
TEST(CblDescription, ScalarDiffusivityFloors) {
    const si::LatticeConverter converter = CblDescription().converter();
    TransportBuild fv{};
    fv.finite_volumes = true;
    const CblNumerics defaults{};
    const float fv_floor = defaults.scalar_for(fv).lattice_value(converter);
    EXPECT_EQ(fv_floor, 1.0E-4f);
    const TransportBuild lattice_build{};
    const float lattice_floor = defaults.scalar_for(lattice_build).lattice_value(converter);
    EXPECT_EQ(lattice_floor, 0.014f);
    CblNumerics given{};
    given.scalar = Diffusivity::lattice(si::LatticeDiffusivity(3.0E-4));
    const float given_value = given.scalar_for(fv).lattice_value(converter);
    EXPECT_EQ(given_value, 3.0E-4f);
}

// the estimate: the grid's cells and the spin-up and recorded steps
TEST(CblDescription, Estimate) {
    const LatticeMemory lattice = LatticeMemory{ au::bytes(100.0) };
    const RunEstimate e = CblDescription().estimate(lattice, std::nullopt);
    const unsigned long long expected_cells = 128ull * 128ull * 207ull;
    EXPECT_EQ(e.cells, expected_cells);
    const unsigned long long expected_steps = 7500ull + 11250ull;
    EXPECT_EQ(e.steps, expected_steps);
    EXPECT_FALSE(e.wall_time.has_value());
}

/// @brief A visitor that writes down the parts it is given, in their order.
class PartsLog final : public DescriptionVisitor {
public:
    std::vector<std::string> parts{}; ///< the parts, in order
    void sounding(const Sounding& start, si::Length zi) override {
        const std::string height = std::to_string((int)zi.in(m));
        const char* kind = start.is_idealized() ? " idealized" : "";
        parts.push_back("sounding " + height + kind);
    }
    void column(const ColumnExtent&, const AtmosphereGrid& cells) override {
        const std::string nx = std::to_string(cells.nx());
        parts.push_back("column " + nx);
    }
    void surface(const SurfaceForcing& floor) override {
        const char* patch = floor.patch.active() ? " patch" : "";
        parts.push_back(std::string("surface") + patch);
    }
    void wind(const std::optional<WindForcing>& held) override {
        const char* part = held ? "wind" : "no wind";
        parts.push_back(part);
    }
    void sponge(const SpongeProfile& layer) override {
        const std::string part = layer.describe();
        parts.push_back(part);
    }
    void numerics(const CblNumerics&, si::Duration step) override {
        const std::string seconds = std::to_string(step.in(s));
        parts.push_back("numerics " + seconds);
    }
    void schedule(const RunSchedule& times) override {
        const std::string interval = std::to_string((int)times.interval.in(s));
        parts.push_back("schedule " + interval);
    }
    void noise(const StartNoise& start_noise) override {
        const char* part = start_noise.active() ? "noise" : "no noise";
        parts.push_back(part);
    }
    void moisture(bool moist) override {
        const char* part = moist ? "moist" : "dry";
        parts.push_back(part);
    }
};

// visit gives every part, in a fixed order
TEST(CblDescription, VisitGivesEveryPart) {
    PartsLog log{};
    quick().visit(log);
    const std::string numerics = "numerics " + std::to_string(0.04);
    const std::vector<std::string> expected = { "sounding 180 idealized", "column 128", "surface patch", "no wind", "no sponge",
                                                numerics, "schedule 10", "noise", "dry" };
    EXPECT_EQ(log.parts, expected);
}

} // namespace
