#include "setup/units/lattice_converter.hpp"
#include "setup/core/setup_error.hpp"
#include "golden_capture.hpp"

#include "units.hpp"

#include <gtest/gtest.h>

namespace {

using namespace si::symbols;

/// @return the converter of 10 m cells and 0.16 s steps about 300 K
si::LatticeConverter ten_metres() {
    const si::LatticeConverter c = si::LatticeConverter(10.0 * m, 0.16 * s, si::kelvin_point(300.0));
    return c;
}

// by hand: u dt / dx in cells per step, g dt^2 / dx, nu dt / dx^2, dt / tau
TEST(LatticeConverter, KinematicByHand) {
    const si::LatticeConverter c = ten_metres();
    const double dt = (double)0.16f; // the core's float step
    const float velocity = c.velocity(5.0 * m / s);
    const double velocity_double = 5.0 * dt / 10.0;
    const float expected_velocity = (float)velocity_double;
    EXPECT_EQ(velocity, expected_velocity);
    const float acceleration = c.acceleration(9.81 * m / (s * s));
    const double acceleration_double = 9.81 * dt * dt / 10.0;
    const float expected_acceleration = (float)acceleration_double;
    EXPECT_EQ(acceleration, expected_acceleration);
    const float viscosity = c.viscosity(1.5e-5 * m * m / s);
    const double viscosity_double = 1.5e-5 * dt / 100.0;
    const float expected_viscosity = (float)viscosity_double;
    EXPECT_EQ(viscosity, expected_viscosity);
    const float per_step = c.per_step(450.0 * s);
    const double per_step_double = dt / 450.0;
    const float expected_per_step = (float)per_step_double;
    EXPECT_EQ(per_step, expected_per_step);
    const double cells = c.cells(1880.0 * m);
    EXPECT_DOUBLE_EQ(cells, 188.0);
    const double si_velocity = c.si_velocity(0.1).in(m / s);
    const double expected_si_velocity = 0.1 * 10.0 / dt;
    EXPECT_DOUBLE_EQ(si_velocity, expected_si_velocity);
    const double physical = c.physical(si::LatticeDiffusivity(1.0E-4)).in(m * m / s);
    const double expected_physical = 1.0E-4 * 100.0 / dt;
    EXPECT_DOUBLE_EQ(physical, expected_physical);
}

// the step is held as the core's float; the cell must be a float exactly
TEST(LatticeConverter, CoreFloats) {
    const double step = ten_metres().step().in(s);
    const double expected_step = (double)0.16f;
    EXPECT_EQ(step, expected_step);
    EXPECT_THROW(si::LatticeConverter(3.3 * m, 0.1 * s), SetupError); // 3.3 is not a float exactly
    EXPECT_THROW(si::LatticeConverter(0.0 * m, 0.1 * s), SetupError);
    EXPECT_THROW(si::LatticeConverter(10.0 * m, 0.0 * s), SetupError);
    EXPECT_THROW(si::LatticeConverter(10.0 * m, 0.16 * s).theta_0(), SetupError); // no thermal conversions without theta_0
}

// the core's to_ulong(): the nearest step, half a step up
TEST(LatticeConverter, StepsRoundToNearest) {
    const si::LatticeConverter c(10.0 * m, 0.5 * s);
    const std::uint64_t at_10 = c.steps(10.0 * s);
    EXPECT_EQ(at_10, 20u);
    const std::uint64_t at_10_2 = c.steps(10.2 * s);
    EXPECT_EQ(at_10_2, 20u);
    const std::uint64_t at_10_3 = c.steps(10.3 * s);
    EXPECT_EQ(at_10_3, 21u);
    const std::uint64_t at_0 = c.steps(0.0 * s);
    EXPECT_EQ(at_0, 0u);
    const double time = c.time(20u).in(s);
    EXPECT_DOUBLE_EQ(time, 10.0);
}

// by hand: T = 1 + (theta - theta_0) / 1 K, the heat rate q dt / dx per 1 K
TEST(LatticeConverter, ThermalByHand) {
    const si::LatticeConverter c = ten_metres();
    const double dt = (double)0.16f;
    const float temperature = c.temperature(si::kelvin_point(301.5));
    EXPECT_EQ(temperature, 2.5f);
    const float excess = c.temperature_excess(-0.25 * K);
    EXPECT_EQ(excess, 0.75f);
    const double si_excess = c.si_excess(1.5).in(K);
    EXPECT_DOUBLE_EQ(si_excess, 0.5);
    const double si_temperature = si::in_kelvin(c.si_temperature(1.5));
    EXPECT_DOUBLE_EQ(si_temperature, 300.5);
    const float heat_rate = c.heat_rate(0.24 * K * m / s);
    const double heat_rate_double = 0.24 * dt / 10.0;
    const float expected_heat_rate = (float)heat_rate_double;
    EXPECT_EQ(heat_rate, expected_heat_rate);
    const float expansion = c.expansion(1.0 / (300.0 * K));
    const double expansion_double = 1.0 / 300.0;
    const float expected_expansion = (float)expansion_double;
    EXPECT_EQ(expansion, expected_expansion);
}

// by hand: Q = 1 + qt / 0.01 kg/kg, the moisture rate m dt / dx / 0.01
TEST(LatticeConverter, MoistureByHand) {
    const si::LatticeConverter c = ten_metres();
    const double dt = (double)0.16f;
    const float moisture = c.moisture(8.0 * g_w / kg);
    const double moisture_double = 1.0 + 0.008 / 0.01;
    const float expected_moisture = (float)moisture_double;
    EXPECT_EQ(moisture, expected_moisture);
    const double water = c.water(1.8).in(kg_w / kg);
    const double expected_water = 0.8 * 0.01;
    EXPECT_DOUBLE_EQ(water, expected_water);
    const float moisture_rate = c.moisture_rate(1.0e-4 * kg_w / kg * m / s);
    const double moisture_rate_double = 1.0e-4 * dt / 10.0 / 0.01;
    const float expected_moisture_rate = (float)moisture_rate_double;
    EXPECT_EQ(moisture_rate, expected_moisture_rate);
}

/// @return the converter of 0.25 m cells and 0.001 s steps with the density of water
si::LatticeConverter water_quarter_metre() {
    const si::Density water = 1000.0 * kg / (m * m * m);
    const si::LatticeConverter c = si::LatticeConverter(0.25 * m, 0.001 * s).with_density(water);
    return c;
}

// by hand: p dt^2 / (rho dx^2), rho_lattice = 1 + 3 p_lattice, sigma dt^2 / (rho dx^3), F back = f rho dx^4 / dt^2
TEST(LatticeConverter, MechanicalByHand) {
    const si::LatticeConverter c = water_quarter_metre();
    const double dt = (double)0.001f; // the core's float step
    const double dx = 0.25;
    const double rho = 1000.0;
    const double p_lattice_double = 100.0 * dt * dt / (rho * dx * dx);
    const float pressure = c.pressure(100.0 * Pa);
    const float expected_pressure = (float)p_lattice_double;
    EXPECT_EQ(pressure, expected_pressure);
    const float density = c.lattice_density(100.0 * Pa);
    const double density_double = 1.0 + 3.0 * p_lattice_double;
    const float expected_density = (float)density_double;
    EXPECT_EQ(density, expected_density);
    const double si_pressure = c.si_pressure(p_lattice_double).in(Pa);
    EXPECT_DOUBLE_EQ(si_pressure, 100.0);
    const double si_density = c.si_density(1.5).in(kg / (m * m * m));
    EXPECT_DOUBLE_EQ(si_density, 1500.0);
    const double mass_unit = c.mass_unit().in(kg);
    EXPECT_DOUBLE_EQ(mass_unit, rho * dx * dx * dx);
    const float tension = c.surface_tension(0.072 * N / m);
    const double tension_double = 0.072 * dt * dt / (rho * dx * dx * dx);
    const float expected_tension = (float)tension_double;
    EXPECT_EQ(tension, expected_tension);
    const double si_tension = c.si_surface_tension(tension_double).in(N / m);
    EXPECT_DOUBLE_EQ(si_tension, 0.072);
    const double force = c.si_force(2.0).in(N);
    const double expected_force = 2.0 * rho * dx * dx * dx * dx / (dt * dt);
    EXPECT_DOUBLE_EQ(force, expected_force);
}

// a pressure, a force or a surface tension needs the reference density: fail fast without it
TEST(LatticeConverter, MechanicalNeedsTheDensity) {
    const si::LatticeConverter c(0.25 * m, 0.001 * s);
    EXPECT_THROW(c.density(), SetupError);
    EXPECT_THROW(c.pressure(1.0 * Pa), SetupError);
    EXPECT_THROW(c.lattice_density(1.0 * Pa), SetupError);
    EXPECT_THROW(c.si_force(1.0), SetupError);
    EXPECT_THROW(c.surface_tension(1.0 * N / m), SetupError);
    EXPECT_THROW(c.core_units(), SetupError);
    const si::Density no_density = 0.0 * kg / (m * m * m);
    EXPECT_THROW(c.with_density(no_density), SetupError);
}

// the core's own Units, set from core_units(), converts as the converter does (within the float rounding of its
// float operations)
TEST(LatticeConverter, CoreUnitsAreTheConverters) {
    const si::LatticeConverter c = water_quarter_metre();
    const si::CoreUnits numbers = c.core_units();
    EXPECT_EQ(numbers.m, 0.25f);
    EXPECT_EQ(numbers.s, 0.001f);
    const double kg_double = 1000.0 * 0.25 * 0.25 * 0.25;
    const float expected_kg = (float)kg_double;
    EXPECT_EQ(numbers.kg, expected_kg);
    Units core{};
    core.set_m_kg_s(numbers.m, numbers.kg, numbers.s);
    const float core_velocity = core.u(2.0f);
    const float velocity = c.velocity(2.0 * m / s);
    EXPECT_FLOAT_EQ(velocity, core_velocity);
    const float core_viscosity = core.nu(1.0e-6f);
    const float viscosity = c.viscosity(1.0e-6 * m * m / s);
    EXPECT_FLOAT_EQ(viscosity, core_viscosity);
    const float core_tension = core.sigma(0.072f);
    const float tension = c.surface_tension(0.072 * N / m);
    EXPECT_FLOAT_EQ(tension, core_tension);
}

// by hand: f dt cycles per step, omega dt radians per step, cells dx back to metres
TEST(LatticeConverter, RatesAndLengths) {
    const si::LatticeConverter c = water_quarter_metre();
    const double dt = (double)0.001f;
    const float frequency = c.frequency(2.0 * Hz);
    const double frequency_double = 2.0 * dt;
    const float expected_frequency = (float)frequency_double;
    EXPECT_EQ(frequency, expected_frequency);
    const float rate = c.angular_rate(180.0 * deg / s);
    const double rate_double = 3.141592653589793 * dt;
    EXPECT_FLOAT_EQ(rate, (float)rate_double);
    const double length = c.si_length(8.0).in(m);
    EXPECT_DOUBLE_EQ(length, 2.0);
}

// a temperature scale with another unit: a cold and a hot wall at reference -/+ unit/2 are 0.5 and 1.5 exactly,
// the expansion is beta times the unit
TEST(LatticeConverter, TemperatureScaleOfAnyUnit) {
    const si::Temperature cold = si::kelvin_point(290.0);
    const si::Temperature hot = si::kelvin_point(310.0);
    const si::TemperatureDifference unit = hot - cold;
    const si::Temperature reference = cold + unit * 0.5;
    const si::LatticeConverter c = si::LatticeConverter(0.5 * m, 0.001 * s).with_temperature_scale(reference, unit);
    const float cold_lattice = c.temperature(cold);
    EXPECT_EQ(cold_lattice, 0.5f);
    const float hot_lattice = c.temperature(hot);
    EXPECT_EQ(hot_lattice, 1.5f);
    const float middle = c.temperature(reference);
    EXPECT_EQ(middle, 1.0f);
    const float expansion = c.expansion(3.4e-3 / K);
    const double expansion_double = 3.4e-3 * 20.0;
    const float expected_expansion = (float)expansion_double;
    EXPECT_EQ(expansion, expected_expansion);
    const double back = si::in_kelvin(c.si_temperature(1.5));
    EXPECT_DOUBLE_EQ(back, 310.0);
    const double default_unit = si::LatticeConverter(0.5 * m, 0.001 * s).temperature_unit().in(K);
    EXPECT_EQ(default_unit, 1.0);
    const si::TemperatureDifference no_unit = 0.0 * K;
    EXPECT_THROW(si::LatticeConverter(0.5 * m, 0.001 * s).with_temperature_scale(cold, no_unit), SetupError);
}

// Regression for review A6: the lattice expansion was beta 300 K for any temperature range, 10x
// too strong for 30 K. The lattice buoyancy g beta_lattice (T_lattice - 1) equals the physical g beta (T - T_ref).
TEST(LatticeConverter, LatticeBuoyancyEqualsThePhysicalBuoyancy) {
    const si::Temperature cold = si::kelvin_point(300.0);
    const si::Temperature hot = si::kelvin_point(330.0);
    const si::TemperatureDifference unit = hot - cold;
    const si::Temperature reference = cold + unit * 0.5;
    const si::LatticeConverter c = si::LatticeConverter(0.5 * m, 0.001 * s).with_temperature_scale(reference, unit);
    const si::ThermalExpansion beta = 0.00341 / K;
    const double g_lattice = 0.0008; // any lattice gravity: both sides scale with it
    const si::Temperature t = si::kelvin_point(327.0);
    const double lattice_buoyancy = g_lattice * (double)c.expansion(beta) * ((double)c.temperature(t) - 1.0);
    const double physical_buoyancy = g_lattice * au::as_raw_number(beta * (t - reference));
    EXPECT_NEAR(lattice_buoyancy, physical_buoyancy, 1e-6 * g_lattice);
    const float expansion = c.expansion(beta);
    const double expansion_double = 0.00341 * 30.0;
    EXPECT_EQ(expansion, (float)expansion_double);
}

// the Mach path: the cell is the reference length over its cells (as a float), the step puts the reference speed at
// the Mach number's lattice speed
TEST(LatticeConverter, FromReferenceByHand) {
    const si::LatticeMach mach(0.13);
    const si::LatticeConverter c = si::LatticeConverter::from_reference(70.0 * m, 280.0, 250.0 * m / s, mach);
    const double cell = c.cell().in(m);
    EXPECT_EQ(cell, 0.25); // 70 / 280, a float exactly
    const double step = c.step().in(s);
    const double step_double = mach.lattice_speed() * 0.25 / 250.0;
    const double expected_step = (double)(float)step_double;
    EXPECT_EQ(step, expected_step);
    const float reference_speed = c.velocity(250.0 * m / s);
    EXPECT_FLOAT_EQ(reference_speed, (float)mach.lattice_speed());
    const si::LatticeConverter derived = si::LatticeConverter::from_reference(1.0 * m, 3.0, 10.0 * m / s, mach);
    const double derived_cell = derived.cell().in(m);
    const double expected_cell = (double)(1.0f / 3.0f);
    EXPECT_EQ(derived_cell, expected_cell); // a derived cell is the nearest float
    EXPECT_THROW(si::LatticeConverter::from_reference(1.0 * m, 0.0, 10.0 * m / s, mach), SetupError);
}

/// The verified examples whose captures the golden tests compare against (tests/golden/cbl).
constexpr const char* golden_cases[] = { "quick", "sp2011" };

/// @param g a capture
/// @return the conversions of its frozen dx, dt and theta_0
si::LatticeConverter converter_of(const GoldenCapture& g) {
    const si::Length dx = g.setting("dx") * m;
    const si::Duration dt = g.setting("dt") * s;
    const si::Temperature theta_0 = si::kelvin_point(g.setting("theta_0"));
    const si::LatticeConverter c = si::LatticeConverter(dx, dt, theta_0);
    return c;
}

// the heat and moisture rates the core gets from the captured fluxes: the verified examples' floats, bit for bit
TEST(LatticeConverter, TheSourceRatesAreTheVerifiedExamples) {
    for(const char* name : golden_cases) {
        const GoldenCapture g(name);
        const si::LatticeConverter c = converter_of(g);
        const si::KinematicHeatFlux flux = g.setting("flux") * K * m / s;
        const float heat_rate = c.heat_rate(flux);
        const float expected_heat_rate = (float)g.setting("heat_rate");
        EXPECT_EQ(heat_rate, expected_heat_rate) << name;
        const si::MoistureFlux moisture_flux = g.setting("moisture_flux") * kg_w / kg * m / s;
        const float moisture_rate = c.moisture_rate(moisture_flux);
        const float expected_moisture_rate = (float)g.setting("moisture_rate");
        EXPECT_EQ(moisture_rate, expected_moisture_rate) << name;
    }
}

// T_ref = 1 + the start profile's excess at every level: the verified examples' floats, bit for bit
TEST(LatticeConverter, TheLatticeExcessIsTheVerifiedExamples) {
    for(const char* name : golden_cases) {
        const GoldenCapture g(name);
        const si::LatticeConverter c = converter_of(g);
        for(std::size_t z = 0u; z < g.levels(); z++) {
            const si::TemperatureDifference start_excess = g.level(z, "start_theta_K") * K;
            const float excess = c.temperature_excess(start_excess);
            const float expected_excess = (float)g.level(z, "T_ref");
            EXPECT_EQ(excess, expected_excess) << name << " z " << z;
        }
    }
}

// the time steps of the spin-up, the recorded time and the interval: the verified examples' counts
TEST(LatticeConverter, TheStepCountsAreTheVerifiedExamples) {
    for(const char* name : golden_cases) {
        const GoldenCapture g(name);
        const si::LatticeConverter c = converter_of(g);
        const std::uint64_t spinup = c.steps(g.setting("spinup") * s);
        const std::uint64_t expected_spinup = (std::uint64_t)g.setting("steps_spinup");
        EXPECT_EQ(spinup, expected_spinup) << name;
        const std::uint64_t record = c.steps(g.setting("record") * s);
        const std::uint64_t expected_record = (std::uint64_t)g.setting("steps_record");
        EXPECT_EQ(record, expected_record) << name;
        const std::uint64_t interval = c.steps(g.setting("interval") * s);
        const std::uint64_t expected_interval = (std::uint64_t)g.setting("steps_interval");
        EXPECT_EQ(interval, expected_interval) << name;
    }
}

} // namespace
