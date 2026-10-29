#include "setup/core/setup_error.hpp"
#include "setup/units/moist_air_physics.hpp"
#include "setup/units/physical_constants.hpp"
#include "setup/core/fluids.hpp"
#include "setup/units/lattice_mach.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"

#include "moist_air.hpp"

#include <gtest/gtest.h>

#include <functional>
#include <limits>
#include <type_traits>

namespace {

using namespace si::symbols;

/// Whether a + b compiles.
template<typename A, typename B> concept Addable = requires(A a, B b) { a + b; };

// the mistakes the types refuse at compile time (metres plus seconds is a hard error inside Au, so it is not tested here)
static_assert(!Addable<si::Temperature, si::Temperature>, "a reading plus a reading");
static_assert(Addable<si::Temperature, si::TemperatureDifference>, "a reading plus a difference");
static_assert(std::is_same_v<decltype(si::Temperature{} - si::Temperature{}), si::TemperatureDifference>, "a reading minus a reading is a difference");
static_assert(!std::is_convertible_v<si::Length, si::Speed>, "a length is not a speed");
static_assert(!std::is_constructible_v<si::Length, double>, "a raw number is not a length");
static_assert(!std::is_convertible_v<si::MoistureFlux, si::Speed>, "a moisture flux is not a speed");
static_assert(!std::is_convertible_v<si::WaterContent, double>, "a water content is not a raw number");
// the call sites are the aliases
static_assert(std::is_convertible_v<decltype(0.262 * K * m / s), si::KinematicHeatFlux>);
static_assert(std::is_convertible_v<decltype(8.0 * g_w / kg), si::WaterContent>);
static_assert(std::is_convertible_v<decltype(3.0 * K / km), si::LapseRate>);

// the constants are the core's (src/moist_air.hpp), with their units
TEST(PhysicalConstants, AreTheCores) {
    const double gravity = si::PhysicalConstants::gravity.in(m / (s * s));
    EXPECT_EQ(gravity, moist_air::G);
    const double heat_capacity = si::PhysicalConstants::dry_air_heat_capacity.in(J / (kg * K));
    EXPECT_EQ(heat_capacity, moist_air::Cp);
    const double latent_heat = si::PhysicalConstants::latent_heat.in(J / kg_w);
    EXPECT_EQ(latent_heat, moist_air::Lv);
    const double ground_pressure = si::PhysicalConstants::ground_pressure.in(Pa);
    EXPECT_EQ(ground_pressure, moist_air::GroundPressure);
}

// the Bowen moisture flux comes out as kg of water per kg of air times m/s
TEST(PhysicalConstants, BowenFluxUnits) {
    const auto heat_flux_cp = 0.262 * K * m / s * si::PhysicalConstants::dry_air_heat_capacity; // K m/s times J/(kg K)
    const auto latent_bowen = si::PhysicalConstants::latent_heat * 0.5;                          // J per kg of water
    const si::MoistureFlux e = heat_flux_cp / latent_bowen;
    const double actual = e.in(kg_w / kg * m / s);
    const double expected = moist_air::moisture_flux(0.262, 0.5);
    EXPECT_DOUBLE_EQ(actual, expected);
}

// the typed model gives the raw model's numbers
TEST(MoistAirPhysics, IsTheRawModel) {
    const si::Temperature thl = si::kelvin_point(300.0);
    const si::WaterContent qt = 18.0 * g_w / kg;
    const si::Pressure p = 85000.0 * Pa;
    const moist_air::Adjusted raw = moist_air::saturation_adjust(300.0, 0.018, 85000.0);
    const si::AdjustedAir typed = si::MoistAirPhysics::saturation_adjust(thl, qt, p);
    const double typed_kelvin = si::in_kelvin(typed.temperature);
    EXPECT_EQ(typed_kelvin, raw.kelvin);
    const double typed_liquid = typed.liquid.in(kg_w / kg);
    EXPECT_EQ(typed_liquid, raw.liquid);
    const si::WaterContent no_liquid = si::WaterContent{ au::ZERO };
    EXPECT_GT(typed.liquid, no_liquid); // saturated at 850 hPa
    const si::Temperature typed_theta_v = si::MoistAirPhysics::virtual_theta(thl, qt, p);
    const double typed_theta_v_kelvin = si::in_kelvin(typed_theta_v);
    const double raw_theta_v = moist_air::virtual_theta(300.0, 0.018, 85000.0);
    EXPECT_EQ(typed_theta_v_kelvin, raw_theta_v);
    const si::Pressure typed_pressure = si::MoistAirPhysics::isentropic_pressure(635.0 * m, thl, 101325.0 * Pa);
    const double typed_pressure_pa = typed_pressure.in(Pa);
    const double raw_pressure = moist_air::hydrostatic_pressure(635.0, 300.0);
    EXPECT_EQ(typed_pressure_pa, raw_pressure);
}

// the closed form ends at cp theta / g: at that height the pressure is 0, above it not a number
TEST(MoistAirPhysics, IsentropicPressureRefusesAHeightAtOrAboveTheTop) {
    const si::Temperature thl = si::kelvin_point(300.0);
    const double top_m = moist_air::Cp * 300.0 / moist_air::G;
    const si::Length top = top_m * m;
    EXPECT_THROW(si::MoistAirPhysics::isentropic_pressure(top, thl, 101325.0 * Pa), SetupError);
    EXPECT_THROW(si::MoistAirPhysics::isentropic_pressure(40000.0 * m, thl, 101325.0 * Pa), SetupError);
    const si::Length below_top = (top_m - 1.0) * m;
    const si::Pressure near_top = si::MoistAirPhysics::isentropic_pressure(below_top, thl, 101325.0 * Pa);
    EXPECT_GT(near_top, 0.0 * Pa);
}

// the derived quantities are the aliases (the quantity.hpp cases on Au)
static_assert(std::is_convertible_v<decltype(1.0 * m / (2.0 * s)), si::Speed>);
static_assert(std::is_convertible_v<decltype(1.0 * m / s / (1.0 * s)), si::Acceleration>);
static_assert(std::is_convertible_v<decltype(1.0 * kg / (m * m * m) * (1.0 * m / (s * s)) * (1.0 * m)), si::Pressure>);
static_assert(std::is_convertible_v<decltype(1.0 * N / m), si::SurfaceTension>);
static_assert(std::is_convertible_v<decltype(1.0 * m * m * m / s / (1.0 * m * m)), si::Speed>, "a flow rate through an area");
static_assert(std::is_convertible_v<decltype(1.0 / (1.0 * s)), si::Frequency>);
static_assert(std::is_convertible_v<decltype(1.0 * Hz), si::Frequency>);
static_assert(std::is_convertible_v<decltype(180.0 * deg / s), si::AngularSpeed>);
static_assert(std::is_convertible_v<decltype(1.0 * kg * m / (s * s)), si::Force>);
static_assert(!std::is_convertible_v<si::Angle, double>, "an angle is not a raw number");
static_assert(!std::is_constructible_v<si::LatticeMach, si::Speed>, "a speed is not a lattice Mach number");
static_assert(!std::is_convertible_v<double, si::LatticeMach>, "a raw number must name the Mach number explicitly");

// every symbol converts to its SI value (the conversions of quantity.hpp's literals)
TEST(Units, SymbolsConvertToSi) {
    const double cm_in_m = (150.0 * cm).in(m);
    EXPECT_DOUBLE_EQ(cm_in_m, 1.5);
    const double mm_in_m = (25.0 * mm).in(m);
    EXPECT_DOUBLE_EQ(mm_in_m, 0.025);
    const double km_in_m = (1.5 * km).in(m);
    EXPECT_DOUBLE_EQ(km_in_m, 1500.0);
    const double ms_in_s = (2.2 * ms).in(s);
    EXPECT_DOUBLE_EQ(ms_in_s, 0.0022);
    const double us_in_s = (90.0 * us).in(s);
    EXPECT_DOUBLE_EQ(us_in_s, 9.0e-5);
    const double minutes_in_s = au::minutes(2.0).in(s);
    EXPECT_DOUBLE_EQ(minutes_in_s, 120.0);
    const si::Speed kmh_speed = 36.0 * kmh;
    const double kmh_in_mps = kmh_speed.in(m / s);
    EXPECT_DOUBLE_EQ(kmh_in_mps, 10.0);
    const si::SurfaceTension tension = 0.072 * N / m;
    const double tension_si = tension.in(N / m);
    EXPECT_DOUBLE_EQ(tension_si, 0.072);
    const double rad_in_deg = (1.0 * rad).in(deg);
    EXPECT_NEAR(rad_in_deg, 57.29577951308232, 1e-12);
    const double right_angle_rad = (90.0 * deg).in(rad);
    EXPECT_DOUBLE_EQ(right_angle_rad, 1.5707963267948966);
}

// Celsius readings are absolute temperatures: 0 °C is exactly 273.15 K
TEST(Units, CelsiusIsAnAbsoluteTemperature) {
    const si::Temperature room = si::celsius_point(20.0);
    const double room_kelvin = si::in_kelvin(room);
    EXPECT_DOUBLE_EQ(room_kelvin, 293.15);
    const double room_celsius = si::in_celsius(room);
    EXPECT_DOUBLE_EQ(room_celsius, 20.0);
    const si::TemperatureDifference warmer = si::celsius_point(25.0) - room;
    const double warmer_kelvin = warmer.in(K);
    EXPECT_DOUBLE_EQ(warmer_kelvin, 5.0);
}

// the memory symbols are binary, as the core counts its MB and GB (quantity.hpp: 1 GB = 1024 MB)
TEST(Units, MemoryIsBinary) {
    const double mib_bytes = (1.0 * MiB).in(au::bytes);
    EXPECT_EQ(mib_bytes, 1048576.0);
    const double gib_in_mib = (20.0 * GiB).in(MiB);
    EXPECT_EQ(gib_in_mib, 20480.0);
}

// a vector adds, scales and changes its quantity with a quantity factor
TEST(Vectors, AlgebraKeepsTheQuantities) {
    const si::Velocity u{ 10.0 * m / s, 0.0 * m / s, -1.0 * m / s };
    const si::Position moved = u * (2.0 * s);
    EXPECT_DOUBLE_EQ(moved.x.in(m), 20.0);
    EXPECT_DOUBLE_EQ(moved.z.in(m), -2.0);
    const si::Velocity doubled = 2.0 * u;
    EXPECT_DOUBLE_EQ(doubled.x.in(m / s), 20.0);
    const si::Velocity halved = u / 2.0;
    EXPECT_DOUBLE_EQ(halved.z.in(m / s), -0.5);
    const si::AccelerationVector slowing = u / (5.0 * s);
    EXPECT_DOUBLE_EQ(slowing.x.in(m / (s * s)), 2.0);
    const si::Velocity sum = u + (-u);
    EXPECT_EQ(sum, si::Velocity{});
    const si::Position mixed{ 1.0 * m, -0.1 * mm, 2.0 * cm };
    EXPECT_DOUBLE_EQ(mixed.y.in(m), -0.0001);
    const si::Speed speed = si::Velocity{ 3.0 * m / s, 4.0 * m / s, 0.0 * m / s }.magnitude();
    EXPECT_DOUBLE_EQ(speed.in(m / s), 5.0);
    const si::Direction axis{ 0.0, 0.0, 2.0 };
    EXPECT_DOUBLE_EQ(axis.magnitude(), 2.0);
}
static_assert(!std::is_invocable_v<std::plus<>, si::Position, si::Velocity>, "a position plus a velocity");
static_assert(std::is_same_v<decltype(si::Velocity{} * si::Duration{}), si::Position>, "a velocity times a duration is a position");
static_assert(std::is_same_v<decltype(si::Velocity{} / si::Duration{}), si::AccelerationVector>, "a velocity over a duration is an acceleration");
static_assert(std::is_same_v<decltype(si::Position{} / si::Duration{}), si::Velocity>, "a position over a duration is a velocity");

// the lattice Mach number: the reference speed in cells per time step over 1/√3
TEST(LatticeMach, IsTheLatticeSpeedOverTheSoundSpeed) {
    const si::LatticeMach default_mach{};
    EXPECT_EQ(default_mach.lattice_speed(), 0.1); // 0.1 cells per time step, exactly
    EXPECT_NEAR(default_mach.value(), 0.17320508075688773, 1e-15);
    const si::LatticeMach sonic(1.0);
    EXPECT_EQ(sonic.lattice_speed(), si::LatticeMach::sound_speed);
    const si::LatticeMach boeing(0.13);
    EXPECT_NEAR(boeing.lattice_speed(), 0.075055534994651, 1e-12);
    EXPECT_THROW(si::LatticeMach(0.0), SetupError);
    EXPECT_THROW(si::LatticeMach(1.5), SetupError);
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(si::LatticeMach{ not_a_number }, SetupError);
}

// the fluids' properties are quantities: a Reynolds number is a plain number, a buoyancy speed a speed
TEST(Fluid, PropertiesAreQuantities) {
    const double reynolds = au::as_raw_number(2.0 * m * (10.0 * m / s) / Fluid::AIR.kinematic_viscosity);
    EXPECT_DOUBLE_EQ(reynolds, 2.0 * 10.0 / 1.48e-5);
    const double water_density = Fluid::WATER.density.in(kg / (m * m * m));
    EXPECT_EQ(water_density, 998.2);
    const si::Speed buoyancy_speed = au::sqrt(9.81 * m / (s * s) * Fluid::AIR.thermal_expansion * (30.0 * K) * (1.0 * m));
    EXPECT_GT(buoyancy_speed, si::Speed{ au::ZERO });
}

} // namespace
