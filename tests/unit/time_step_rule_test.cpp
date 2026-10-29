#include "setup/core/time_step_rule.hpp"
#include "setup/core/diffusivity.hpp"
#include "setup/core/turbulent_prandtl.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/units/lattice_converter.hpp"

#include <gtest/gtest.h>
#include <limits>

namespace {

using namespace si::symbols;

/// The time step of 10 m cells as the core holds it: the float nearest to 0.16 s (si::LatticeConverter's step).
const double core_step_of_ten_metres = (double)0.16f;

// The convective boundary layer example's rule, dt = 0.04 s cell / 2.5 m, at every cell size (by hand, in double)
TEST(TimeStepRule, ConstantLatticeSpeedGivesTheExamplesStep) {
    const TimeStepRule rule = TimeStepRule::constant_lattice_speed(0.04 * s, 2.5 * m);
    for(const double cell : { 0.5, 1.0, 2.5, 5.0, 10.0, 12.5, 20.0 }) {
        const double step = rule.step(cell * m).in(s);
        const double expected_step = 0.04 * cell / 2.5;
        EXPECT_DOUBLE_EQ(step, expected_step) << cell << " m";
    }
    const double at_10 = rule.step(10.0 * m).in(s);
    EXPECT_DOUBLE_EQ(at_10, 0.16);
    const double at_5 = rule.step(5.0 * m).in(s);
    EXPECT_DOUBLE_EQ(at_5, 0.08);
    const double at_2_5 = rule.step(2.5 * m).in(s);
    EXPECT_DOUBLE_EQ(at_2_5, 0.04);
    const double speed_10 = rule.lattice_speed(10.0 * m).in(m / s);
    EXPECT_DOUBLE_EQ(speed_10, 62.5);
    const double speed_2_5 = rule.lattice_speed(2.5 * m).in(m / s);
    EXPECT_DOUBLE_EQ(speed_2_5, 62.5);
}

TEST(TimeStepRule, FixedIgnoresTheCellSize) {
    const TimeStepRule rule = TimeStepRule::fixed(1.0 * s);
    const double at_1 = rule.step(1.0 * m).in(s);
    EXPECT_EQ(at_1, 1.0);
    const double at_7 = rule.step(7.0 * m).in(s);
    EXPECT_EQ(at_7, 1.0);
}

// by hand: dt = cell / fastest * the lattice speed of the Mach number (si::LatticeMach's default: 0.1 cells per step)
TEST(TimeStepRule, FromMachPutsTheFastestSpeedAtTheLatticeSpeed) {
    const TimeStepRule rule = TimeStepRule::from_mach(10.0 * m / s, si::LatticeMach());
    const double lattice_speed = 0.1;
    const double step = rule.step(0.02 * m).in(s);
    const double crossing_time = 0.02 / 10.0;
    const double expected_step = crossing_time * lattice_speed;
    EXPECT_DOUBLE_EQ(step, expected_step);
}

TEST(TimeStepRule, RefusesNonPositiveValues) {
    EXPECT_THROW(TimeStepRule::constant_lattice_speed(0.0 * s, 2.5 * m), SetupError);
    EXPECT_THROW(TimeStepRule::constant_lattice_speed(0.04 * s, 0.0 * m), SetupError);
    EXPECT_THROW(TimeStepRule::fixed(-1.0 * s), SetupError);
    EXPECT_THROW(TimeStepRule::from_mach(0.0 * m / s, si::LatticeMach()), SetupError);
    EXPECT_THROW(TimeStepRule::fixed(1.0 * s).step(0.0 * m), SetupError);
}

TEST(Diffusivity, LatticeValuesPassThroughAndPhysicalOnesConvert) {
    const si::LatticeConverter converter(10.0 * m, 0.16 * s);
    const double dt = core_step_of_ten_metres;
    const Diffusivity les = Diffusivity::lattice(si::LatticeDiffusivity(1.0E-4));
    EXPECT_TRUE(les.is_lattice());
    const float les_lattice = les.lattice_value(converter);
    EXPECT_EQ(les_lattice, 1.0E-4f);
    const double les_physical = les.physical_value(converter).in(m * m / s);
    const double expected_les_physical = 1.0E-4 * 10.0 * 10.0 / dt;
    EXPECT_DOUBLE_EQ(les_physical, expected_les_physical); // 0.0625 m2/s
    const Diffusivity air = Diffusivity::physical(0.0625 * m * m / s);
    EXPECT_FALSE(air.is_lattice());
    const float air_lattice = air.lattice_value(converter);
    const float expected_air_lattice = converter.viscosity(0.0625 * m * m / s);
    EXPECT_EQ(air_lattice, expected_air_lattice);
    const double air_physical = air.physical_value(converter).in(m * m / s);
    EXPECT_DOUBLE_EQ(air_physical, 0.0625);
    EXPECT_THROW(Diffusivity::physical(-1.0 * m * m / s), SetupError);
    EXPECT_THROW(Diffusivity::lattice(si::LatticeDiffusivity(-1.0)), SetupError);
}

// The same lattice value is another physical diffusivity at another cell size (D = value dx²/dt, dx/dt constant)
TEST(Diffusivity, ALatticeValueScalesWithTheCell) {
    const TimeStepRule rule = TimeStepRule::constant_lattice_speed(0.04 * s, 2.5 * m);
    const Diffusivity les = Diffusivity::lattice(si::LatticeDiffusivity(1.0E-4));
    const si::LatticeConverter coarse(10.0 * m, rule.step(10.0 * m));
    const si::LatticeConverter fine(5.0 * m, rule.step(5.0 * m));
    const double fine_physical = les.physical_value(fine).in(m * m / s);
    const double coarse_physical = les.physical_value(coarse).in(m * m / s);
    const double expected_fine = 0.5 * coarse_physical;
    EXPECT_DOUBLE_EQ(fine_physical, expected_fine);
}

TEST(TurbulentPrandtl, KeepsTheInverseExact) {
    const double third_inverse = TurbulentPrandtl::one_third().inverse();
    EXPECT_EQ(third_inverse, 3.0);
    const float third_lattice_inverse = TurbulentPrandtl::one_third().lattice_inverse();
    EXPECT_EQ(third_lattice_inverse, 3.0f);
    const double given_inverse = TurbulentPrandtl::from_inverse(3.0).inverse();
    EXPECT_EQ(given_inverse, 3.0);
    const double half_inverse = TurbulentPrandtl::of(0.5).inverse();
    EXPECT_DOUBLE_EQ(half_inverse, 2.0);
    EXPECT_THROW(TurbulentPrandtl::of(0.0), SetupError);
    EXPECT_THROW(TurbulentPrandtl::from_inverse(-1.0), SetupError);
}

// Regression (simple-code review): a NaN passed the checks of below 0 and became a NaN diffusivity or 1/Pr_t
TEST(Diffusivity, RefusesNotANumber) {
    const double not_a_number{ std::numeric_limits<double>::quiet_NaN() };
    EXPECT_THROW(Diffusivity::physical(not_a_number * m * m / s), SetupError);
    EXPECT_THROW(Diffusivity::lattice(si::LatticeDiffusivity(not_a_number)), SetupError);
}

TEST(TurbulentPrandtl, RefusesNotANumber) {
    const double not_a_number{ std::numeric_limits<double>::quiet_NaN() };
    EXPECT_THROW(TurbulentPrandtl::of(not_a_number), SetupError);
    EXPECT_THROW(TurbulentPrandtl::from_inverse(not_a_number), SetupError);
}

// the convection quantities written with the symbols, read back in other units
TEST(SiUnits, ConvectionQuantitiesAndTheirSymbols) {
    const auto heat_flux_symbols = 0.24 * K * m / s; // the symbols' own type
    const double heat_flux = heat_flux_symbols.in(K * m / s);
    EXPECT_EQ(heat_flux, 0.24);
    const si::LapseRate per_km = si::LapseRate(5.0 * K / km);
    const double per_km_in_m = per_km.in(K / m);
    EXPECT_DOUBLE_EQ(per_km_in_m, 0.005);
    const si::LapseRate per_m = si::LapseRate(0.003 * K / m);
    const double per_m_in_m = per_m.in(K / m);
    EXPECT_EQ(per_m_in_m, 0.003);
    const si::WaterContent grams = si::WaterContent(10.0 * g_w / kg);
    const double grams_in_kg = grams.in(kg_w / kg);
    EXPECT_DOUBLE_EQ(grams_in_kg, 0.01);
    const auto kilograms_symbols = 0.009 * kg_w / kg; // the symbols' own type
    const double kilograms_in_g = kilograms_symbols.in(g_w / kg);
    EXPECT_DOUBLE_EQ(kilograms_in_g, 9.0);
    const auto hectopascals = 1013.25 * hPa; // the symbols' own type
    const double pascals = hectopascals.in(Pa);
    EXPECT_DOUBLE_EQ(pascals, 101325.0);
    const auto speed = 0.5 * m / s;
    const si::MoistureFlux flux = 0.01 * kg_w / kg * speed;
    const double flux_value = flux.in(kg_w / kg * m / s);
    const double expected_flux = 0.01 * 0.5;
    EXPECT_EQ(flux_value, expected_flux);
    const si::KinematicHeatFlux q = 2.0 * K * speed;
    const double q_value = q.in(K * m / s);
    EXPECT_EQ(q_value, 1.0);
    const auto depth = 200.0 * m;
    const si::LapseRate g = 1.0 * K / depth;
    const double g_value = g.in(K / m);
    const double expected_g = 1.0 / 200.0;
    EXPECT_EQ(g_value, expected_g);
}

} // namespace
