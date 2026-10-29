#include "setup/core/boussinesq_buoyancy.hpp"
#include "setup/core/setup_error.hpp"

#include <gtest/gtest.h>

namespace {

using namespace si::symbols;

/// The time step of 10 m cells as the core holds it: the float nearest to 0.16 s (si::LatticeConverter's step).
const double core_step_of_ten_metres = (double)0.16f;

/// @return the gravitational acceleration of the tests, 9.81 m/s2
si::Acceleration gravity() {
    const si::Acceleration g = 9.81 * m / (s * s);
    return g;
}

// |fz| beta = g dt^2/(dx theta_0): the lattice buoyancy of 1 K is g/theta_0 in lattice units
TEST(BoussinesqBuoyancy, ExpansionTimesCarrierIsTheLatticeBuoyancyOfOneKelvin) {
    const si::LatticeConverter converter(10.0 * m, 0.16 * s);
    const BoussinesqBuoyancy b(gravity(), si::kelvin_point(300.0));
    const float force = b.lattice_force();
    EXPECT_EQ(force, -1.0E-6f);
    const double dt = core_step_of_ten_metres;
    const double g_dt2 = 9.81 * dt * dt;
    const double dx_theta_0 = 10.0 * 300.0;
    const double expected = g_dt2 / dx_theta_0; // in double: the derivation, not the code's order
    const double expansion = (double)b.lattice_expansion(converter);
    const double actual = b.carrier() * expansion;
    const double tolerance = expected * 1.0E-6;
    EXPECT_NEAR(actual, expected, tolerance);
}

// the example's beta() = g/dx*(dt*dt)/(theta_0*-fz), by hand in double, to the float the core gets
TEST(BoussinesqBuoyancy, MatchesTheExamplesExpression) {
    const double dx = 10.0;
    const double dt = core_step_of_ten_metres;
    const double theta_0 = 298.9387;
    const double fz = -1.0E-6;
    const si::LatticeConverter converter(dx * m, 0.16 * s);
    const BoussinesqBuoyancy b(gravity(), si::kelvin_point(theta_0));
    const float actual = b.lattice_expansion(converter);
    const double g_over_dx = 9.81 / dx;
    const double dt2 = dt * dt;
    const double numerator = g_over_dx * dt2;
    const double denominator = theta_0 * -fz;
    const double beta = numerator / denominator;
    const float expected = (float)beta;
    EXPECT_FLOAT_EQ(actual, expected);
}

// |fz| beta does not depend on the carrier: halving it doubles beta
TEST(BoussinesqBuoyancy, TheCarrierCancels) {
    const si::LatticeConverter converter(5.0 * m, 0.08 * s);
    const BoussinesqBuoyancy a(gravity(), si::kelvin_point(300.0), 1.0E-6);
    const BoussinesqBuoyancy b(gravity(), si::kelvin_point(300.0), 0.5E-6);
    const double a_expansion = (double)a.lattice_expansion(converter);
    const double a_buoyancy = a.carrier() * a_expansion;
    const double b_expansion = (double)b.lattice_expansion(converter);
    const double b_buoyancy = b.carrier() * b_expansion;
    EXPECT_DOUBLE_EQ(a_buoyancy, b_buoyancy);
}

// the hydrostatic ramp rho = 1 + 3 fz (z - middle): 3 carrier levels; refused above 1 %
TEST(BoussinesqBuoyancy, TheColumnCheck) {
    const BoussinesqBuoyancy b(gravity(), si::kelvin_point(300.0));
    const double ramp = b.density_ramp(378u);
    const double expected_ramp = 3.0E-6 * 378.0;
    EXPECT_DOUBLE_EQ(ramp, expected_ramp);
    EXPECT_NO_THROW(b.validate(378u));  // the moist sounding case: 0.11 %
    EXPECT_NO_THROW(b.validate(3333u)); // 1.0 %
    EXPECT_THROW(b.validate(3334u), SetupError);
}

// Regression for defect 3: the expansion is per 1 K; a converter with another temperature unit is refused (it gave a
// buoyancy off by the ratio of its unit to 1 K)
TEST(BoussinesqBuoyancy, RefusesAConverterWithAnotherTemperatureUnit) {
    const BoussinesqBuoyancy b(gravity(), si::kelvin_point(300.0));
    const si::LatticeConverter one_kelvin = si::LatticeConverter(10.0 * m, 0.16 * s).with_theta_0(si::kelvin_point(300.0));
    EXPECT_NO_THROW(b.lattice_expansion(one_kelvin));
    const si::LatticeConverter two_kelvin = si::LatticeConverter(10.0 * m, 0.16 * s).with_temperature_scale(si::kelvin_point(300.0), 2.0 * K);
    EXPECT_THROW(b.lattice_expansion(two_kelvin), SetupError);
}

TEST(BoussinesqBuoyancy, RefusesNonPositiveValues) {
    EXPECT_THROW(BoussinesqBuoyancy(0.0 * m / (s * s), si::kelvin_point(300.0)), SetupError);
    EXPECT_THROW(BoussinesqBuoyancy(gravity(), si::kelvin_point(0.0)), SetupError);
    EXPECT_THROW(BoussinesqBuoyancy(gravity(), si::kelvin_point(300.0), 0.0), SetupError);
}

} // namespace
