#include "setup/atmosphere/wind_target.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/level_grid.hpp"
#include "setup/units/lattice_converter.hpp"
#include "golden_capture.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

namespace {

using namespace si::symbols;

/// The largest error of a wind component that is 0 by hand (double sin and cos of an angle in radians).
constexpr double zero_component_mps = 1.0E-12;

// by hand: u_ref at h_ref, 16^0.25 = 2 times it at 16 h_ref, 0 at and below the floor's top
TEST(PowerLawWind, SpeedByHand) {
    const PowerLawWind wind(5.0 * m / s, 100.0 * m, 0.25, 0.0 * deg);
    const double at_reference = wind.speed(100.0 * m).in(m / s);
    EXPECT_EQ(at_reference, 5.0);
    const double at_16_references = wind.speed(1600.0 * m).in(m / s);
    EXPECT_EQ(at_16_references, 10.0);
    const double at_floor = wind.speed(0.0 * m).in(m / s);
    EXPECT_EQ(at_floor, 0.0);
    const double below_floor = wind.speed(-5.0 * m).in(m / s);
    EXPECT_EQ(below_floor, 0.0);
    const double reference_speed = wind.reference_speed().in(m / s);
    EXPECT_EQ(reference_speed, 5.0);
    const double shape = PowerLawWind::shape(16.0, 0.25);
    EXPECT_EQ(shape, 2.0);
    const float shape_float = PowerLawWind::shape(16.0f, 0.25f);
    EXPECT_EQ(shape_float, 2.0f); // the float form, BoundaryBuilder's lattice
    const PowerLawWind uniform(3.0 * m / s, 10.0 * m, 0.0, 0.0 * deg);
    const double uniform_speed = uniform.speed(500.0 * m).in(m / s);
    EXPECT_EQ(uniform_speed, 3.0); // exponent 0: uniform
}

/// @param from_deg the direction the wind comes from (degrees clockwise from north)
/// @return a 4 m/s wind at 100 m, exponent 0.25, at its reference height
HorizontalWind blowing(double from_deg) {
    const si::Angle from = from_deg * deg;
    const PowerLawWind wind(4.0 * m / s, 100.0 * m, 0.25, from);
    const HorizontalWind at_reference = wind.at(100.0 * m);
    return at_reference;
}

// the direction is where the wind comes from, clockwise from north
TEST(PowerLawWind, DirectionConvention) {
    const HorizontalWind north = blowing(0.0);
    const HorizontalWind east = blowing(90.0);
    const HorizontalWind south = blowing(180.0);
    const HorizontalWind west = blowing(270.0);
    const double north_x = north.x.in(m / s);
    EXPECT_NEAR(north_x, 0.0, zero_component_mps);
    const double north_y = north.y.in(m / s);
    EXPECT_DOUBLE_EQ(north_y, -4.0); // from the north: toward -y
    const double east_x = east.x.in(m / s);
    EXPECT_DOUBLE_EQ(east_x, -4.0);  // from the east: toward -x
    const double east_y = east.y.in(m / s);
    EXPECT_NEAR(east_y, 0.0, zero_component_mps);
    const double south_y = south.y.in(m / s);
    EXPECT_DOUBLE_EQ(south_y, 4.0);
    const double west_x = west.x.in(m / s);
    EXPECT_DOUBLE_EQ(west_x, 4.0);
    const HorizontalWind diagonal = blowing(45.0); // from the north-east: toward -x and -y, each 4 / sqrt(2)
    const double root_two = std::sqrt(2.0);
    const double expected_component = -4.0 / root_two;
    const double diagonal_x = diagonal.x.in(m / s);
    EXPECT_DOUBLE_EQ(diagonal_x, expected_component);
    const double diagonal_y = diagonal.y.in(m / s);
    EXPECT_DOUBLE_EQ(diagonal_y, expected_component);
}

// a WindTarget through its interface
TEST(PowerLawWind, IsAWindTarget) {
    const std::unique_ptr<WindTarget> target = std::make_unique<PowerLawWind>(5.0 * m / s, 100.0 * m, 0.25, 90.0 * deg);
    const double x = target->at(100.0 * m).x.in(m / s);
    EXPECT_DOUBLE_EQ(x, -5.0);
    const double reference_speed = target->reference_speed().in(m / s);
    EXPECT_EQ(reference_speed, 5.0);
    const std::string described = target->describe();
    const std::size_t at = described.find("from 90 deg");
    EXPECT_NE(at, std::string::npos) << described;
}

TEST(PowerLawWind, RefusesBadValues) {
    EXPECT_THROW(PowerLawWind(0.0 * m / s, 100.0 * m, 0.25, 0.0 * deg), SetupError);
    EXPECT_THROW(PowerLawWind(5.0 * m / s, 0.0 * m, 0.25, 0.0 * deg), SetupError);
    EXPECT_THROW(PowerLawWind(5.0 * m / s, 100.0 * m, -0.1, 0.0 * deg), SetupError);
    const double not_a_number = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(PowerLawWind(5.0 * m / s, 100.0 * m, not_a_number, 0.0 * deg), SetupError);
}

// an extra windy case (FLUIDX3D_EXTRA_CBL_WIND): the target and the start wind in lattice units at every level,
// bit for bit against its capture; the floor and the lid get no start wind (levels.csv after #23: a capture frozen
// before it holds the target in the lid row)
TEST(PowerLawWind, PrivateWindCaseIsItsCapture) {
    const std::optional<std::string> folder = GoldenCapture::environment("FLUIDX3D_EXTRA_CBL_WIND");
    if(!folder) {
        GTEST_SKIP() << "FLUIDX3D_EXTRA_CBL_WIND is not set";
    }
    const GoldenCapture g = GoldenCapture::at(*folder);
    const si::Speed speed = g.setting("wind_speed") * m / s;
    const si::Length height = g.setting("wind_height") * m;
    const double exponent = g.setting("wind_exponent");
    const si::Angle from = g.setting("wind_from_deg") * deg;
    const PowerLawWind wind(speed, height, exponent, from);
    const si::Length cell = g.setting("dx") * m;
    const std::uint32_t nz = (std::uint32_t)g.setting("Nz");
    const LevelGrid grid(nz, cell);
    const si::Duration dt = g.setting("dt") * s;
    const si::LatticeConverter converter(cell, dt);
    for(std::uint32_t z = 0u; z < grid.levels(); z++) {
        const HorizontalWind w = wind.at(grid.height(z));
        const double x = w.x.in(m / s);
        const double expected_x = g.level(z, "wind_target_x_mps");
        EXPECT_EQ(x, expected_x) << "z " << z;
        const double y = w.y.in(m / s);
        const double expected_y = g.level(z, "wind_target_y_mps");
        EXPECT_EQ(y, expected_y) << "z " << z;
        const bool fluid = z >= grid.first_fluid() && z <= grid.last_fluid();
        float ux{ 0.0f }; // the floor and the lid get no start wind
        float uy{ 0.0f };
        if(fluid) {
            ux = converter.velocity(w.x);
            uy = converter.velocity(w.y);
        }
        const float expected_ux = (float)g.level(z, "start_ux");
        EXPECT_EQ(ux, expected_ux) << "z " << z;
        const float expected_uy = (float)g.level(z, "start_uy");
        EXPECT_EQ(uy, expected_uy) << "z " << z;
    }
}

} // namespace
