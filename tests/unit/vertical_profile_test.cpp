#include "setup/atmosphere/vertical_profile.hpp"
#include "setup/core/setup_error.hpp"

#include <gtest/gtest.h>

namespace {

using namespace si::symbols;

// by hand: linear between the points, the end values below the first and above the last
TEST(Profile, LinearBetweenHeldOutside) {
    const std::vector<SpeedPoint> points{ { 10.0 * m, 2.0 * m / s }, { 30.0 * m, 6.0 * m / s }, { 70.0 * m, 4.0 * m / s } };
    const SpeedProfile p = SpeedProfile::from_points(points);
    const double below = p.at(0.0 * m).in(m / s);
    EXPECT_EQ(below, 2.0);
    const double at_first = p.at(10.0 * m).in(m / s);
    EXPECT_EQ(at_first, 2.0);
    const double between_first = p.at(20.0 * m).in(m / s);
    EXPECT_EQ(between_first, 4.0);
    const double at_second = p.at(30.0 * m).in(m / s);
    EXPECT_EQ(at_second, 6.0);
    const double between_second = p.at(50.0 * m).in(m / s);
    EXPECT_EQ(between_second, 5.0);
    const double above = p.at(100.0 * m).in(m / s);
    EXPECT_EQ(above, 4.0);
    const double bottom = p.bottom().in(m);
    EXPECT_EQ(bottom, 10.0);
    const double top = p.top().in(m);
    EXPECT_EQ(top, 70.0);
    const std::size_t count = p.size();
    EXPECT_EQ(count, 3u);
}

// a water content and a temperature reading (a point on the scale: a + f (b - a)) interpolate the same way
TEST(Profile, HoldsWaterContentAndTemperature) {
    const std::vector<WaterContentPoint> water_points{ { 0.0 * m, 10.0 * g_w / kg }, { 100.0 * m, 6.0 * g_w / kg } };
    const WaterContentProfile q = WaterContentProfile::from_points(water_points);
    const double q_25 = q.at(25.0 * m).in(g_w / kg);
    EXPECT_DOUBLE_EQ(q_25, 9.0);
    const double q_200 = q.at(200.0 * m).in(g_w / kg);
    EXPECT_DOUBLE_EQ(q_200, 6.0); // held in kg/kg: back in g/kg within rounding
    const std::vector<TemperaturePoint> temperature_points{ { 0.0 * m, si::kelvin_point(300.0) }, { 100.0 * m, si::kelvin_point(302.0) } };
    const TemperatureProfile t = TemperatureProfile::from_points(temperature_points);
    const double t_25 = si::in_kelvin(t.at(25.0 * m));
    EXPECT_DOUBLE_EQ(t_25, 300.5);
}

TEST(Profile, RefusesTooFewOrUnorderedPoints) {
    const std::vector<SpeedPoint> one{ { 0.0 * m, 1.0 * m / s } };
    EXPECT_THROW(SpeedProfile::from_points(one), SetupError);
    const std::vector<SpeedPoint> same_height{ { 0.0 * m, 1.0 * m / s }, { 0.0 * m, 2.0 * m / s } };
    EXPECT_THROW(SpeedProfile::from_points(same_height), SetupError);
    const std::vector<SpeedPoint> downwards{ { 10.0 * m, 1.0 * m / s }, { 5.0 * m, 2.0 * m / s } };
    EXPECT_THROW(SpeedProfile::from_points(downwards), SetupError);
}

} // namespace
