#include "setup/atmosphere/moist_column.hpp"
#include "setup/core/setup_error.hpp"
#include "golden_capture.hpp"

#include "moist_air.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

namespace {

using namespace si::symbols;

/// @param theta theta_l, the same at every height
/// @param qt the total water, the same at every height
/// @return a uniform sounding to 5 km, referenced to theta
Sounding uniform(si::Temperature theta, si::WaterContent qt) {
    const std::vector<TemperaturePoint> thl_points{ { 0.0 * m, theta }, { 5000.0 * m, theta } };
    const std::vector<WaterContentPoint> qt_points{ { 0.0 * m, qt }, { 5000.0 * m, qt } };
    const TemperatureProfile thl = TemperatureProfile::from_points(thl_points);
    const WaterContentProfile total_water = WaterContentProfile::from_points(qt_points);
    const Sounding sounding = Sounding::tabulated(thl, total_water, theta);
    return sounding;
}

/// @param h a height above the ground
/// @param theta a constant dry potential temperature
/// @return the exact hydrostatic pressure from 101325 Pa: dp/dz = -g p / (Rd theta (p / 1e5)^k) integrates to
/// p^k = p0^k - g k 1e5^k z / (Rd theta), k = Rd / cp (derived here with Rd 287.04, cp 1004, g 9.81, not the column's step)
si::Pressure exact_dry_pressure(si::Length h, si::Temperature theta) {
    const double k = 287.04 / 1004.0;
    const double p0 = 101325.0;
    const double z = h.in(m);
    const double t = si::in_kelvin(theta);
    const double p0_k = std::pow(p0, k);
    const double g_k = 9.81 * k;
    const double reference_k = std::pow(1.0E5, k);
    const double g_k_reference = g_k * reference_k;
    const double drop_numerator = g_k_reference * z;
    const double drop_denominator = 287.04 * t;
    const double drop = drop_numerator / drop_denominator;
    const double base = p0_k - drop;
    const double exponent = 1.0 / k;
    const double pressure = std::pow(base, exponent);
    const si::Pressure exact = pressure * Pa;
    return exact;
}

/// @param cell the level spacing
/// @return the largest relative error of the dry column's pressure against the exact one to 2 km (a ratio)
double worst_dry_error(si::Length cell) {
    const si::Temperature theta = si::kelvin_point(300.0);
    const std::uint32_t levels = LevelGrid::levels_for(2000.0 * m, cell);
    const LevelGrid grid(levels, cell);
    const Sounding dry = uniform(theta, si::WaterContent{ au::ZERO });
    const MoistColumn column = MoistColumn::hydrostatic(dry, grid);
    double worst = 0.0;
    for(std::uint32_t z = 0u; z < grid.levels(); z++) {
        const si::Length height = au::max(grid.height(z), si::Length{ au::ZERO });
        const si::Pressure exact = exact_dry_pressure(height, theta);
        const si::Pressure difference = column.pressure(z) - exact;
        const double relative = au::as_raw_number(difference / exact);
        const double error = std::fabs(relative);
        worst = std::max(worst, error);
    }
    return worst;
}

// the explicit step against the exact dry column: within its derived error, the floor cells at the ground's pressure.
// The step takes T of the lower level, so the relative error grows as g dh / (2 Rd T) * dT / T (about 3.8e-5 at 10 m
// over 2 km): the bound 6e-5
TEST(MoistColumn, DryColumnIsTheExactOneWithinTheStepError) {
    const std::uint32_t levels = LevelGrid::levels_for(2000.0 * m, 10.0 * m);
    const LevelGrid grid(levels, 10.0 * m);
    const Sounding dry = uniform(si::kelvin_point(300.0), si::WaterContent{ au::ZERO });
    const MoistColumn column = MoistColumn::hydrostatic(dry, grid);
    const double ground = column.pressure(0u).in(Pa);
    EXPECT_EQ(ground, 101325.0);
    const std::uint32_t column_levels = column.levels();
    const std::uint32_t grid_levels = grid.levels();
    EXPECT_EQ(column_levels, grid_levels);
    const double worst = worst_dry_error(10.0 * m);
    EXPECT_LT(worst, 6.0E-5);
}

// the step is first order: half the cell, half the error
TEST(MoistColumn, PressureStepIsFirstOrder) {
    const double coarse = worst_dry_error(10.0 * m);
    const double fine = worst_dry_error(5.0 * m);
    const double ratio = coarse / fine;
    EXPECT_GT(ratio, 1.8);
    EXPECT_LT(ratio, 2.2);
}

// unsaturated air: theta_v = theta_l (1 + 0.61 qt), the pressure has no part in it
TEST(MoistColumn, ThetaVOfUnsaturatedAir) {
    const LevelGrid grid(50u, 10.0 * m);
    const Sounding moist = uniform(si::kelvin_point(300.0), 0.005 * kg_w / kg);
    const MoistColumn column = MoistColumn::hydrostatic(moist, grid);
    const double vapour_factor = 1.0 + 0.61 * 0.005;
    const double expected_theta_v = 300.0 * vapour_factor;
    for(std::uint32_t z : { 1u, 20u, 48u }) {
        const double theta_v = si::in_kelvin(column.theta_v(z));
        EXPECT_NEAR(theta_v, expected_theta_v, 1.0E-4) << "z " << z;
    }
    const std::size_t core_levels = column.core_theta_v().size();
    EXPECT_EQ(core_levels, 50u);
    const float core_pressure = column.core_pressure()[20];
    const float expected_core_pressure = (float)column.pressure(20u).in(Pa);
    EXPECT_EQ(core_pressure, expected_core_pressure);
}

// the saturation adjustment at a level: dry air holds no liquid, air with qt over qsat holds qt - qsat at most
TEST(MoistColumn, LiquidAtALevel) {
    const LevelGrid grid(50u, 10.0 * m);
    const Sounding moist = uniform(si::kelvin_point(300.0), 0.005 * kg_w / kg);
    const MoistColumn column = MoistColumn::hydrostatic(moist, grid);
    const double dry_liquid = column.liquid(si::kelvin_point(300.0), 0.005 * kg_w / kg, 1u).in(kg_w / kg);
    EXPECT_EQ(dry_liquid, 0.0);
    const si::WaterContent l = column.liquid(si::kelvin_point(290.0), 0.03 * kg_w / kg, 1u); // qsat near 290 K and 1013 hPa is about 0.012
    const double liquid = l.in(kg_w / kg);
    EXPECT_GT(liquid, 0.0);
    const double most = 0.03 - 0.010;
    EXPECT_LT(liquid, most);
}

// the isentropic column: the closed-form pressure at each level's height (the ground's at the floor's cells); the
// saturated environment of tests/physics/moist_bubble.cpp (thl 300 K, qt 26 g/kg) holds about 0.5 g/kg of liquid at
// the ground and 3.5 g/kg at 1200 m
TEST(MoistColumn, IsentropicColumn) {
    const LevelGrid grid(128u, 10.0 * m);
    const si::Temperature thl = si::kelvin_point(300.0);
    const si::WaterContent qt = 0.026 * kg_w / kg;
    const MoistColumn column = MoistColumn::isentropic(thl, qt, grid);
    const double ground = column.pressure(0u).in(Pa);
    EXPECT_EQ(ground, 101325.0);
    const double pressure_64 = column.pressure(64u).in(Pa);
    const double raw_pressure_64 = moist_air::hydrostatic_pressure(635.0, 300.0);
    EXPECT_EQ(pressure_64, raw_pressure_64);
    const double g_h = 9.81 * 635.0;
    const double cp_theta = 1004.0 * 300.0;
    const double fraction = g_h / cp_theta;
    const double base = 1.0 - fraction;
    const double exponent = 1004.0 / 287.04;
    const double closed_form = 101325.0 * std::pow(base, exponent);
    EXPECT_DOUBLE_EQ(pressure_64, closed_form);
    const double ground_liquid = column.liquid(thl, qt, 1u).in(kg_w / kg);
    EXPECT_NEAR(ground_liquid, 0.5E-3, 0.1E-3);
    const double high_liquid = column.liquid(thl, qt, 120u).in(kg_w / kg);
    EXPECT_NEAR(high_liquid, 3.5E-3, 0.1E-3); // 1195 m
    const double theta_v_64 = si::in_kelvin(column.theta_v(64u));
    const double raw_theta_v_64 = moist_air::virtual_theta(300.0, 0.026, pressure_64);
    EXPECT_EQ(theta_v_64, raw_theta_v_64);
    const MoistColumn dry = MoistColumn::isentropic(thl, si::WaterContent{ au::ZERO }, grid);
    const double dry_theta_v = si::in_kelvin(dry.theta_v(20u));
    EXPECT_DOUBLE_EQ(dry_theta_v, 300.0); // dry air: theta_v = theta
    EXPECT_THROW(MoistColumn::isentropic(si::kelvin_point(0.0), si::WaterContent{ au::ZERO }, grid), SetupError);
}

TEST(MoistColumn, RefusesDryAndBadGround) {
    const LevelGrid grid(50u, 10.0 * m);
    const IdealizedProfile profile = IdealizedProfile{ .inversion_base = 1000.0 * m, .jump = 8.0 * K, .jump_depth = 100.0 * m, .lapse = 3.0 * K / km };
    const Sounding dry = Sounding::idealized(si::kelvin_point(300.0), profile);
    EXPECT_THROW(MoistColumn::hydrostatic(dry, grid), SetupError);
    const Sounding moist = uniform(si::kelvin_point(300.0), 0.005 * kg_w / kg);
    EXPECT_THROW(MoistColumn::hydrostatic(moist, grid, 0.0 * Pa), SetupError);
}

// a column that has fewer levels than the box is refused with a SetupError before a parallel loop reads it
TEST(MoistColumn, RequireLevelsRefusesAShorterColumn) {
    const LevelGrid grid(50u, 10.0 * m);
    const MoistColumn column = MoistColumn::isentropic(si::kelvin_point(300.0), 0.005 * kg_w / kg, grid);
    EXPECT_NO_THROW(column.require_levels(50u));
    EXPECT_NO_THROW(column.require_levels(40u));
    EXPECT_THROW(column.require_levels(51u), SetupError);
}

// an extra moist case (FLUIDX3D_EXTRA_CBL, FLUIDX3D_EXTRA_SOUNDING): the pressure and theta_v at every level,
// bit for bit against its capture
TEST(MoistColumn, PrivateMoistCaseIsItsCapture) {
    const std::optional<std::string> folder = GoldenCapture::environment("FLUIDX3D_EXTRA_CBL");
    const std::optional<std::string> csv = GoldenCapture::environment("FLUIDX3D_EXTRA_SOUNDING");
    if(!folder || !csv) {
        GTEST_SKIP() << "FLUIDX3D_EXTRA_CBL and FLUIDX3D_EXTRA_SOUNDING are not set";
    }
    const GoldenCapture g = GoldenCapture::at(*folder);
    const si::Length zi = g.setting("zi") * m;
    const Sounding tabulated = Sounding::from_csv(*csv);
    const Sounding sounding = tabulated.with_reference_from_mixed_layer(zi);
    const std::uint32_t nz = (std::uint32_t)g.setting("Nz");
    const si::Length cell = g.setting("dx") * m;
    const LevelGrid grid(nz, cell);
    const MoistColumn column = MoistColumn::hydrostatic(sounding, grid);
    for(std::uint32_t z = 0u; z < grid.levels(); z++) {
        const double pressure = column.pressure(z).in(Pa);
        const double expected_pressure = g.level(z, "pressure_Pa");
        EXPECT_EQ(pressure, expected_pressure) << "z " << z;
        const double theta_v = si::in_kelvin(column.theta_v(z));
        const double expected_theta_v = g.level(z, "theta_v_K");
        EXPECT_EQ(theta_v, expected_theta_v) << "z " << z;
    }
}

} // namespace
