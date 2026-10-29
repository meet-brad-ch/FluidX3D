#include "setup/atmosphere/sounding.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/level_grid.hpp"
#include "golden_capture.hpp"

#include <gtest/gtest.h>

#include <fstream>

namespace {

using namespace si::symbols;

/// @param name a file name in the test folder
/// @param text the file's content
/// @return the file's path
std::filesystem::path write_file(const std::string& name, const std::string& text) {
    const std::filesystem::path folder = std::filesystem::path(FLUIDX3D_TEST_DIR);
    const std::filesystem::path path = folder / name;
    std::ofstream(path, std::ios::binary) << text;
    return path;
}

/// @return a three-point tabulated theta_l: 300 K at 0 m, 302 K at 100 m, 301 K at 200 m
TemperatureProfile three_points() {
    const std::vector<TemperaturePoint> points{ { 0.0 * m, si::kelvin_point(300.0) },
                                                             { 100.0 * m, si::kelvin_point(302.0) },
                                                             { 200.0 * m, si::kelvin_point(301.0) } };
    const TemperatureProfile profile = TemperatureProfile::from_points(points);
    return profile;
}

/// @return the idealized profile of the by-hand tests: zi 1000 m, +8 K over 100 m, 3 K/km above
IdealizedProfile thousand_metres() {
    const IdealizedProfile profile = IdealizedProfile{ .inversion_base = 1000.0 * m, .jump = 8.0 * K, .jump_depth = 100.0 * m, .lapse = 3.0 * K / km };
    return profile;
}

/// @param g a capture
/// @return its idealized start profile, from its settings
IdealizedProfile profile_of(const GoldenCapture& g) {
    const si::Length inversion_base = g.setting("zi") * m;
    const si::TemperatureDifference jump = g.setting("jump") * K;
    const si::Length jump_depth = g.setting("jump_depth") * m;
    const si::LapseRate lapse = g.setting("lapse") * K / m;
    const IdealizedProfile profile = IdealizedProfile{ .inversion_base = inversion_base, .jump = jump, .jump_depth = jump_depth, .lapse = lapse };
    return profile;
}

// the idealized start profile at every level: the verified example's values, bit for bit
// Regression for defect 4: the start's lattice temperature 1 is the converter's theta_0, the excess is over the
// sounding's own theta_0; a different theta_0 is refused (it gave a wrong lattice temperature without an error)
TEST(Sounding, RequireReferenceRefusesAnotherTheta0) {
    const IdealizedProfile profile{ .inversion_base = 974.0 * m, .jump = 8.0 * K, .jump_depth = 100.0 * m, .lapse = 3.0 * K / km };
    const Sounding sounding = Sounding::idealized(si::kelvin_point(300.0), profile);
    EXPECT_NO_THROW(sounding.require_reference(si::kelvin_point(300.0)));
    EXPECT_THROW(sounding.require_reference(si::kelvin_point(301.0)), SetupError);
}

TEST(Sounding, IdealizedIsTheVerifiedExamples) {
    for(const char* name : { "quick", "sp2011" }) {
        const GoldenCapture g(name);
        const si::Temperature theta_0 = si::kelvin_point(g.setting("theta_0"));
        const IdealizedProfile profile = profile_of(g);
        const Sounding air = Sounding::idealized(theta_0, profile);
        const double reference = si::in_kelvin(air.reference());
        const double expected_reference = g.setting("theta_0");
        ASSERT_EQ(reference, expected_reference) << name;
        for(std::size_t z = 0u; z < g.levels(); z++) {
            const si::Length h = g.level(z, "height_m") * m;
            const double excess = air.theta_excess(h).in(K);
            const double expected_excess = g.level(z, "start_theta_K");
            EXPECT_EQ(excess, expected_excess) << name << " z " << z;
        }
    }
}

// by hand: the jump spread linearly over its depth, the lapse above it, 0 below
TEST(Sounding, IdealizedByHand) {
    const Sounding air = Sounding::idealized(si::kelvin_point(300.0), thousand_metres());
    const double mixed = air.theta_excess(500.0 * m).in(K);
    EXPECT_EQ(mixed, 0.0);
    const double in_jump = air.theta_excess(1050.0 * m).in(K);
    EXPECT_EQ(in_jump, 4.0);
    const double above = air.theta_excess(2100.0 * m).in(K);
    const double expected_above = 8.0 + 0.003 * 1000.0;
    EXPECT_DOUBLE_EQ(above, expected_above);
    EXPECT_FALSE(air.has_water());
    EXPECT_TRUE(air.is_idealized());
    EXPECT_THROW(air.total_water(10.0 * m), SetupError);
    EXPECT_THROW(air.with_reference_from_mixed_layer(1000.0 * m), SetupError);
    // a step (depth 0): the jump at the inversion's base, no division by the depth
    const IdealizedProfile step_profile = IdealizedProfile{ .inversion_base = 180.0 * m, .jump = 1.0 * K, .jump_depth = 0.0 * m, .lapse = 5.0 * K / km };
    const Sounding step = Sounding::idealized(si::kelvin_point(300.0), step_profile);
    const double below_step = step.theta_excess(179.0 * m).in(K);
    EXPECT_EQ(below_step, 0.0);
    const double at_step = step.theta_excess(180.0 * m).in(K);
    EXPECT_EQ(at_step, 1.0);
}

// the parts a header writer reads: the idealized profile, or the tabulated theta_l
TEST(Sounding, Parts) {
    const Sounding i = Sounding::idealized(si::kelvin_point(300.0), thousand_metres());
    ASSERT_TRUE(i.idealized_profile().has_value());
    const double inversion_base = i.idealized_profile()->inversion_base.in(m);
    EXPECT_EQ(inversion_base, 1000.0);
    EXPECT_FALSE(i.tabulated_theta_l().has_value());
    const Sounding t = Sounding::tabulated(three_points(), std::nullopt, si::kelvin_point(300.0));
    EXPECT_FALSE(t.idealized_profile().has_value());
    ASSERT_TRUE(t.tabulated_theta_l().has_value());
    const std::size_t points = t.tabulated_theta_l()->points().size();
    EXPECT_EQ(points, 3u);
}

TEST(Sounding, IdealizedRefusesBadValues) {
    EXPECT_THROW(Sounding::idealized(si::kelvin_point(0.0), thousand_metres()), SetupError);
    const IdealizedProfile negative_depth = IdealizedProfile{ .inversion_base = 1000.0 * m, .jump = 8.0 * K, .jump_depth = -1.0 * m, .lapse = 3.0 * K / km };
    EXPECT_THROW(Sounding::idealized(si::kelvin_point(300.0), negative_depth), SetupError);
    const IdealizedProfile negative_base = IdealizedProfile{ .inversion_base = -1.0 * m, .jump = 8.0 * K, .jump_depth = 100.0 * m, .lapse = 3.0 * K / km };
    EXPECT_THROW(Sounding::idealized(si::kelvin_point(300.0), negative_base), SetupError);
}

// by hand: linear between the points, held at the end values outside them, minus theta_0
TEST(Sounding, TabulatedByHand) {
    const Sounding air = Sounding::tabulated(three_points(), std::nullopt, si::kelvin_point(300.0));
    const double below = air.theta_excess(-10.0 * m).in(K);
    EXPECT_EQ(below, 0.0);
    const double at_50 = air.theta_excess(50.0 * m).in(K);
    EXPECT_EQ(at_50, 1.0);
    const double at_100 = air.theta_excess(100.0 * m).in(K);
    EXPECT_EQ(at_100, 2.0);
    const double at_150 = air.theta_excess(150.0 * m).in(K);
    EXPECT_EQ(at_150, 1.5);
    const double above = air.theta_excess(900.0 * m).in(K);
    EXPECT_EQ(above, 1.0);
    EXPECT_FALSE(air.is_idealized());
    // no reference yet: theta_excess and reference refuse
    const Sounding open = Sounding::tabulated(three_points(), std::nullopt, std::nullopt);
    EXPECT_THROW(open.reference(), SetupError);
    EXPECT_THROW(open.theta_excess(50.0 * m), SetupError);
}

// the mixed layer's median: strictly inside (0.2 zi, 0.8 zi), the upper middle value for an even count
TEST(Sounding, ReferenceFromTheMixedLayer) {
    std::vector<TemperaturePoint> points{};
    const double theta[] = { 310.0, 305.0, 301.0, 299.0, 300.0, 298.0, 302.0, 290.0, 320.0, 330.0 };
    const si::Length spacing = 100.0 * m;
    for(int i = 0; i < 10; i++) {
        const si::Length height = spacing * (double)i;
        const si::Temperature value = si::kelvin_point(theta[i]);
        points.push_back({ height, value });
    }
    const TemperatureProfile profile = TemperatureProfile::from_points(points);
    const Sounding air = Sounding::tabulated(profile, std::nullopt, std::nullopt);
    // zi 1000 m: (200, 800) holds 300..700 m: 299 300 298 302 290 -> sorted 290 298 299 300 302 -> 299
    const double at_1000 = si::in_kelvin(air.with_reference_from_mixed_layer(1000.0 * m).reference());
    EXPECT_EQ(at_1000, 299.0);
    // zi 900 m: (180, 720) holds 200..700 m: 301 299 300 298 302 290 -> sorted 290 298 299 300 301 302 -> the upper middle 300
    const double at_900 = si::in_kelvin(air.with_reference_from_mixed_layer(900.0 * m).reference());
    EXPECT_EQ(at_900, 300.0);
    // zi 1250 m: (250, 1000) holds 300..900 m: 299 300 298 302 290 320 330 -> sorted 290 298 299 300 302 320 330 -> 300
    const double at_1250 = si::in_kelvin(air.with_reference_from_mixed_layer(1250.0 * m).reference());
    EXPECT_EQ(at_1250, 300.0);
    // zi 120 m: (24, 96): no point
    EXPECT_THROW(air.with_reference_from_mixed_layer(120.0 * m), SetupError);
}

TEST(Sounding, ReadsACsv) {
    const std::filesystem::path dry = write_file("sounding_dry.csv", "z_m,thl_K\n0,300\n100,302\r\n200,301\n\n");
    const Sounding air = Sounding::from_csv(dry);
    EXPECT_FALSE(air.has_water());
    // zi 250 m: (50, 200) holds only 100 m: theta_0 302 K; at 150 m 301.5 K, so -0.5 K
    const Sounding referenced = air.with_reference_from_mixed_layer(250.0 * m);
    const double reference = si::in_kelvin(referenced.reference());
    EXPECT_EQ(reference, 302.0);
    const double at_150 = referenced.theta_excess(150.0 * m).in(K);
    EXPECT_EQ(at_150, -0.5);
    const std::filesystem::path moist = write_file("sounding_moist.csv", "z_m,thl_K,qt_kgkg\n0,300,0.010\n100,302,0.008\n");
    const Sounding wet = Sounding::from_csv(moist);
    ASSERT_TRUE(wet.has_water());
    const double at_50 = wet.total_water(50.0 * m).in(kg_w / kg);
    EXPECT_DOUBLE_EQ(at_50, 0.009);
    const double at_500 = wet.total_water(500.0 * m).in(kg_w / kg);
    EXPECT_EQ(at_500, 0.008);
}

TEST(Sounding, CsvErrorsNameTheFileAndLine) {
    const std::filesystem::path missing = std::filesystem::path(FLUIDX3D_TEST_DIR) / "no_such_sounding.csv";
    EXPECT_THROW(Sounding::from_csv(missing), SetupError);
    const std::filesystem::path one_row = write_file("sounding_one.csv", "z_m,thl_K\n0,300\n");
    EXPECT_THROW(Sounding::from_csv(one_row), SetupError);
    const std::filesystem::path no_qt = write_file("sounding_noqt.csv", "z_m,thl_K,qt_kgkg\n0,300,0.01\n100,302\n");
    EXPECT_THROW(Sounding::from_csv(no_qt), SetupError);
    const std::filesystem::path text = write_file("sounding_text.csv", "z_m,thl_K\n0,300\nabc,302\n");
    try {
        Sounding::from_csv(text);
        FAIL() << "a row that is not a number must throw";
    } catch(const SetupError& e) {
        const std::string message = std::string(e.what());
        const std::size_t at = message.find("line 3");
        EXPECT_NE(at, std::string::npos) << e.what();
    }
    const std::filesystem::path down = write_file("sounding_down.csv", "z_m,thl_K\n100,300\n0,302\n");
    EXPECT_THROW(Sounding::from_csv(down), SetupError);
}

/// @brief Expects from_csv() to refuse a file with a SetupError that holds a part of the message.
/// @param name the file's name in the test folder
/// @param text the file's text
/// @param part a part of the expected message
void expect_csv_fault(const std::string& name, const std::string& text, const std::string& part) {
    const std::filesystem::path path = write_file(name, text);
    try {
        Sounding::from_csv(path);
        ADD_FAILURE() << "no error, expected: " << part << " (" << name << ")";
    } catch(const SetupError& e) {
        const std::string message = std::string(e.what());
        const std::size_t at = message.find(part);
        EXPECT_NE(at, std::string::npos) << e.what();
    }
}

// the header names the columns, and a value is a number with nothing after it
TEST(Sounding, CsvRefusesABadHeaderAndTextAfterANumber) {
    expect_csv_fault("sounding_names.csv", "height,theta\n0,300\n100,302\n", "header");
    expect_csv_fault("sounding_no_header.csv", "0,300\n100,302\n200,301\n", "header");
    expect_csv_fault("sounding_unit.csv", "z_m,thl_K\n0,300 K\n100,302\n", "line 2");
    expect_csv_fault("sounding_moist_unit.csv", "z_m,thl_K,qt_kgkg\n0,300 K,0.01\n100,302,0.01\n", "line 2");
    expect_csv_fault("sounding_extra.csv", "z_m,thl_K\n0,300,0.01\n100,302,0.01\n", "line 2");
    const std::filesystem::path spaces = write_file("sounding_spaces.csv", "z_m,thl_K\r\n0, 300 \r\n100,302\r\n");
    EXPECT_NO_THROW(Sounding::from_csv(spaces));
}

// an extra moist case (outside the repository: two environment variables): its sounding's start profile and total
// water at every level, bit for bit against its capture
TEST(Sounding, PrivateMoistCaseIsItsCapture) {
    const std::optional<std::string> folder = GoldenCapture::environment("FLUIDX3D_EXTRA_CBL");
    const std::optional<std::string> csv = GoldenCapture::environment("FLUIDX3D_EXTRA_SOUNDING");
    if(!folder || !csv) {
        GTEST_SKIP() << "FLUIDX3D_EXTRA_CBL and FLUIDX3D_EXTRA_SOUNDING are not set";
    }
    const GoldenCapture g = GoldenCapture::at(*folder);
    const si::Length zi = g.setting("zi") * m;
    const Sounding tabulated = Sounding::from_csv(*csv);
    const Sounding air = tabulated.with_reference_from_mixed_layer(zi);
    const double reference = si::in_kelvin(air.reference());
    const double expected_reference = g.setting("theta_0");
    ASSERT_EQ(reference, expected_reference);
    const std::uint32_t nz = (std::uint32_t)g.setting("Nz");
    const si::Length cell = g.setting("dx") * m;
    const LevelGrid grid(nz, cell);
    for(std::uint32_t z = 0u; z < grid.levels(); z++) {
        const si::Length h = grid.height(z);
        const double excess = air.theta_excess(h).in(K);
        const double expected_excess = g.level(z, "start_theta_K");
        EXPECT_EQ(excess, expected_excess) << "z " << z;
        // the example reads qt at the height clamped into the fluid (the floor's cell gets the lowest fluid value)
        const si::Length top_fluid = grid.height(grid.levels() - 2u);
        const si::Length clamped = au::clamp(h, si::Length{ au::ZERO }, top_fluid);
        const double qt = air.total_water(clamped).in(kg_w / kg);
        const double expected_qt = g.level(z, "qt_kgkg");
        EXPECT_EQ(qt, expected_qt) << "z " << z;
    }
}

} // namespace
