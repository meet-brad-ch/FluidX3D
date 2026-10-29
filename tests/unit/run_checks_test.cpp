#include "setup/atmosphere/run_checks.hpp"
#include "setup/core/setup_error.hpp"
#include "golden_capture.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <regex>

namespace {

using namespace si::symbols;

/// @param levels the levels
/// @param value the water content of every level
/// @return the water content per level
std::vector<si::WaterContent> water_levels(std::size_t levels, si::WaterContent value) {
    const std::vector<si::WaterContent> water = std::vector<si::WaterContent>(levels, value);
    return water;
}

// by hand: levels 3-5 at or over 1e-3 wet, the largest fraction and ql wherever they are
TEST(CloudExtent, ByHand) {
    const LevelGrid grid(10u, 10.0 * m);
    const std::vector<si::WaterContent> wet = water_levels(10u, 0.01 * kg_w / kg);
    const std::vector<si::WaterContent> no_liquid = water_levels(10u, si::WaterContent{ au::ZERO });
    const std::vector<double> no_fraction = std::vector<double>(10u, 0.0);
    CloudLevels c = CloudLevels{ .qt = wet, .ql = no_liquid, .fraction = no_fraction };
    c.fraction[3] = 1.0E-3;
    c.fraction[4] = 0.2;
    c.fraction[5] = 0.05;
    c.fraction[7] = 0.0009;
    c.ql[4] = 2.0E-5 * kg_w / kg;
    c.ql[7] = 3.0E-5 * kg_w / kg;
    const CloudExtent e = CloudExtent::of(c, grid, 1.0E-3);
    ASSERT_TRUE(e.base && e.top);
    const double base = e.base->in(m);
    EXPECT_EQ(base, 25.0);
    const double top = e.top->in(m);
    EXPECT_EQ(top, 45.0);
    EXPECT_EQ(e.cover, 0.2);
    const double ql_max = e.ql_max.in(kg_w / kg);
    EXPECT_EQ(ql_max, 3.0E-5);
    const std::vector<si::WaterContent> dry = water_levels(10u, si::WaterContent{ au::ZERO });
    const CloudLevels clear_levels = CloudLevels{ .qt = dry, .ql = no_liquid, .fraction = no_fraction };
    const CloudExtent clear = CloudExtent::of(clear_levels, grid, 1.0E-3);
    EXPECT_FALSE(clear.base.has_value());
    EXPECT_FALSE(clear.top.has_value());
}

// by hand: the spin-up does not count; intervals without a cloud count as recorded, not as cloudy
TEST(CloudBaseCheck, ByHand) {
    CloudBaseCheck check{};
    check.add(false, 500.0 * m, 1000.0 * m); // spin-up
    check.add(true, 1000.0 * m, 1000.0 * m);
    check.add(true, 1100.0 * m, 1000.0 * m);
    check.add(true, std::nullopt, 1000.0 * m);
    const CheckResult r = check.result();
    EXPECT_TRUE(r.passed);
    const std::string expected_line = "RESULT CLOUD the mean cloud base 1050 m against the mean condensation level 1000 m: 5.0 % (limit 10 %) PASSED; a cloud in 2 of 3 recorded intervals";
    EXPECT_EQ(r.line, expected_line);
    CloudBaseCheck high{};
    high.add(true, 1200.0 * m, 1000.0 * m);
    const CheckResult high_result = high.result();
    EXPECT_FALSE(high_result.passed);
    CloudBaseCheck none{};
    none.add(true, std::nullopt, std::nullopt);
    none.add(true, std::nullopt, 1000.0 * m);
    const CheckResult none_result = none.result();
    EXPECT_FALSE(none_result.passed);
    const CheckResult none_again = none.result();
    const std::string expected_none_line = "RESULT CLOUD FAILED: no cloud in the 2 recorded intervals";
    EXPECT_EQ(none_again.line, expected_none_line);
    EXPECT_THROW(CloudBaseCheck(0.0), SetupError);
}

TEST(BlowUpCheck, LimitsAndNonFinite) {
    const BlowUpCheck check{};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double infinity = std::numeric_limits<double>::infinity();
    const std::optional<std::string> calm = check.failure(100.0 * s, -3.0 * m / s, 5.0 * m / s);
    EXPECT_FALSE(calm.has_value());
    const std::optional<std::string> at_limit = check.failure(100.0 * s, -40.0 * m / s, 40.0 * m / s);
    EXPECT_FALSE(at_limit.has_value());
    const std::optional<std::string> over_up = check.failure(120.0 * s, -2.0 * m / s, 40.5 * m / s);
    const std::string expected_message = "FAILED: the flow blew up at t 120 s (w -2.00..40.50 m/s)";
    EXPECT_EQ(*over_up, expected_message);
    const std::optional<std::string> over_down = check.failure(100.0 * s, -41.0 * m / s, 5.0 * m / s);
    EXPECT_TRUE(over_down.has_value());
    const si::Speed not_a_number = nan * m / s;
    const std::optional<std::string> nan_up = check.failure(100.0 * s, -1.0 * m / s, not_a_number);
    EXPECT_TRUE(nan_up.has_value());
    const si::Speed minus_infinity = -infinity * m / s;
    const std::optional<std::string> infinite_down = check.failure(100.0 * s, minus_infinity, 1.0 * m / s);
    EXPECT_TRUE(infinite_down.has_value());
    EXPECT_THROW(BlowUpCheck(0.0 * m / s), SetupError);
}

// a recorded moist run's log (FLUIDX3D_EXTRA_RUN_LOG): its CLOUD lines through the check give its RESULT CLOUD line
// (measured on the 10 m and the 5 m runs)
/// @brief The lines of a run's log that the cloud base check needs.
struct RunLog {
    /// @brief One CLOUD line: a height in metres as printed, -1 for none.
    struct Interval {
        double base_m = 0.0;  ///< the line's cloud base
        double level_m = 0.0; ///< the line's condensation level
    };
    std::vector<Interval> intervals{}; ///< the CLOUD lines, spin-up first
    std::string expected{};            ///< the RESULT CLOUD line, or empty
    std::size_t recorded = 0u;         ///< its recorded intervals
    std::optional<LevelGrid> grid{};   ///< the levels of the SETUP line

    /// @param in the log
    /// @return its CLOUD, RESULT CLOUD and SETUP lines
    static RunLog read(std::istream& in) {
        const std::regex cloud(R"(^CLOUD t (\d+) s: base (-?\d+) m, top (-?\d+) m, .* condensation level (-?\d+) m)");
        const std::regex result(R"(^RESULT CLOUD .* of (\d+) recorded intervals)");
        const std::regex setup(R"(^SETUP \d+ x \d+ x (\d+) cells of ([0-9.]+) m)");
        RunLog r{};
        for(std::string line; std::getline(in, line);) {
            if(!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            std::smatch match{};
            if(std::regex_search(line, match, cloud)) {
                const double base_m = std::stod(match[2]);
                const double level_m = std::stod(match[4]);
                r.intervals.push_back(Interval{ .base_m = base_m, .level_m = level_m });
            }
            if(std::regex_search(line, match, result)) {
                r.expected = line;
                r.recorded = (std::size_t)std::stoul(match[1]);
            }
            if(std::regex_search(line, match, setup)) {
                const std::uint32_t levels = (std::uint32_t)std::stoul(match[1]);
                const si::Length cell = std::stod(match[2]) * m;
                r.grid.emplace(levels, cell);
            }
        }
        return r;
    }
};

/// @brief The lines print the level heights to the metre (at 5 m cells they end in .5 m): back to the level's height.
/// @param grid the run's levels
/// @param metres a height as printed (at or below 0: none)
/// @return the height of the level nearest to it, or none
std::optional<si::Length> level_height(const LevelGrid& grid, double metres) {
    if(!(metres > 0.0)) {
        return std::nullopt;
    }
    const si::Length printed = metres * m;
    const std::uint32_t level = grid.nearest_level(printed);
    const si::Length height = grid.height(level);
    return height;
}

TEST(CloudBaseCheck, PrivateRunGivesItsResultLine) {
    const std::optional<std::string> log = GoldenCapture::environment("FLUIDX3D_EXTRA_RUN_LOG");
    if(!log) {
        GTEST_SKIP() << "FLUIDX3D_EXTRA_RUN_LOG is not set";
    }
    std::ifstream in(*log);
    ASSERT_TRUE(in.good()) << *log;
    const RunLog r = RunLog::read(in);
    ASSERT_FALSE(r.expected.empty()) << "no RESULT CLOUD line in " << *log;
    ASSERT_LE(r.recorded, r.intervals.size());
    ASSERT_TRUE(r.grid.has_value()) << "no SETUP line in " << *log;
    CloudBaseCheck check{};
    const LevelGrid& grid = *r.grid;
    const std::size_t first_recorded = r.intervals.size() - r.recorded;
    for(std::size_t i = 0u; i < r.intervals.size(); i++) {
        const bool recording = i >= first_recorded;
        const std::optional<si::Length> base = level_height(grid, r.intervals[i].base_m);
        const std::optional<si::Length> level = level_height(grid, r.intervals[i].level_m);
        check.add(recording, base, level);
    }
    const CheckResult result = check.result();
    EXPECT_EQ(result.line, r.expected);
}

} // namespace
