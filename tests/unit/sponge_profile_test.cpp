#include "setup/atmosphere/sponge_profile.hpp"
#include "setup/core/setup_error.hpp"
#include "golden_capture.hpp"

#include <gtest/gtest.h>

namespace {

using namespace si::symbols;

/// @return the conversions of 10 m cells and 0.16 s steps
si::LatticeConverter ten_metres() {
    const si::LatticeConverter converter = si::LatticeConverter(10.0 * m, 0.16 * s);
    return converter;
}

/// @param g a capture
/// @return the conversions of its cell and time step
si::LatticeConverter converter_of(const GoldenCapture& g) {
    const si::Length dx = g.setting("dx") * m;
    const si::Duration dt = g.setting("dt") * s;
    const si::LatticeConverter converter = si::LatticeConverter(dx, dt);
    return converter;
}

/// @param g a capture
/// @return its levels
LevelGrid grid_of(const GoldenCapture& g) {
    const std::uint32_t nz = (std::uint32_t)g.setting("Nz");
    const si::Length cell = g.setting("dx") * m;
    const LevelGrid grid = LevelGrid(nz, cell);
    return grid;
}

// by hand: 0 below and at the start, a quarter of dt / tau half-way, dt / tau at the lid
TEST(SpongeProfile, QuadraticByHand) {
    const SpongeProfile sponge = SpongeProfile::quadratic(1000.0 * m, 400.0 * s);
    const si::LatticeConverter converter = ten_metres();
    const double top = (double)0.16f / 400.0; // the core's float step over tau
    const float below = sponge.lattice_rate(500.0 * m, 2000.0 * m, converter);
    EXPECT_EQ(below, 0.0f);
    const float at_start = sponge.lattice_rate(1000.0 * m, 2000.0 * m, converter);
    EXPECT_EQ(at_start, 0.0f);
    const float half_way = sponge.lattice_rate(1500.0 * m, 2000.0 * m, converter);
    const double quarter_top = 0.25 * top;
    const float expected_half_way = (float)quarter_top;
    EXPECT_FLOAT_EQ(half_way, expected_half_way);
    const float at_lid = sponge.lattice_rate(2000.0 * m, 2000.0 * m, converter);
    const float expected_at_lid = (float)top;
    EXPECT_FLOAT_EQ(at_lid, expected_at_lid);
    EXPECT_TRUE(sponge.active());
    const std::string described = sponge.describe();
    const std::size_t at = described.find("from 1000 m");
    EXPECT_NE(at, std::string::npos) << described;
}

// per level: the lid is the top level's centre, so the top level gets dt / tau; the rate rises with the height
TEST(SpongeProfile, RatesPerLevel) {
    const LevelGrid grid(207u, 10.0 * m);
    const si::LatticeConverter converter = ten_metres();
    const SpongeProfile sponge = SpongeProfile::quadratic(1600.0 * m, 450.0 * s);
    const std::vector<float> rates = sponge.lattice_rates(grid, converter);
    ASSERT_EQ(rates.size(), 207u);
    const float expected_top = converter.per_step(450.0 * s);
    EXPECT_FLOAT_EQ(rates[206], expected_top);
    EXPECT_EQ(rates[160], 0.0f); // 1595 m, below 1600 m
    for(std::uint32_t z = 162u; z < 207u; z++) {
        EXPECT_GT(rates[z], rates[z - 1u]) << "z " << z;
    }
}

// no sponge: 0 everywhere, the verified examples' (no sponge in quick and sp2011) bit for bit
TEST(SpongeProfile, NoneIsTheVerifiedExamples) {
    for(const char* name : { "quick", "sp2011" }) {
        const GoldenCapture g(name);
        const double sponge_from = g.setting("sponge_from");
        ASSERT_EQ(sponge_from, 0.0) << name;
        const LevelGrid grid = grid_of(g);
        const si::LatticeConverter converter = converter_of(g);
        const std::vector<float> rates = SpongeProfile::none().lattice_rates(grid, converter);
        for(std::size_t z = 0u; z < g.levels(); z++) {
            const float expected_rate = (float)g.level(z, "sponge_rate");
            EXPECT_EQ(rates[z], expected_rate) << name << " z " << z;
        }
    }
    EXPECT_FALSE(SpongeProfile::none().active());
}

TEST(SpongeProfile, RefusesBadValues) {
    EXPECT_THROW(SpongeProfile::quadratic(0.0 * m, 450.0 * s), SetupError);
    EXPECT_THROW(SpongeProfile::quadratic(1000.0 * m, 0.0 * s), SetupError);
    const si::LatticeConverter converter = ten_metres();
    EXPECT_THROW(SpongeProfile::quadratic(1000.0 * m, 450.0 * s).lattice_rate(900.0 * m, 1000.0 * m, converter), SetupError);
}

// an extra case with a sponge (FLUIDX3D_EXTRA_CBL): the rate at every level bit for bit against its capture
TEST(SpongeProfile, PrivateCaseIsItsCapture) {
    const std::optional<std::string> folder = GoldenCapture::environment("FLUIDX3D_EXTRA_CBL");
    if(!folder) {
        GTEST_SKIP() << "FLUIDX3D_EXTRA_CBL is not set";
    }
    const GoldenCapture g = GoldenCapture::at(*folder);
    const double sponge_from_m = g.setting("sponge_from");
    ASSERT_GT(sponge_from_m, 0.0);
    const si::Length sponge_from = sponge_from_m * m;
    const si::Duration sponge_tau = g.setting("sponge_tau") * s;
    const SpongeProfile sponge = SpongeProfile::quadratic(sponge_from, sponge_tau);
    const LevelGrid grid = grid_of(g);
    const si::LatticeConverter converter = converter_of(g);
    const std::vector<float> rates = sponge.lattice_rates(grid, converter);
    for(std::size_t z = 0u; z < g.levels(); z++) {
        const float expected_rate = (float)g.level(z, "sponge_rate");
        EXPECT_EQ(rates[z], expected_rate) << "z " << z;
    }
}

} // namespace
