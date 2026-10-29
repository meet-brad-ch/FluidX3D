#include "setup/core/scalar_transport_rules.hpp"
#include "setup/core/setup_error.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace {

/// @return a build with the given transport extensions (the others off)
TransportBuild build_with(bool fix, bool finite_volumes, bool fp16) {
    TransportBuild build{};
    build.diffusivity_fix = fix;
    build.finite_volumes = finite_volumes;
    build.fp16_ddfs = fp16;
    return build;
}

// src/lbm.cpp: with the fix tauT = 4 alpha + 1/2, without it 2 alpha + 1/2; the lattice diffuses with (tauT - 1/2)/4
TEST(ScalarTransportRules, CoreAlphaMakesTheLatticeDiffuseWithTheWantedValue) {
    const float wanted = 0.0625f;
    const float fixed = ScalarTransportRules::core_alpha(wanted, build_with(true, false, false));
    const float unfixed = ScalarTransportRules::core_alpha(wanted, build_with(false, false, false));
    EXPECT_EQ(fixed, wanted);
    EXPECT_EQ(unfixed, 2.0f * wanted);
    EXPECT_FLOAT_EQ(((4.0f * fixed + 0.5f) - 0.5f) / 4.0f, wanted);   // the core's rate with the fix
    EXPECT_FLOAT_EQ(((2.0f * unfixed + 0.5f) - 0.5f) / 4.0f, wanted); // the core's rate without the fix
    EXPECT_EQ(ScalarTransportRules::core_alpha(wanted, build_with(true, true, false)), wanted); // finite volumes
    EXPECT_EQ(ScalarTransportRules::core_alpha(0.0f, build_with(false, false, false)), 0.0f);
}

TEST(ScalarTransportRules, CoreAlphaRefusesInvalidInput) {
    EXPECT_THROW(ScalarTransportRules::core_alpha(-1.0E-4f, build_with(true, false, false)), SetupError);
    EXPECT_THROW(ScalarTransportRules::core_alpha(1.0E-4f, build_with(false, true, false)), SetupError); // SCALAR_FV without the fix
}

// Regression for defect 16: a NaN or an infinite diffusivity is refused (NaN < 0 is false, so it passed)
TEST(ScalarTransportRules, CoreAlphaRefusesANonFiniteDiffusivity) {
    const float not_a_number = std::numeric_limits<float>::quiet_NaN();
    EXPECT_THROW(ScalarTransportRules::core_alpha(not_a_number, build_with(true, false, false)), SetupError);
    const float infinite = std::numeric_limits<float>::infinity();
    EXPECT_THROW(ScalarTransportRules::core_alpha(infinite, build_with(true, false, false)), SetupError);
}

TEST(ScalarTransportRules, RelaxationTime) {
    EXPECT_FLOAT_EQ(ScalarTransportRules::relaxation_time(0.014f), 0.556f);
    EXPECT_EQ(ScalarTransportRules::relaxation_time(0.0f), 0.5f);
}

TEST(ScalarTransportRules, RefusesTheD3Q7LatticeBelowTheWallSafeRateNextToWallsWithBuoyancy) {
    const TransportBuild d3q7 = build_with(true, false, false);
    const ScalarContext walls_buoyant{ true, true, false };
    EXPECT_TRUE(ScalarTransportRules::refusal(1.0E-4f, d3q7, walls_buoyant).has_value());
    EXPECT_FALSE(ScalarTransportRules::refusal(0.015f, d3q7, walls_buoyant).has_value());
    EXPECT_FALSE(ScalarTransportRules::refusal(1.0E-4f, d3q7, ScalarContext{ false, true, false }).has_value()); // no walls
    EXPECT_FALSE(ScalarTransportRules::refusal(1.0E-4f, d3q7, ScalarContext{ true, false, false }).has_value()); // no buoyancy
    EXPECT_FALSE(ScalarTransportRules::refusal(1.0E-4f, build_with(true, true, false), walls_buoyant).has_value()); // finite volumes
}

TEST(ScalarTransportRules, RefusesFp16WithTheD3Q7LatticeButNotWithFiniteVolumes) {
    const ScalarContext open{ false, true, false };
    EXPECT_TRUE(ScalarTransportRules::refusal(0.1f, build_with(true, false, true), open).has_value());
    EXPECT_FALSE(ScalarTransportRules::refusal(0.1f, build_with(true, true, true), open).has_value());
}

TEST(ScalarTransportRules, WarningsForTheGenericSimulation) {
    const ScalarContext buoyant{ true, true, false };
    EXPECT_EQ(ScalarTransportRules::warnings(1.0E-4f, build_with(true, false, false), buoyant).size(), 1u); // tauT below 0.52
    EXPECT_TRUE(ScalarTransportRules::warnings(0.01f, build_with(true, false, false), buoyant).empty());     // tauT 0.54
    EXPECT_EQ(ScalarTransportRules::warnings(0.01f, build_with(true, false, true), buoyant).size(), 1u);    // FP16S
    EXPECT_EQ(ScalarTransportRules::warnings(0.01f, build_with(true, false, false), ScalarContext{ true, true, true }).size(), 1u); // moisture drift
    EXPECT_TRUE(ScalarTransportRules::warnings(1.0E-4f, build_with(true, true, true), ScalarContext{ true, true, true }).empty()); // finite volumes
}

} // namespace
