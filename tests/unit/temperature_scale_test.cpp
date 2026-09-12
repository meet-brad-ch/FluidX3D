#include "setup/core/temperature_scale.hpp"
#include "setup/core/fluids.hpp"

#include <gtest/gtest.h>

// rayleigh_benard's walls: 330 K below, 300 K above
TEST(TemperatureScale, TheWallsBecomeOneAndAHalfAndOneHalf) {
    const TemperatureScale scale = TemperatureScale::between(300.0_K, 330.0_K);
    EXPECT_FLOAT_EQ(scale.lattice(330.0_K), 1.5f);
    EXPECT_FLOAT_EQ(scale.lattice(300.0_K), 0.5f);
    EXPECT_FLOAT_EQ(scale.lattice(315.0_K), 1.0f); // the reference: the core's average temperature
    EXPECT_EQ(scale.reference(), 315.0_K);
    EXPECT_EQ(scale.difference(), 30.0_K);
}

// Regression for review A6: the lattice expansion was beta*300 K for any temperature range, 10x too strong for 30 K.
TEST(TemperatureScale, LatticeBuoyancyEqualsThePhysicalBuoyancy) {
    const TemperatureScale scale = TemperatureScale::between(300.0_K, 330.0_K);
    const ThermalExpansion beta = Fluid::AIR.thermal_expansion;
    const float g_lbm = 0.0008f; // any lattice gravity: both sides scale with it
    const Temperature T = 327.0_K;
    const float lattice_buoyancy = g_lbm * scale.expansion(beta) * (scale.lattice(T) - 1.0f);
    const float physical_buoyancy = g_lbm * (beta * (T - scale.reference())); // g*beta*(T - T_ref) in lattice units
    EXPECT_NEAR(lattice_buoyancy, physical_buoyancy, 1e-6f * g_lbm);
    EXPECT_FLOAT_EQ(scale.expansion(beta), 0.00341f * 30.0f);
}
