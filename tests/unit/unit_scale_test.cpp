#include "setup/core/unit_scale.hpp"

#include <gtest/gtest.h>

namespace {

// 2 m model = 100 cells, 10 m/s = lattice speed 0.1, water
UnitScale water_scale() {
    return UnitScale::from_reference(2.0_m, 100.0f, 10.0_mps, 0.1f, 1000.0_kgpm3);
}

TEST(UnitScale, BaseUnitsFollowFromTheReference) {
    const UnitScale scale = water_scale();
    EXPECT_FLOAT_EQ(scale.cell_size().si(), 0.02f);          // 2 m / 100 cells
    EXPECT_FLOAT_EQ(scale.time_step().si(), 0.0002f);        // 0.1/10 * 0.02 s
    EXPECT_FLOAT_EQ(scale.density().si(), 1000.0f);
}

TEST(UnitScale, ReferenceValuesMapBack) {
    const UnitScale scale = water_scale();
    EXPECT_FLOAT_EQ(scale.length(2.0_m), 100.0f);
    EXPECT_FLOAT_EQ(scale.velocity(10.0_mps), 0.1f);
    EXPECT_FLOAT_EQ(scale.si_length(100.0f).si(), 2.0f);
    EXPECT_FLOAT_EQ(scale.si_velocity(0.1f).si(), 10.0f);
}

// Regression for review A2: gravity was converted as a force per volume without the fluid density,
// which made it 1000x too weak in water. As an acceleration it does not depend on the density.
TEST(UnitScale, GravityDoesNotDependOnTheDensity) {
    const UnitScale water = water_scale();
    const UnitScale air = UnitScale::from_reference(2.0_m, 100.0f, 10.0_mps, 0.1f, 1.225_kgpm3);
    EXPECT_FLOAT_EQ(water.acceleration(9.81_mps2), air.acceleration(9.81_mps2));
    EXPECT_FLOAT_EQ(water.acceleration(9.81_mps2), 9.81f / 0.02f * 0.0002f * 0.0002f);
}

TEST(UnitScale, ViscosityAndSurfaceTension) {
    const UnitScale scale = water_scale();
    EXPECT_FLOAT_EQ(scale.viscosity(1.0e-3_m2ps), 1.0e-3f * 0.0002f / (0.02f * 0.02f));
    const float unit_kg = 1000.0f * 0.02f * 0.02f * 0.02f;
    EXPECT_FLOAT_EQ(scale.surface_tension(0.072_Npm), 0.072f * 0.0002f * 0.0002f / unit_kg);
    EXPECT_NEAR(scale.si_surface_tension(scale.surface_tension(0.072_Npm)).si(), 0.072f, 1e-7f);
}

TEST(UnitScale, TimeStepsRoundToTheNearestStep) {
    const UnitScale scale = water_scale(); // 0.0002 s per step
    EXPECT_EQ(scale.time_steps(1.0_s), 5000u);
    EXPECT_EQ(scale.time_steps(Duration::from_si(0.00029f)), 1u);
    EXPECT_EQ(scale.time_steps(Duration::from_si(0.00031f)), 2u);
    EXPECT_EQ(scale.time_steps(Duration::from_si(-1.0f)), 0u);
    EXPECT_FLOAT_EQ(scale.si_time(5000u).si(), 1.0f);
}

TEST(UnitScale, ForceBackToNewtons) {
    const UnitScale scale = water_scale();
    const float unit_kg = 1000.0f * 0.02f * 0.02f * 0.02f;
    EXPECT_FLOAT_EQ(scale.si_force(1.0f).si(), unit_kg * 0.02f / (0.0002f * 0.0002f));
}

TEST(UnitScale, PressureAndDensityRoundTrip) {
    const UnitScale scale = water_scale();
    const float unit_kg = 1000.0f * 0.02f * 0.02f * 0.02f;
    EXPECT_FLOAT_EQ(scale.si_pressure(1.0f).si(), unit_kg / (0.02f * 0.0002f * 0.0002f)); // Pa per lattice pressure unit
    EXPECT_NEAR(scale.pressure(scale.si_pressure(0.3f)), 0.3f, 1e-6f);
    EXPECT_FLOAT_EQ(scale.si_density(1.0f).si(), 1000.0f); // the lattice density 1 is the fluid's
}

} // namespace
