#include "setup/core/quantity.hpp"
#include "setup/core/fluids.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <type_traits>

namespace {

// Compile-time checks. The negative ones need named concepts: a requires-expression outside a template
// makes an invalid expression a hard error instead of false.
template<typename A, typename B> concept Addable = requires(A a, B b) { a + b; };
template<typename A, typename B> concept Comparable = requires(A a, B b) { a < b; };
template<typename To, typename From> concept Constructible = requires(From f) { To{f}; };

static_assert(std::is_same_v<decltype(1.0_m / 2.0_s), Speed>);
static_assert(std::is_same_v<decltype(1.0_mps / 1.0_s), Acceleration>);
static_assert(std::is_same_v<decltype(1.0_m * 1.0_m), Area>);
static_assert(std::is_same_v<decltype(1.0_kgpm3 * 1.0_mps2 * 1.0_m), Pressure>);
static_assert(std::is_same_v<decltype(1.0_m / 1.0_m), float>);
static_assert(std::is_same_v<decltype(1.0f / 1.0_s), Frequency>);
static_assert(std::is_same_v<decltype(1.0_m3ps / (1.0_m * 1.0_m)), Speed>); // flow rate through an area
static_assert(std::is_same_v<decltype(sqrt(1.0_mps2 * 1.0_m)), Speed>);    // shallow-water wave speed sqrt(g*h)

template<typename Q> concept HasSqrt = requires(Q q) { sqrt(q); };
static_assert(HasSqrt<Area>);
static_assert(!HasSqrt<Length>); // m^(1/2) is not a quantity

static_assert(Addable<Length, Length>);
static_assert(!Addable<Length, Speed>);
static_assert(!Addable<Angle, float>);
static_assert(Comparable<Length, Length>);
static_assert(!Comparable<Length, Duration>);
static_assert(!Constructible<Speed, Length>);
static_assert(!Constructible<Length, float>);
static_assert(!std::is_convertible_v<float, Length>);

static_assert(PhysicalQuantity<Speed>);
static_assert(!PhysicalQuantity<float>);

TEST(Quantity, LiteralsAreStoredInSiUnits) {
    EXPECT_FLOAT_EQ((2.0_m).si(), 2.0f);
    EXPECT_FLOAT_EQ((2_m).si(), 2.0f);
    EXPECT_FLOAT_EQ((150.0_cm).si(), 1.5f);
    EXPECT_FLOAT_EQ((25.0_mm).si(), 0.025f);
    EXPECT_FLOAT_EQ((1.5_km).si(), 1500.0f);
    EXPECT_FLOAT_EQ((2.0_min).si(), 120.0f);
    EXPECT_FLOAT_EQ((36.0_kmh).si(), 10.0f);
    EXPECT_FLOAT_EQ((9.81_mps2).si(), 9.81f);
    EXPECT_FLOAT_EQ((998.2_kgpm3).si(), 998.2f);
    EXPECT_FLOAT_EQ((0.072_Npm).si(), 0.072f);
    EXPECT_FLOAT_EQ((0.25_m3ps).si(), 0.25f);
}

TEST(Quantity, CelsiusIsAnAbsoluteTemperature) {
    EXPECT_FLOAT_EQ((20.0_C).si(), 293.15f);
    EXPECT_FLOAT_EQ((300.0_K).si(), 300.0f);
}

TEST(Quantity, ArithmeticKeepsTheDimension) {
    EXPECT_EQ(1.0_m + 50.0_cm, 1.5_m);
    EXPECT_EQ(2.0_m - 0.5_m, 1.5_m);
    EXPECT_EQ(2.0f * 1.0_m, 2.0_m);
    EXPECT_EQ(3.0_m / 2.0f, 1.5_m);
    EXPECT_EQ(-(1.0_m), Length::from_si(-1.0f));
    EXPECT_LT(1.0_m, 2.0_m);
}

TEST(Quantity, ArithmeticCombinesDimensions) {
    const Speed u = 10.0_m / 2.0_s;
    EXPECT_FLOAT_EQ(u.si(), 5.0f);
    const Pressure p = 1000.0_kgpm3 * 9.81_mps2 * 2.0_m; // hydrostatic pressure of 2 m water
    EXPECT_FLOAT_EQ(p.si(), 19620.0f);
    const float reynolds = 2.0_m * 1.0_mps / 1.48e-5_m2ps;
    EXPECT_NEAR(reynolds, 135135.1f, 0.1f);
}

TEST(Quantity, SquareRootHalvesTheDimension) {
    EXPECT_FLOAT_EQ(sqrt(4.0_m * 1.0_m).si(), 2.0f);
    EXPECT_EQ(sqrt(2.0f * 9.81_mps2 * 0.75_m).si(), std::sqrt(2.0f * 9.81f * 0.75f)); // the same float operations
}

TEST(Quantity, KilometresPerHourAreDividedBy3point6) {
    EXPECT_EQ((300.0_kmh).si(), 300.0f / 3.6f);
    EXPECT_EQ((226_kmh).si(), 226.0f / 3.6f);
}

TEST(Fluid, PropertiesAreQuantities) {
    const float reynolds = 2.0_m * 10.0_mps / Fluid::AIR.kinematic_viscosity; // dimensionless
    EXPECT_FLOAT_EQ(reynolds, 2.0f * 10.0f / 1.48e-5f);
    EXPECT_EQ(Fluid::WATER.density, Density::from_si(998.2f));
    static_assert(std::is_same_v<decltype(sqrt(9.81_mps2 * Fluid::AIR.thermal_expansion * 30.0_K * 1.0_m)), Speed>); // buoyancy speed
}

TEST(Angle, ConvertsBetweenDegreesAndRadians) {
    EXPECT_FLOAT_EQ((90.0_deg).rad(), 1.5707964f);
    EXPECT_NEAR((1.0_rad).deg(), 57.29578f, 1e-4f);
    EXPECT_NEAR((30_deg + 15.0_deg).deg(), 45.0f, 1e-4f);
}

static_assert(!Constructible<ModelLengths, float>);
static_assert(!Constructible<MemorySize, unsigned int>);
static_assert(!Addable<ModelLengths, Length>);

TEST(ModelLengths, AreAPlainNumberOfModelLengths) {
    EXPECT_FLOAT_EQ((0.5_lengths).value(), 0.5f);
    EXPECT_FLOAT_EQ((2_lengths).value(), 2.0f);
    EXPECT_LT(0.5_lengths, 1.0_lengths);
}

TEST(MemorySize, GigabytesAre1024Megabytes) {
    EXPECT_EQ((2000_mb).mb(), 2000u);
    EXPECT_EQ((20_gb).mb(), 20480u);
    EXPECT_EQ(1_gb, 1024_mb);
}

static_assert(std::is_same_v<decltype(180.0_deg / 1.0_s), AngularSpeed>);
static_assert(!Constructible<LatticeMach, Speed>);
static_assert(!std::is_convertible_v<float, LatticeMach>);

TEST(Duration, MillisecondsAndMicroseconds) {
    EXPECT_FLOAT_EQ((2.2_ms).si(), 0.0022f);
    EXPECT_FLOAT_EQ((90_us).si(), 9.0e-5f);
}

TEST(AngularSpeed, IsAnAnglePerDuration) {
    const AngularSpeed half_turn_per_second = 180_deg / 1.0_s;
    EXPECT_FLOAT_EQ(half_turn_per_second.deg_per_s(), 180.0f);
    EXPECT_FLOAT_EQ(half_turn_per_second.rad_per_s(), 3.14159265f);
    EXPECT_FLOAT_EQ((90_deg / 0.5_s).deg_per_s(), 180.0f);
}

TEST(LatticeMach, IsTheLatticeSpeedOverTheLatticeSpeedOfSound) {
    EXPECT_EQ(LatticeMach().lattice_speed(), 0.1f); // the default: 0.1 cells per time step, exactly
    EXPECT_NEAR(LatticeMach().value(), 0.17320508f, 1e-6f);
    EXPECT_EQ(LatticeMach(1.0f).lattice_speed(), 0.57735027f); // the lattice speed of sound
    EXPECT_NEAR(LatticeMach(0.13f).lattice_speed(), 0.075055f, 1e-6f);
    EXPECT_NEAR(LatticeMach(0.075f * sqrtf(3.0f)).lattice_speed(), 0.075f, 1e-7f); // a lattice speed round trips
}

TEST(Quantity, SpecsUseDesignatedInitializers) {
    struct CameraSpec {
        Angle yaw{};
        Angle pitch{};
        Length distance = 10.0_m;
    };
    const CameraSpec camera{ .yaw = 30.0_deg, .distance = 8.0_m };
    EXPECT_NEAR(camera.yaw.deg(), 30.0f, 1e-4f);
    EXPECT_EQ(camera.pitch, 0.0_deg);
    EXPECT_EQ(camera.distance, 8.0_m);
}

} // namespace
