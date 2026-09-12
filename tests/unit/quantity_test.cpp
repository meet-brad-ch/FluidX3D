#include "setup/core/quantity.hpp"

#include <gtest/gtest.h>
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
