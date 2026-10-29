#include "setup/domain/level_grid.hpp"
#include "setup/core/setup_error.hpp"
#include "golden_capture.hpp"

#include <gtest/gtest.h>

namespace {

using namespace si::symbols;

// every level's height: the verified example's values, bit for bit
// Regression for defect 19: a column under half a cell has no fluid level: refused where its levels are counted
TEST(LevelGrid, LevelsForRefusesAColumnUnderHalfACell) {
    EXPECT_THROW(LevelGrid::levels_for(4.0 * m, 10.0 * m), SetupError);
    const std::uint32_t one_fluid_level = LevelGrid::levels_for(6.0 * m, 10.0 * m);
    EXPECT_EQ(one_fluid_level, 3u);
}

TEST(LevelGrid, HeightsAreTheVerifiedExamples) {
    for(const char* name : { "quick", "sp2011" }) {
        const GoldenCapture g(name);
        const std::uint32_t nz = (std::uint32_t)g.setting("Nz");
        const si::Length cell = g.setting("dx") * m;
        const LevelGrid grid(nz, cell);
        const std::uint32_t levels = grid.levels();
        const std::size_t expected_levels = g.levels();
        ASSERT_EQ(levels, expected_levels) << name;
        for(std::uint32_t z = 0u; z < grid.levels(); z++) {
            const double height = grid.height(z).in(m);
            const double expected_height = g.level(z, "height_m");
            EXPECT_EQ(height, expected_height) << name << " z " << z;
        }
    }
}

// sp2011's lid: 2048 m at 10 m cells is 207 levels (the example's lround(2048/dx) + 2)
TEST(LevelGrid, LevelsForAHeight) {
    const GoldenCapture g("sp2011");
    const std::uint32_t sp2011_levels = LevelGrid::levels_for(2048.0 * m, 10.0 * m);
    const std::uint32_t expected_sp2011_levels = (std::uint32_t)g.setting("Nz");
    EXPECT_EQ(sp2011_levels, expected_sp2011_levels);
    const std::uint32_t moist_levels = LevelGrid::levels_for(3760.0 * m, 10.0 * m);
    EXPECT_EQ(moist_levels, 378u); // 2 zi of the moist sounding case
    EXPECT_THROW(LevelGrid::levels_for(0.0 * m, 10.0 * m), SetupError);
}

TEST(LevelGrid, FluidLevelsAndTheLid) {
    const LevelGrid grid(207u, 10.0 * m);
    const std::uint32_t first_fluid = grid.first_fluid();
    EXPECT_EQ(first_fluid, 1u);
    const std::uint32_t last_fluid = grid.last_fluid();
    EXPECT_EQ(last_fluid, 205u);
    const std::uint32_t fluid_levels = grid.fluid_levels();
    EXPECT_EQ(fluid_levels, 205u);
    const double first_height = grid.height(1u).in(m);
    EXPECT_EQ(first_height, 5.0);         // half a cell above the floor's top
    const double lid_height = grid.lid_height().in(m);
    EXPECT_EQ(lid_height, 2055.0);    // the top cell's centre (the example's lid for the sponge)
    const std::uint32_t at_5 = grid.nearest_level(5.0 * m);
    EXPECT_EQ(at_5, 1u);
    const std::uint32_t at_974 = grid.nearest_level(974.0 * m);
    EXPECT_EQ(at_974, 98u);   // the centre 975 m
    const std::uint32_t below_floor = grid.nearest_level(-50.0 * m);
    EXPECT_EQ(below_floor, 1u);    // clamped
    const std::uint32_t above_lid = grid.nearest_level(9000.0 * m);
    EXPECT_EQ(above_lid, 205u); // clamped
}

TEST(LevelGrid, RefusesNoCellNoWallNoFluid) {
    EXPECT_THROW(LevelGrid(10u, 0.0 * m), SetupError);
    EXPECT_THROW(LevelGrid(10u, 10.0 * m, 0u, 1u), SetupError);
    EXPECT_THROW(LevelGrid(2u, 10.0 * m), SetupError);
}

} // namespace
