#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/shape.hpp"
#include "shapes.hpp"

namespace {

const uint3 grid(20u, 30u, 10u); // cells of 0.1 m: a 2 m x 3 m x 1 m domain
constexpr float cell = 0.1f;

uint32_t count(const Shape::Cells& cells) {
    uint32_t n = 0u;
    for(uint32_t z = 0u; z < grid.z; z++) for(uint32_t y = 0u; y < grid.y; y++) for(uint32_t x = 0u; x < grid.x; x++) {
        if(cells.contains(x, y, z)) n++;
    }
    return n;
}

// The domain's center in metres is the core's lbm.center(), 0.5*N-0.5 in cells.
TEST(Shape, SphereAtTheDomainCenterIsTheCoresSphereAtCenter) {
    const Shape::Cells cells = Shape::sphere({ 1_m, 1.5_m, 0.5_m }, 0.35_m).in_cells(cell, grid);
    const float3 center(9.5f, 14.5f, 4.5f);
    for(uint32_t z = 0u; z < grid.z; z++) for(uint32_t y = 0u; y < grid.y; y++) for(uint32_t x = 0u; x < grid.x; x++) {
        ASSERT_EQ(cells.contains(x, y, z), sphere(x, y, z, center, 3.5f)) << x << " " << y << " " << z;
    }
}

TEST(Shape, BoxCoversWholeCells) {
    const Shape::Cells cells = Shape::box({ 0.1_m, 0_m, 0_m }, { 0.34_m, 3_m, 0.2_m }).in_cells(cell, grid);
    EXPECT_FALSE(cells.contains(0u, 0u, 0u));
    EXPECT_TRUE(cells.contains(1u, 0u, 0u));
    EXPECT_TRUE(cells.contains(2u, 29u, 1u)); // up to the domain's end
    EXPECT_FALSE(cells.contains(3u, 0u, 0u)); // 3.4 cells round to 3
    EXPECT_FALSE(cells.contains(1u, 0u, 2u));
    EXPECT_EQ(count(cells), 2u * 30u * 2u);
}

TEST(Shape, CylinderHasItsLengthAlongItsAxis) {
    const Shape::Cells cells = Shape::cylinder({ 1_m, 1.5_m, 0.5_m }, Axis::Y, 0.2_m, 1_m).in_cells(cell, grid);
    EXPECT_TRUE(cells.contains(9u, 14u, 4u));
    EXPECT_TRUE(cells.contains(9u, 19u, 4u));  // 0.5 m (4.5 cells) along the axis from the center
    EXPECT_FALSE(cells.contains(9u, 20u, 4u));
    EXPECT_FALSE(cells.contains(12u, 14u, 4u)); // 2.5 cells off the axis
}

TEST(Shape, CombinationsAreSetOperations) {
    const Shape big = Shape::sphere({ 1_m, 1.5_m, 0.5_m }, 0.45_m);
    const Shape lower = Shape::box({ 0_m, 0_m, 0_m }, { 2_m, 3_m, 0.5_m });
    const uint32_t all = grid.x * grid.y * grid.z, in_big = count(big.in_cells(cell, grid));
    EXPECT_EQ(count((!big).in_cells(cell, grid)), all - in_big);
    const uint32_t both = count((big & lower).in_cells(cell, grid));
    EXPECT_GT(both, 0u);
    EXPECT_LT(both, in_big);
    EXPECT_EQ(count((big | lower).in_cells(cell, grid)), in_big + count(lower.in_cells(cell, grid)) - both);
}

TEST(Shape, SphereFillIsSmoothAtItsSurface) {
    const Shape::Cells cells = Shape::sphere({ 1_m, 1.5_m, 0.5_m }, 0.3_m).in_cells(cell, grid);
    EXPECT_FLOAT_EQ(cells.fill(9u, 14u, 4u), 1.0f);
    EXPECT_FLOAT_EQ(cells.fill(0u, 0u, 0u), 0.0f);
    const float surface = cells.fill(12u, 14u, 4u); // 2.5 cells from the center, the radius 3 cells: partly inside
    EXPECT_GT(surface, 0.0f);
    EXPECT_LT(surface, 1.0f);
    EXPECT_FLOAT_EQ((!Shape::sphere({ 1_m, 1.5_m, 0.5_m }, 0.3_m)).in_cells(cell, grid).fill(12u, 14u, 4u), 1.0f - surface);
}

TEST(Shape, TorusIsARingAboutItsAxis) {
    const Shape::Cells cells = Shape::torus({ 1_m, 1.5_m, 0.5_m }, Axis::Z, 0.5_m, 0.15_m).in_cells(cell, grid);
    EXPECT_FALSE(cells.contains(9u, 14u, 4u)); // the hole
    EXPECT_TRUE(cells.contains(14u, 14u, 4u)); // on the ring, 5 cells from the center
    EXPECT_FALSE(cells.contains(14u, 14u, 7u)); // above it
}

} // namespace
