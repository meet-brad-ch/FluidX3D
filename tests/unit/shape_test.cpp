#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/shape.hpp"
#include "shapes.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

using namespace si::symbols;

const uint3 grid(20u, 30u, 10u); // cells of 0.1 m: a 2 m x 3 m x 1 m domain
const si::Length cell = static_cast<double>(0.1f) * si::symbols::m; // the core's float cell of 0.1 m, as the builders pass it

uint32_t count(const Shape::Cells& cells) {
    uint32_t n = 0u;
    for(uint32_t z = 0u; z < grid.z; z++) for(uint32_t y = 0u; y < grid.y; y++) for(uint32_t x = 0u; x < grid.x; x++) {
        if(cells.contains(x, y, z)) n++;
    }
    return n;
}

// The domain's center in metres is the core's lbm.center(), 0.5*N-0.5 in cells.
TEST(Shape, SphereAtTheDomainCenterIsTheCoresSphereAtCenter) {
    const Shape::Cells cells = Shape::sphere({ 1.0 * m, 1.5 * m, 0.5 * m }, 0.35 * m).in_cells(cell, grid);
    const float3 center(9.5f, 14.5f, 4.5f);
    for(uint32_t z = 0u; z < grid.z; z++) for(uint32_t y = 0u; y < grid.y; y++) for(uint32_t x = 0u; x < grid.x; x++) {
        ASSERT_EQ(cells.contains(x, y, z), sphere(x, y, z, center, 3.5f)) << x << " " << y << " " << z;
    }
}

TEST(Shape, BoxCoversWholeCells) {
    const Shape::Cells cells = Shape::box({ 0.1 * m, 0.0 * m, 0.0 * m }, { 0.34 * m, 3.0 * m, 0.2 * m }).in_cells(cell, grid);
    EXPECT_FALSE(cells.contains(0u, 0u, 0u));
    EXPECT_TRUE(cells.contains(1u, 0u, 0u));
    EXPECT_TRUE(cells.contains(2u, 29u, 1u)); // up to the domain's end
    EXPECT_FALSE(cells.contains(3u, 0u, 0u)); // 3.4 cells round to 3
    EXPECT_FALSE(cells.contains(1u, 0u, 2u));
    EXPECT_EQ(count(cells), 2u * 30u * 2u);
}

TEST(Shape, CylinderHasItsLengthAlongItsAxis) {
    const Shape::Cells cells = Shape::cylinder({ 1.0 * m, 1.5 * m, 0.5 * m }, Axis::Y, 0.2 * m, 1.0 * m).in_cells(cell, grid);
    EXPECT_TRUE(cells.contains(9u, 14u, 4u));
    EXPECT_TRUE(cells.contains(9u, 19u, 4u));  // 0.5 m (4.5 cells) along the axis from the center
    EXPECT_FALSE(cells.contains(9u, 20u, 4u));
    EXPECT_FALSE(cells.contains(12u, 14u, 4u)); // 2.5 cells off the axis
}

// The half-space is the side of the plane the normal points away from: below a horizontal plane at 0.5 m, the lower
// half of the 1 m tall grid; a tilted plane gives the core's plane() cells.
TEST(Shape, HalfSpaceIsBehindItsNormal) {
    EXPECT_EQ(count(Shape::half_space({ 0.0 * m, 0.0 * m, 0.5 * m }, si::Direction{ 0.0, 0.0, 1.0 }).in_cells(cell, grid)), grid.x * grid.y * 5u);
    const Shape::Cells beach = Shape::half_space({ 1.0 * m, 1.0 * m, 0.0 * m }, si::Direction{ 0.0, -1.0, 8.0 }).in_cells(cell, grid);
    const float3 point(9.5f, 9.5f, -0.5f); // the core's cell coordinates of the point
    for(uint32_t z = 0u; z < grid.z; z++) for(uint32_t y = 0u; y < grid.y; y++) for(uint32_t x = 0u; x < grid.x; x++) {
        ASSERT_EQ(beach.contains(x, y, z), plane(x, y, z, point, float3(0.0f, -1.0f, 8.0f))) << x << " " << y << " " << z;
    }
}

TEST(Shape, CombinationsAreSetOperations) {
    const Shape big = Shape::sphere({ 1.0 * m, 1.5 * m, 0.5 * m }, 0.45 * m);
    const Shape lower = Shape::box({ 0.0 * m, 0.0 * m, 0.0 * m }, { 2.0 * m, 3.0 * m, 0.5 * m });
    const uint32_t all = grid.x * grid.y * grid.z, in_big = count(big.in_cells(cell, grid));
    EXPECT_EQ(count((!big).in_cells(cell, grid)), all - in_big);
    const uint32_t both = count((big & lower).in_cells(cell, grid));
    EXPECT_GT(both, 0u);
    EXPECT_LT(both, in_big);
    EXPECT_EQ(count((big | lower).in_cells(cell, grid)), in_big + count(lower.in_cells(cell, grid)) - both);
}

TEST(Shape, SphereFillIsSmoothAtItsSurface) {
    const Shape::Cells cells = Shape::sphere({ 1.0 * m, 1.5 * m, 0.5 * m }, 0.3 * m).in_cells(cell, grid);
    EXPECT_FLOAT_EQ(cells.fill(9u, 14u, 4u), 1.0f);
    EXPECT_FLOAT_EQ(cells.fill(0u, 0u, 0u), 0.0f);
    const float surface = cells.fill(12u, 14u, 4u); // 2.5 cells from the center, the radius 3 cells: partly inside
    EXPECT_GT(surface, 0.0f);
    EXPECT_LT(surface, 1.0f);
    EXPECT_FLOAT_EQ((!Shape::sphere({ 1.0 * m, 1.5 * m, 0.5 * m }, 0.3 * m)).in_cells(cell, grid).fill(12u, 14u, 4u), 1.0f - surface);
}

TEST(Shape, TorusIsARingAboutItsAxis) {
    const Shape::Cells cells = Shape::torus({ 1.0 * m, 1.5 * m, 0.5 * m }, Axis::Z, 0.5 * m, 0.15 * m).in_cells(cell, grid);
    EXPECT_FALSE(cells.contains(9u, 14u, 4u)); // the hole
    EXPECT_TRUE(cells.contains(14u, 14u, 4u)); // on the ring, 5 cells from the center
    EXPECT_FALSE(cells.contains(14u, 14u, 7u)); // above it
}

/// One cell of a shape on the grid: its contains() bit and its fill() value.
struct CellState {
    bool inside{};  ///< whether the cell's centre is inside
    float share{};  ///< the cell's fill, 0 to 1
};

/// @brief Gives every cell of a shape on the grid, cell n = x + (y + z Ny) Nx.
/// @param cells the shape on the grid
/// @return one state per cell
std::vector<CellState> states(const Shape::Cells& cells) {
    std::vector<CellState> result{};
    result.reserve(static_cast<std::size_t>(grid.x) * grid.y * grid.z);
    for(uint32_t z = 0u; z < grid.z; z++) {
        for(uint32_t y = 0u; y < grid.y; y++) {
            for(uint32_t x = 0u; x < grid.x; x++) {
                const CellState state{ .inside = cells.contains(x, y, z), .share = cells.fill(x, y, z) };
                result.push_back(state);
            }
        }
    }
    return result;
}

/// @brief Gives every cell of one primitive shape on the grid.
/// @param shape a shape of one primitive
/// @return one state per cell
std::vector<CellState> primitive_states(const Shape& shape) {
    const Shape::Cells cells{ shape.in_cells(cell, grid) };
    const std::vector<CellState> result{ states(cells) };
    return result;
}

/// @brief Checks a combination cell by cell against its reference, and gives its count of inside cells.
/// @param actual the combination's cells (the flat postfix form)
/// @param expected the reference computed cell by cell from the primitives
/// @return the count of inside cells of the combination
uint32_t expect_same_cells(const std::vector<CellState>& actual, const std::vector<CellState>& expected) {
    uint32_t inside{ 0u };
    for(std::size_t n = 0u; n < actual.size(); n++) {
        EXPECT_EQ(actual[n].inside, expected[n].inside) << "cell " << n;
        EXPECT_EQ(actual[n].share, expected[n].share) << "cell " << n; // exact: the same operations on the same values
        if(actual[n].inside) {
            inside++;
        }
    }
    return inside;
}

/// @brief Gives sphere i of the deep combinations: spread over the domain, radii from 0.12 m to 0.31 m.
Shape sphere_number(int i) {
    const double fraction{ static_cast<double>(i) / 40.0 };
    const si::Position centre{ (0.1 + 1.8 * fraction) * m, (0.2 + 2.6 * fraction * fraction) * m, (0.15 + 0.7 * fraction) * m };
    const si::Length radius{ (0.12 + 0.005 * static_cast<double>(i % 39)) * m };
    const Shape sphere{ Shape::sphere(centre, radius) };
    return sphere;
}

/// @brief Gives slab i of the mixed combination: a 0.04 m thick box across the domain, 0.05 i metres along y.
Shape slab_number(int i) {
    const double low{ 0.05 * static_cast<double>(i) };
    const Shape slab{ Shape::box({ 0.0 * m, low * m, 0.0 * m }, { 2.0 * m, (low + 0.04) * m, 1.0 * m }) };
    return slab;
}

/// @brief Gives the box of the nested combination: the lower 0.4 m of the domain's first metre along x.
Shape nested_box() {
    const Shape box{ Shape::box({ 0.0 * m, 0.0 * m, 0.0 * m }, { 1.0 * m, 3.0 * m, 0.4 * m }) };
    return box;
}

/// @brief Gives the right-deep union s39 | (s38 | (... | s0)) cell by cell: either inside, the larger fill.
/// @return one state per cell
std::vector<CellState> right_deep_reference() {
    std::vector<CellState> result{ primitive_states(sphere_number(0)) };
    for(int i = 1; i < 40; i++) {
        const std::vector<CellState> sphere{ primitive_states(sphere_number(i)) };
        for(std::size_t n = 0u; n < sphere.size(); n++) {
            const CellState below{ result[n] };
            result[n] = CellState{ .inside = sphere[n].inside || below.inside, .share = std::fmax(sphere[n].share, below.share) };
        }
    }
    return result;
}

/// @brief Gives the mixed combination ((s0 & !b1) | s1) & !b2 ... cell by cell: AND both and the smaller fill, NOT the
/// outside and 1 - fill, OR either and the larger fill.
/// @return one state per cell
std::vector<CellState> mixed_reference() {
    std::vector<CellState> result{ primitive_states(sphere_number(0)) };
    for(int i = 1; i < 30; i++) {
        const std::vector<CellState> slab{ primitive_states(slab_number(i)) };
        const std::vector<CellState> sphere{ primitive_states(sphere_number(i)) };
        for(std::size_t n = 0u; n < sphere.size(); n++) {
            const CellState level{ result[n] };
            const bool kept_inside{ level.inside && !slab[n].inside };
            const float kept_share{ std::fmin(level.share, 1.0f - slab[n].share) };
            result[n] = CellState{ .inside = kept_inside || sphere[n].inside, .share = std::fmax(kept_share, sphere[n].share) };
        }
    }
    return result;
}

/// @brief Gives the nested combination !(!(!(s5 & box))) cell by cell: the three NOTs one after the other, as the
/// postfix form applies them.
/// @return one state per cell
std::vector<CellState> nested_reference() {
    const std::vector<CellState> sphere{ primitive_states(sphere_number(5)) };
    const std::vector<CellState> box{ primitive_states(nested_box()) };
    std::vector<CellState> result(sphere.size());
    for(std::size_t n = 0u; n < sphere.size(); n++) {
        const bool both{ sphere[n].inside && box[n].inside };
        const float both_share{ std::fmin(sphere[n].share, box[n].share) };
        const float once{ 1.0f - both_share };
        const float twice{ 1.0f - once };
        const float three_times{ 1.0f - twice };
        result[n] = CellState{ .inside = !both, .share = three_times };
    }
    return result;
}

// Regression for the flat postfix form (simple-code review, misc-no-recursion): deep combinations give, in every cell,
// the contains() bit and the fill() value of the tree form's rules (AND: both and the smaller fill, OR: either and the
// larger fill, NOT: the outside and 1 - fill), applied here cell by cell to the primitives' own cells. The reference
// runs on the same compiler as the code under test: the primitives' fills come from the core's fast-math functions
// (sphere_plic: cbrt, asin, sin), whose last bit differs between MSVC and GCC. The inside counts are those of the tree form
// that the flat form replaced, and they are the same on both compilers.
TEST(Shape, DeepCombinationsKeepTheirCellsAndFills) {
    Shape right_deep{ sphere_number(0) }; // s39 | (s38 | (... | s0)): 40 levels, the deepest case of a stack
    for(int i = 1; i < 40; i++) {
        right_deep = sphere_number(i) | right_deep;
    }
    Shape mixed{ sphere_number(0) }; // ((s0 & !b1) | s1) & !b2 ...: each level an AND, a NOT and an OR
    for(int i = 1; i < 30; i++) {
        mixed = (mixed & !slab_number(i)) | sphere_number(i);
    }
    const Shape nested{ !(!(!(sphere_number(5) & nested_box()))) };

    const std::vector<CellState> right_deep_expected{ right_deep_reference() };
    const std::vector<CellState> mixed_expected{ mixed_reference() };
    const std::vector<CellState> nested_expected{ nested_reference() };
    const std::vector<CellState> right_deep_cells{ states(right_deep.in_cells(cell, grid)) };
    const std::vector<CellState> mixed_cells{ states(mixed.in_cells(cell, grid)) };
    const std::vector<CellState> nested_cells{ states(nested.in_cells(cell, grid)) };
    EXPECT_EQ(expect_same_cells(right_deep_cells, right_deep_expected), 584u);
    EXPECT_EQ(expect_same_cells(mixed_cells, mixed_expected), 317u);
    EXPECT_EQ(expect_same_cells(nested_cells, nested_expected), 5987u); // all 6000 cells but the 13 in both the sphere and the box
}

} // namespace
