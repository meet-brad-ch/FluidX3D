#include "setup/core/boundary_utils.hpp"
#include "setup/core/setup_error.hpp"

#include <gtest/gtest.h>

namespace {

/// @brief The box of the tests: 4 x 5 x 6 cells.
/// @return its cells along x, y and z
uint3 box() {
    const uint3 cells{ 4u, 5u, 6u };
    return cells;
}

} // namespace

// a face's own layer: x = 0 for X_MIN, z = Nz - 1 for Z_MAX
TEST(FaceLayer, TheOwnLayerOfAFace) {
    const FaceLayer x_min(Face::X_MIN, box());
    const FaceLayer z_max(Face::Z_MAX, box());
    EXPECT_TRUE(x_min.contains(0u, 2u, 3u));
    EXPECT_FALSE(x_min.contains(1u, 2u, 3u));
    EXPECT_TRUE(z_max.contains(1u, 2u, 5u));
    EXPECT_FALSE(z_max.contains(1u, 2u, 4u));
}

// an offset moves the layer inward: one cell from Y_MAX is y = Ny - 2, one cell from Y_MIN is y = 1
TEST(FaceLayer, AnOffsetMovesTheLayerInward) {
    const FaceLayer y_max(Face::Y_MAX, box(), 1u);
    const FaceLayer y_min(Face::Y_MIN, box(), 1u);
    EXPECT_TRUE(y_max.contains(0u, 3u, 0u));
    EXPECT_FALSE(y_max.contains(0u, 4u, 0u));
    EXPECT_TRUE(y_min.contains(0u, 1u, 0u));
}

// the largest offset is the opposite face's layer: Nx - 1 from X_MAX is x = 0
TEST(FaceLayer, TheLargestOffsetIsTheOppositeLayer) {
    const FaceLayer x_max(Face::X_MAX, box(), 3u);
    EXPECT_TRUE(x_max.contains(0u, 0u, 0u));
}

// an offset of the cells along the axis or more is outside the box: refused, not wrapped around
TEST(FaceLayer, AnOffsetOutsideTheBoxIsRefused) {
    EXPECT_THROW(FaceLayer(Face::X_MAX, box(), 4u), SetupError);
    EXPECT_THROW(FaceLayer(Face::X_MIN, box(), 4u), SetupError);
    EXPECT_THROW(FaceLayer(Face::Z_MAX, box(), 7u), SetupError);
    const uint3 flat{ 4u, 5u, 1u };
    EXPECT_THROW(FaceLayer(Face::Z_MAX, flat, 1u), SetupError);
    EXPECT_NO_THROW(FaceLayer(Face::Z_MAX, flat, 0u));
}
