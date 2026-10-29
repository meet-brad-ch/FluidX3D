#include "setup/atmosphere/heated_patch.hpp"
#include "setup/core/setup_error.hpp"
#include "golden_capture.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <utility>
#include <vector>

namespace {

using namespace si::symbols;

/// The floor cells of a patch: x and y, y then x (the example's visiting order).
using FloorCells = std::vector<std::pair<std::uint32_t, std::uint32_t>>;

/// @param patch the patch
/// @param nx the cells along x
/// @param ny the cells along y
/// @param cell the cell size
/// @return the floor cells under the patch, y then x (the example's visiting order)
FloorCells footprint(const HeatedPatch& patch, std::uint32_t nx, std::uint32_t ny, si::Length cell) {
    FloorCells cells{};
    for(std::uint32_t y = 0u; y < ny; y++) {
        for(std::uint32_t x = 0u; x < nx; x++) {
            if(patch.covers(x, y, nx, ny, cell)) {
                cells.push_back({ x, y });
            }
        }
    }
    return cells;
}

/// @param g a capture
/// @return its patch.csv cells
FloorCells captured(const GoldenCapture& g) {
    FloorCells cells{};
    const std::string file = g.path("patch.csv");
    std::ifstream csv(file);
    std::string line{};
    std::getline(csv, line); // x,y
    while(std::getline(csv, line)) {
        const std::size_t c = line.find(',');
        if(c == std::string::npos) {
            continue;
        }
        const std::uint32_t x = (std::uint32_t)std::stoul(line.substr(0u, c));
        const std::uint32_t y = (std::uint32_t)std::stoul(line.substr(c + 1u));
        cells.push_back({ x, y });
    }
    return cells;
}

/// @param g a capture
/// @return its patch, from its settings
HeatedPatch patch_of(const GoldenCapture& g) {
    const si::Length diameter = g.setting("patch") * m;
    const si::Length north = g.setting("patch_north") * m;
    const bool surface = g.setting("patch_surface") != 0.0;
    const PatchMode mode = surface ? PatchMode::SURFACE_LAYER : PatchMode::HEATED_LAYERS;
    const std::uint32_t layers = (std::uint32_t)g.setting("patch_layers");
    const HeatedPatch patch = HeatedPatch::disc(diameter, north, mode, layers);
    return patch;
}

/// @param g a capture
/// @return the floor cells under its patch on its grid
FloorCells footprint_of(const GoldenCapture& g) {
    const HeatedPatch patch = patch_of(g);
    const std::uint32_t nx = (std::uint32_t)g.setting("Nx");
    const std::uint32_t ny = (std::uint32_t)g.setting("Ny");
    const si::Length cell = g.setting("dx") * m;
    const FloorCells cells = footprint(patch, nx, ny, cell);
    return cells;
}

// the cells of a 400 m disc on 10 m cells against pi r^2 (1256.6 cells): the boundary's cells decide within about one
// perimeter's ring, 2 pi r = 126 cells; the count is symmetric in the four quadrants
TEST(HeatedPatch, CellsAgainstTheDiscArea) {
    const HeatedPatch patch = HeatedPatch::disc(400.0 * m, 0.0 * m, PatchMode::SURFACE_LAYER);
    const FloorCells cells = footprint(patch, 128u, 128u, 10.0 * m);
    const double area = 3.14159265358979 * 20.0 * 20.0;
    const double count = (double)cells.size();
    const double difference = std::fabs(count - area);
    const double limit = 2.0 * 3.14159265358979 * 20.0 * 0.5;
    EXPECT_LT(difference, limit);
    std::size_t quadrant[4]{};
    for(const auto& [x, y] : cells) {
        const unsigned east = x >= 64u ? 1u : 0u;
        const unsigned north = y >= 64u ? 2u : 0u;
        const unsigned index = east + north;
        quadrant[index]++;
    }
    EXPECT_EQ(quadrant[0], quadrant[1]);
    EXPECT_EQ(quadrant[0], quadrant[2]);
    EXPECT_EQ(quadrant[0], quadrant[3]);
}

// the same patch in metres at two cell sizes: the centre 2000 m north of the middle at 10 m and at 5 m cells (the
// example's 200 dx is 2000 m only at 10 m)
TEST(HeatedPatch, NorthIsInMetres) {
    const HeatedPatch patch = HeatedPatch::disc(400.0 * m, 2000.0 * m, PatchMode::SURFACE_LAYER);
    const si::Length box = 6400.0 * m;
    for(const double dx : { 10.0, 5.0 }) {
        const si::Length cell = dx * m;
        const double cells_across = au::as_raw_number(box / cell);
        const long rounded = std::lround(cells_across);
        const std::uint32_t n = (std::uint32_t)rounded;
        double sum_y = 0.0;
        const FloorCells cells = footprint(patch, n, n, cell);
        ASSERT_FALSE(cells.empty()) << dx;
        for(const auto& c : cells) {
            const double centre = (double)c.second + 0.5;
            const double metres = centre * dx;
            sum_y += metres;
        }
        const double mean_y = sum_y / (double)cells.size();
        const double north = mean_y - 3200.0;
        const double tolerance = 0.5 * dx;
        EXPECT_NEAR(north, 2000.0, tolerance) << dx; // the centroid's metres north of the middle
    }
}

TEST(HeatedPatch, ModesAndFluxFactor) {
    const HeatedPatch surface = HeatedPatch::disc(400.0 * m, 0.0 * m, PatchMode::SURFACE_LAYER, 7u);
    const std::uint32_t surface_layers = surface.layers();
    EXPECT_EQ(surface_layers, 1u);
    const std::uint32_t surface_factor = surface.flux_factor();
    EXPECT_EQ(surface_factor, 4u); // the core's fixed factor, whatever layers says
    const HeatedPatch layers = HeatedPatch::disc(400.0 * m, 0.0 * m, PatchMode::HEATED_LAYERS, 4u);
    const std::uint32_t heated_layers = layers.layers();
    EXPECT_EQ(heated_layers, 4u);
    const std::uint32_t layers_factor = layers.flux_factor();
    EXPECT_EQ(layers_factor, 4u);
    const HeatedPatch none = HeatedPatch::none();
    EXPECT_FALSE(none.active());
    EXPECT_FALSE(none.covers(64u, 64u, 128u, 128u, 10.0 * m));
    EXPECT_THROW(HeatedPatch::none().mode(), SetupError);
    const std::string described = layers.describe();
    const std::size_t at = described.find("4 heated layers");
    EXPECT_NE(at, std::string::npos) << described;
}

TEST(HeatedPatch, RefusesBadValues) {
    EXPECT_THROW(HeatedPatch::disc(0.0 * m, 0.0 * m, PatchMode::SURFACE_LAYER), SetupError);
    EXPECT_THROW(HeatedPatch::disc(400.0 * m, 0.0 * m, PatchMode::HEATED_LAYERS, 0u), SetupError);
}

// the verified examples' patch cells, the same cells in the same order (quick: 4 heated layers, quick surface)
TEST(HeatedPatch, CellsAreTheVerifiedExamples) {
    for(const char* name : { "quick", "quick_surface" }) {
        const GoldenCapture g(name);
        const FloorCells cells = footprint_of(g);
        const double count = (double)cells.size();
        const double expected_count = g.setting("patch_cells");
        EXPECT_EQ(count, expected_count) << name;
        const FloorCells expected = captured(g);
        EXPECT_EQ(cells, expected) << name;
    }
}

// the extra cases (FLUIDX3D_EXTRA_CBL, FLUIDX3D_EXTRA_CBL_WIND: the patch upwind, north of the middle)
TEST(HeatedPatch, PrivateCasesAreTheirCaptures) {
    int cases = 0;
    for(const char* variable : { "FLUIDX3D_EXTRA_CBL", "FLUIDX3D_EXTRA_CBL_WIND" }) {
        const std::optional<std::string> folder = GoldenCapture::environment(variable);
        if(!folder) {
            continue;
        }
        const GoldenCapture g = GoldenCapture::at(*folder);
        const FloorCells cells = footprint_of(g);
        const double count = (double)cells.size();
        const double expected_count = g.setting("patch_cells");
        EXPECT_EQ(count, expected_count) << variable;
        const FloorCells expected = captured(g);
        EXPECT_EQ(cells, expected) << variable;
        cases++;
    }
    if(cases == 0) {
        GTEST_SKIP() << "FLUIDX3D_EXTRA_CBL and FLUIDX3D_EXTRA_CBL_WIND are not set";
    }
}

} // namespace
