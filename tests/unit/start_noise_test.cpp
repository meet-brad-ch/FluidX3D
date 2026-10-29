#include "setup/atmosphere/start_noise.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/level_grid.hpp"
#include "golden_capture.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <random>
#include <vector>

namespace {

using namespace si::symbols;

/// The largest per-cell draw of a 0.1 K amplitude: the draws are floats in [-0.1f, 0.1f] (the example's distribution).
const double largest_draw_kelvin = (double)0.1f;

/// @param x a cell's index along x
/// @param y along y
/// @param h its height
/// @param cell the cell size
/// @return the cell's centre
CellCentre centre(std::uint32_t x, std::uint32_t y, si::Length h, si::Length cell) {
    constexpr double half = 0.5; // a cell's centre, in cells from its lower face
    const double x_cells = (double)x + half;
    const double y_cells = (double)y + half;
    const si::Length east = cell * x_cells;
    const si::Length north = cell * y_cells;
    const CellCentre c = CellCentre{ .east = east, .north = north, .height = h };
    return c;
}

/// @param h a height
/// @return a cell centre at that height (the per-cell draws do not depend on the place)
CellCentre at_height(si::Length h) {
    const CellCentre c = CellCentre{ .east = 5.0 * m, .north = 5.0 * m, .height = h };
    return c;
}

/// @brief The draws of a start on a box: the fluid cells visited z, then y, then x (the start builder's order).
struct BoxDraws {
    std::vector<si::TemperatureDifference> values{}; ///< the draws, in the order they were drawn
};

/// @param noise the noise
/// @param grid the column's levels
/// @param nx the cells along x
/// @param ny the cells along y
/// @return the draws of the box's fluid cells (the floor and the lid get none)
BoxDraws draw_box(const StartNoise& noise, const LevelGrid& grid, std::uint32_t nx, std::uint32_t ny) {
    BoxDraws box{};
    StartNoise::Sequence sequence = noise.sequence(grid.cell());
    for(std::uint32_t z = grid.first_fluid(); z <= grid.last_fluid(); z++) {
        for(std::uint32_t y = 0u; y < ny; y++) {
            for(std::uint32_t x = 0u; x < nx; x++) {
                const std::size_t before = sequence.draws();
                const CellCentre cell_centre = centre(x, y, grid.height(z), grid.cell());
                const si::TemperatureDifference t = sequence.at(cell_centre);
                if(sequence.draws() > before) { // a draw was used (a draw can be 0)
                    box.values.push_back(t);
                }
            }
        }
    }
    return box;
}

#ifdef _MSC_VER // only the MSVC draws test uses it (the captures come from MSVC's distribution)
/// @param values the draws (floats of the example's distribution)
/// @return their FNV-1a 64 hash over the float bytes (the capture's noise_hash.txt)
std::uint64_t fnv1a(const std::vector<si::TemperatureDifference>& values) {
    std::uint64_t h = 14695981039346656037ull;
    for(const si::TemperatureDifference& value : values) {
        const float v = (float)value.in(K); // exact: each draw is a float
        unsigned char b[4]{};
        std::memcpy(b, &v, 4u);
        for(const unsigned char c : b) {
            h ^= c;
            h *= 1099511628211ull;
        }
    }
    return h;
}
#endif // _MSC_VER

/// @brief Expects the statistics of a uniform distribution in [-0.1 K, 0.1 K]: mean 0 and variance a^2 / 3 within their
/// sampling errors (3 sigma: sigma_mean = a / sqrt(3 N), the variance's relative sigma sqrt(0.8 / N)).
/// @param sum the sum of the values (K)
/// @param square the sum of their squares (K^2)
/// @param n the number of values
void expect_uniform_statistics(double sum, double square, double n) {
    const double mean = sum / n;
    const double square_mean = square / n;
    const double mean_squared = mean * mean;
    const double variance = square_mean - mean_squared;
    const double expected = 0.01 / 3.0;
    const double mean_error = std::fabs(mean);
    const double three_amplitudes = 3.0 * 0.1;
    const double three_n = 3.0 * n;
    const double mean_limit = three_amplitudes / std::sqrt(three_n);
    EXPECT_LT(mean_error, mean_limit);
    const double variance_ratio = variance / expected;
    const double variance_error = std::fabs(variance_ratio - 1.0);
    const double relative_sigma = std::sqrt(0.8 / n);
    const double variance_limit = 3.0 * relative_sigma;
    EXPECT_LT(variance_error, variance_limit);
}

// the draws' statistics, on every standard library: within [-a, a], mean 0 and variance a^2 / 3 within their
// sampling errors (3 sigma: sigma_mean = a / sqrt(3 N), the variance's relative sigma sqrt(0.8 / N))
TEST(StartNoise, UniformInItsRange) {
    const StartNoise noise = StartNoise::per_cell(0.1 * K, 300.0 * m, 2u);
    const LevelGrid grid(207u, 10.0 * m);
    const BoxDraws box = draw_box(noise, grid, 64u, 64u);
    const double n = (double)box.values.size();
    const std::size_t expected_draws = 30u * 64u * 64u;
    ASSERT_EQ(box.values.size(), expected_draws); // the levels at 5 .. 295 m
    double sum = 0.0;
    double square = 0.0;
    for(const si::TemperatureDifference& draw : box.values) {
        const double v = draw.in(K);
        ASSERT_GE(v, -largest_draw_kelvin);
        ASSERT_LE(v, largest_draw_kelvin);
        sum += v;
        const double v2 = v * v;
        square += v2;
    }
    expect_uniform_statistics(sum, square, n);
}

// no draw at and above the top; none at all without noise
TEST(StartNoise, TopAndNone) {
    StartNoise::Sequence values = StartNoise::per_cell(0.1 * K, 30.0 * m, 2u).sequence(10.0 * m);
    const double below_top = values.at(at_height(25.0 * m)).in(K);
    EXPECT_NE(below_top, 0.0);
    const double at_top = values.at(at_height(30.0 * m)).in(K);
    EXPECT_EQ(at_top, 0.0);
    const double above_top = values.at(at_height(35.0 * m)).in(K);
    EXPECT_EQ(above_top, 0.0);
    const std::size_t draws = values.draws();
    EXPECT_EQ(draws, 1u);
    StartNoise::Sequence quiet = StartNoise::none().sequence(10.0 * m);
    const double below_floor = quiet.at(at_height(-5.0 * m)).in(K);
    EXPECT_EQ(below_floor, 0.0);
    const double in_air = quiet.at(at_height(5.0 * m)).in(K);
    EXPECT_EQ(in_air, 0.0);
    const std::size_t quiet_draws = quiet.draws();
    EXPECT_EQ(quiet_draws, 0u);
    EXPECT_FALSE(StartNoise::none().active());
}

// the same seed, the same draws; another seed, others
TEST(StartNoise, SeedDecides) {
    StartNoise::Sequence a = StartNoise::per_cell(0.1 * K, 30.0 * m, 7u).sequence(10.0 * m);
    StartNoise::Sequence b = StartNoise::per_cell(0.1 * K, 30.0 * m, 7u).sequence(10.0 * m);
    StartNoise::Sequence c = StartNoise::per_cell(0.1 * K, 30.0 * m, 8u).sequence(10.0 * m);
    const CellCentre low = at_height(5.0 * m);
    int differ = 0;
    for(int i = 0; i < 100; i++) {
        const double va = a.at(low).in(K);
        const double vb = b.at(low).in(K);
        EXPECT_EQ(va, vb);
        const double vc = c.at(low).in(K);
        if(va != vc) {
            differ++;
        }
    }
    EXPECT_EQ(differ, 100);
}

// the seed, the amplitude and the top of per-cell and block noise; none without noise
TEST(StartNoise, Seed) {
    const StartNoise per_cell = StartNoise::per_cell(0.1 * K, 30.0 * m, 7u);
    const unsigned per_cell_seed = per_cell.seed();
    EXPECT_EQ(per_cell_seed, 7u);
    const StartNoise blocks = StartNoise::blocks(0.1 * K, 30.0 * m, 9u, 10.0 * m);
    const unsigned blocks_seed = blocks.seed();
    EXPECT_EQ(blocks_seed, 9u);
    const double amplitude = per_cell.amplitude().in(K);
    EXPECT_EQ(amplitude, 0.1);
    const double top = per_cell.top().in(m);
    EXPECT_EQ(top, 30.0);
    EXPECT_THROW(StartNoise::none().seed(), SetupError);
}

// Regression for defect 9: block noise at a negative coordinate (a cell inside the floor) is refused; the cast of a
// negative cube index to an unsigned index was undefined
TEST(StartNoise, BlocksRefuseANegativeCoordinate) {
    const StartNoise noise = StartNoise::blocks(0.1 * K, 300.0 * m, 2u, 10.0 * m);
    StartNoise::Sequence sequence = noise.sequence(10.0 * m);
    const CellCentre below_floor{ .east = 5.0 * m, .north = 5.0 * m, .height = -5.0 * m };
    EXPECT_THROW(sequence.at(below_floor), SetupError);
    const CellCentre west_of_box{ .east = -1.0 * m, .north = 5.0 * m, .height = 5.0 * m };
    EXPECT_THROW(sequence.at(west_of_box), SetupError);
}

TEST(StartNoise, RefusesBadValues) {
    EXPECT_THROW(StartNoise::per_cell(0.0 * K, 30.0 * m, 2u), SetupError);
    EXPECT_THROW(StartNoise::per_cell(0.1 * K, 0.0 * m, 2u), SetupError);
}

/// @param cell the cell size
/// @param x a distance from the box's edge (or a height above the floor's top)
/// @return the centre of the cell of that size that holds the distance
si::Length centre_of(si::Length cell, si::Length x) {
    const double cells = au::as_raw_number(x / cell);
    const double index = std::floor(cells);
    const double centre_cells = index + 0.5;
    const si::Length c = cell * centre_cells;
    return c;
}

// blocks: the value of the block that holds a physical point is the same at 10, 5 and 2.5 m cells
TEST(StartNoise, BlocksAreTheSameFieldAtEveryDividingCell) {
    const StartNoise noise = StartNoise::blocks(0.1 * K, 40.0 * m, 3u, 10.0 * m);
    std::mt19937 points(11u);
    std::uniform_real_distribution<double> place(0.0, 100.0); // metres
    std::uniform_real_distribution<double> height(0.0, 40.0); // metres
    for(int i = 0; i < 200; i++) {
        const si::Length east = place(points) * m;
        const si::Length north = place(points) * m;
        const si::Length up = height(points) * m;
        double first = 0.0;
        for(const double dx : { 10.0, 5.0, 2.5 }) {
            const si::Length cell = dx * m;
            // the centre of the cell of size dx that holds the point (its height above the floor's top)
            StartNoise::Sequence values = noise.sequence(cell);
            const CellCentre point = CellCentre{ .east = centre_of(cell, east), .north = centre_of(cell, north), .height = centre_of(cell, up) };
            const double v = values.at(point).in(K);
            if(dx == 10.0) {
                first = v;
            }
            EXPECT_EQ(v, first) << "point " << i << " cell " << dx;
        }
    }
    const std::size_t draws = noise.sequence(5.0 * m).draws();
    EXPECT_EQ(draws, 0u);
}

// blocks: uniform in [-a, a) over many blocks, no value at and above the top
TEST(StartNoise, BlocksUniformInTheirRange) {
    const StartNoise noise = StartNoise::blocks(0.1 * K, 300.0 * m, 2u, 10.0 * m);
    const si::Length cell = 10.0 * m;
    StartNoise::Sequence values = noise.sequence(cell);
    double sum = 0.0;
    double square = 0.0;
    double n = 0.0;
    for(std::uint32_t z = 0u; z < 30u; z++) {
        for(std::uint32_t y = 0u; y < 64u; y++) {
            for(std::uint32_t x = 0u; x < 64u; x++) {
                const double z_cells = (double)z + 0.5;
                const si::Length h = cell * z_cells;
                const CellCentre cell_centre = centre(x, y, h, cell);
                const double v = values.at(cell_centre).in(K);
                ASSERT_GE(v, -0.1);
                ASSERT_LT(v, 0.1);
                sum += v;
                const double v2 = v * v;
                square += v2;
                n += 1.0;
            }
        }
    }
    expect_uniform_statistics(sum, square, n);
    const CellCentre at_top = CellCentre{ .east = 5.0 * m, .north = 5.0 * m, .height = 300.0 * m };
    const double top_value = values.at(at_top).in(K);
    EXPECT_EQ(top_value, 0.0);
}

/// @param seed the hash's seed
/// @param east the point's distance east
/// @param north the point's distance north
/// @param up the point's height
/// @return the value of 0.1 K blocks of 10 m (top 1000 m) at the point, on 10 m cells (K)
double pinned_value(unsigned seed, si::Length east, si::Length north, si::Length up) {
    const StartNoise noise = StartNoise::blocks(0.1 * K, 1000.0 * m, seed, 10.0 * m);
    StartNoise::Sequence values = noise.sequence(10.0 * m);
    const CellCentre point = CellCentre{ .east = east, .north = north, .height = up };
    const double value = values.at(point).in(K);
    return value;
}

// blocks: the values of a SplitMix64 copy in Python (seed, then the x, y, z block indices; 0.1 K (2 u - 1) in double),
// on every library
TEST(StartNoise, BlocksArePinned) {
    const double origin = pinned_value(2u, 5.0 * m, 5.0 * m, 5.0 * m);
    EXPECT_EQ(origin, 0x1.aa2de66666667p-7);      // the block (0, 0, 0)
    const double block_3_1_2 = pinned_value(2u, 35.0 * m, 15.0 * m, 25.0 * m);
    EXPECT_EQ(block_3_1_2, -0x1.99a8p-7);           // (3, 1, 2)
    const double block_10_20_0 = pinned_value(7u, 105.0 * m, 205.0 * m, 5.0 * m);
    EXPECT_EQ(block_10_20_0, -0x1.cd87a66666667p-5); // (10, 20, 0)
}

TEST(StartNoise, BlocksRefuseCellsThatDoNotDivide) {
    const StartNoise noise = StartNoise::blocks(0.1 * K, 300.0 * m, 2u, 10.0 * m);
    EXPECT_THROW(noise.sequence(3.0 * m), SetupError);
    EXPECT_THROW(noise.sequence(20.0 * m), SetupError);
    EXPECT_NO_THROW(noise.sequence(2.5 * m));
    EXPECT_THROW(StartNoise::blocks(0.1 * K, 305.0 * m, 2u, 10.0 * m), SetupError); // the top not whole blocks
    EXPECT_THROW(StartNoise::blocks(0.1 * K, 300.0 * m, 2u, 0.0 * m), SetupError);
    const std::string described = noise.describe();
    const std::size_t at = described.find("per 10.0 m block");
    EXPECT_NE(at, std::string::npos) << described;
}

#ifdef _MSC_VER
/// @param g a capture
/// @return the draws of its start noise on its box
BoxDraws captured_box_draws(const GoldenCapture& g) {
    const si::TemperatureDifference amplitude = g.setting("noise") * K;
    const si::Length top = g.setting("noise_top") * m;
    const unsigned seed = (unsigned)g.setting("seed");
    const StartNoise noise = StartNoise::per_cell(amplitude, top, seed);
    const std::uint32_t nz = (std::uint32_t)g.setting("Nz");
    const si::Length cell = g.setting("dx") * m;
    const LevelGrid grid(nz, cell);
    const std::uint32_t nx = (std::uint32_t)g.setting("Nx");
    const std::uint32_t ny = (std::uint32_t)g.setting("Ny");
    const BoxDraws box = draw_box(noise, grid, nx, ny);
    return box;
}
#endif // _MSC_VER

// the verified examples' draws bit for bit: the count, the first 4096 and the last, the hash of all. Only with MSVC's
// standard library: the C++ standard leaves std::uniform_real_distribution's algorithm to the library
TEST(StartNoise, DrawsAreTheVerifiedExamples) {
#ifndef _MSC_VER
    GTEST_SKIP() << "the captures were drawn with MSVC's std::uniform_real_distribution";
#else
    for(const char* name : { "quick", "sp2011" }) {
        const GoldenCapture g(name);
        const BoxDraws box = captured_box_draws(g);
        const double draws = (double)box.values.size();
        const double expected_draws = g.setting("noise_draws");
        ASSERT_EQ(draws, expected_draws) << name;
        const std::string csv_file = g.path("noise.csv");
        std::ifstream csv(csv_file);
        std::string line{};
        std::getline(csv, line); // draw,value
        std::size_t rows = 0u;
        while(std::getline(csv, line)) {
            const std::size_t c = line.find(',');
            const std::size_t i = (std::size_t)std::stoull(line.substr(0u, c));
            const double value = box.values.at(i).in(K);
            const std::string value_text = line.substr(c + 1u);
            const double expected_value = std::strtod(value_text.c_str(), nullptr);
            EXPECT_EQ(value, expected_value) << name << " draw " << i;
            rows++;
        }
        EXPECT_EQ(rows, 4097u) << name;
        const std::string hash_file = g.path("noise_hash.txt");
        std::ifstream hash(hash_file);
        std::size_t count = 0u;
        std::string hex{};
        hash >> count >> hex;
        const std::uint64_t hashed = fnv1a(box.values);
        const unsigned long long expected_hash = std::stoull(hex, nullptr, 16);
        EXPECT_EQ(hashed, expected_hash) << name;
    }
#endif
}

} // namespace
