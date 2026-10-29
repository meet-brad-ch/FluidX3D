#include "setup/atmosphere/wind_nudging.hpp"
#include "setup/core/setup_error.hpp"
#include "golden_capture.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>

namespace {

using namespace si::symbols;

/// @return a 5 m/s wind from the north at every height (exponent 0)
std::shared_ptr<const WindTarget> north_wind() {
    const std::shared_ptr<const WindTarget> wind = std::make_shared<PowerLawWind>(5.0 * m / s, 100.0 * m, 0.0, 0.0 * deg);
    return wind;
}

// by hand: the target -5 m/s along y, the mean -4: the force -1 m/s dt / dx dt / tau per step; along x the mean 0.5
TEST(WindNudging, ForcesByHand) {
    const WindNudging nudging(north_wind(), 600.0 * s);
    const LevelGrid grid(10u, 10.0 * m);
    const si::LatticeConverter converter(10.0 * m, 0.16 * s);
    std::vector<LayerMeans> means(10u);
    for(LayerMeans& l : means) {
        l.u = 0.5 * m / s;
        l.v = -4.0 * m / s;
    }
    const WindNudging::Forces f = nudging.forces(means, grid, converter);
    const double dt = (double)0.16f; // the core's float step
    const double per_step = dt / 600.0;
    const double lattice = dt / 10.0;
    EXPECT_EQ(f.fx[0], 0.0f); // the floor
    EXPECT_EQ(f.fy[9], 0.0f); // the lid
    const double fy_lattice = -1.0 * lattice;
    const double fy_double = fy_lattice * per_step;
    const float expected_fy = (float)fy_double;
    const double fx_lattice = -0.5 * lattice;
    const double fx_double = fx_lattice * per_step;
    const float expected_fx = (float)fx_double;
    for(std::uint32_t z = 1u; z <= 8u; z++) {
        EXPECT_FLOAT_EQ(f.fy[z], expected_fy) << z;
        EXPECT_NEAR(f.fx[z], expected_fx, 1.0E-12f) << z;
    }
}

// by hand: the largest |<u> - target| below zi over 5 m/s; the levels at and above zi do not count
TEST(WindNudging, DeviationBelowZi) {
    const WindNudging nudging(north_wind(), 600.0 * s);
    const LevelGrid grid(20u, 10.0 * m);
    std::vector<LayerMeans> means(20u);
    for(LayerMeans& l : means) {
        l.v = -5.0 * m / s;
    }
    means[3].u = 3.0 * m / s; // 25 m: |(-3, -4)| = 5, so 1.0
    means[3].v = -1.0 * m / s;
    means[15].u = 50.0 * m / s; // 145 m: above zi
    const double deviation = nudging.deviation(means, grid, 100.0 * m);
    EXPECT_NEAR(deviation, 1.0, 1.0E-12);
    means[3] = means[4];
    const double no_deviation = nudging.deviation(means, grid, 100.0 * m);
    EXPECT_EQ(no_deviation, 0.0);
}

// Regression for defect 24: means with fewer entries than the levels up to the last fluid level are refused with a
// message (std::vector::at() threw std::out_of_range)
TEST(WindNudging, RefusesTooFewLayerMeans) {
    const WindNudging nudging(north_wind(), 600.0 * s);
    const LevelGrid grid(10u, 10.0 * m);
    const si::LatticeConverter converter(10.0 * m, 0.16 * s);
    const std::vector<LayerMeans> fluid_only(grid.fluid_levels());
    EXPECT_THROW(nudging.forces(fluid_only, grid, converter), SetupError);
    EXPECT_THROW(nudging.deviation(fluid_only, grid, 1000.0 * m), SetupError);
}

TEST(WindNudging, RefusesNoTargetOrTau) {
    EXPECT_THROW(WindNudging(nullptr, 600.0 * s), SetupError);
    EXPECT_THROW(WindNudging(north_wind(), 0.0 * s), SetupError);
}

/// @param g a windy capture
/// @return its power-law target
std::shared_ptr<const WindTarget> wind_of(const GoldenCapture& g) {
    const si::Speed speed = g.setting("wind_speed") * m / s;
    const si::Length height = g.setting("wind_height") * m;
    const double exponent = g.setting("wind_exponent");
    const si::Angle from = g.setting("wind_from_deg") * deg;
    const std::shared_ptr<const WindTarget> wind = std::make_shared<PowerLawWind>(speed, height, exponent, from);
    return wind;
}

/// @brief One captured interval of a windy run: the level means' u and v and the nudging forces per level.
struct CapturedForces {
    std::vector<LayerMeans> means{}; ///< per level: u and v from the capture, the rest 0
    std::vector<float> fx{};         ///< the captured force along x per level
    std::vector<float> fy{};         ///< along y
};

/// @param line a row of an interval_*.csv
/// @return its numbers
std::vector<double> numbers_of(const std::string& line) {
    std::vector<double> v{};
    std::stringstream cells(line);
    std::string word{};
    while(std::getline(cells, word, ',')) {
        const double number = std::strtod(word.c_str(), nullptr);
        v.push_back(number);
    }
    return v;
}

/// @param in an interval_*.csv, after its two header lines
/// @param levels the column's levels
/// @return its level means (u, v) and its forces
CapturedForces read_forces(std::istream& in, std::uint32_t levels) {
    CapturedForces c{};
    c.means = std::vector<LayerMeans>(levels);
    c.fx = std::vector<float>(levels, 0.0f);
    c.fy = std::vector<float>(levels, 0.0f);
    std::string line{};
    while(std::getline(in, line)) {
        const std::vector<double> v = numbers_of(line);
        if(v.size() < 12u) {
            continue;
        }
        const std::size_t z = (std::size_t)v[0];
        c.means.at(z).u = v[6] * m / s;
        c.means[z].v = v[7] * m / s;
        c.fx[z] = (float)v[10];
        c.fy[z] = (float)v[11];
    }
    return c;
}

// an extra windy case (FLUIDX3D_EXTRA_CBL_WIND): the forces from the captured layer means of its first three
// intervals against the captured forces of the same intervals, bit for bit
TEST(WindNudging, PrivateWindCaseIsItsCapture) {
    const std::optional<std::string> folder = GoldenCapture::environment("FLUIDX3D_EXTRA_CBL_WIND");
    if(!folder) {
        GTEST_SKIP() << "FLUIDX3D_EXTRA_CBL_WIND is not set";
    }
    const GoldenCapture g = GoldenCapture::at(*folder);
    const si::Length cell = g.setting("dx") * m;
    const std::uint32_t nz = (std::uint32_t)g.setting("Nz");
    const LevelGrid grid(nz, cell);
    const si::Duration dt = g.setting("dt") * s;
    const si::LatticeConverter converter(cell, dt);
    const std::shared_ptr<const WindTarget> target = wind_of(g);
    const si::Duration tau = g.setting("nudge_tau") * s;
    const WindNudging nudging(target, tau);
    for(const char* name : { "interval_1.csv", "interval_2.csv", "interval_3.csv" }) {
        const std::string file = g.path(name);
        std::ifstream in(file);
        ASSERT_TRUE(in.good()) << name;
        std::string line{};
        std::getline(in, line); // # time_s ...
        std::getline(in, line); // z,theta,w_var,w_skew,theta_var,w_theta,u,v,w_max,w_min,nudge_fx,nudge_fy,...
        const CapturedForces captured = read_forces(in, grid.levels());
        const WindNudging::Forces f = nudging.forces(captured.means, grid, converter);
        for(std::uint32_t z = grid.first_fluid(); z <= grid.last_fluid(); z++) {
            EXPECT_EQ(f.fx[z], captured.fx[z]) << name << " z " << z;
            EXPECT_EQ(f.fy[z], captured.fy[z]) << name << " z " << z;
        }
    }
}

} // namespace
