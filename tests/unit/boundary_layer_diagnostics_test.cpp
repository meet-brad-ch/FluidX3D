#include "setup/atmosphere/boundary_layer_diagnostics.hpp"
#include "setup/atmosphere/run_checks.hpp"
#include <limits>
#include "setup/units/lattice_converter.hpp"
#include "golden_capture.hpp"

#include "moist_air.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <sstream>

namespace {

using namespace si::symbols;

/// @param levels the column's levels
/// @return layer means with every theta and flux 0
std::vector<LayerMeans> flat(std::uint32_t levels) {
    const std::vector<LayerMeans> means = std::vector<LayerMeans>(levels);
    return means;
}

// a layer of five cells with known moments: w 1 2 3 4 10 (mean 4, variance 10, central third moment 36),
// theta 0 0 0 0 1 (mean 0.2, variance 0.16), <w' theta'> = <w theta> - <w><theta> = 2 - 0.8
TEST(LayerMoments, KnownMoments) {
    LayerMoments layer{};
    const double w[] = { 1.0, 2.0, 3.0, 4.0, 10.0 };
    const double t[] = { 0.0, 0.0, 0.0, 0.0, 1.0 };
    for(int i = 0; i < 5; i++) {
        const si::TemperatureDifference theta = t[i] * K;
        const si::Speed w_cell = w[i] * m / s;
        layer.add(theta, w_cell, 0.5 * m / s, -0.25 * m / s);
    }
    const LayerMeans mean = layer.means(5.0);
    const double theta = mean.theta.in(K);
    EXPECT_DOUBLE_EQ(theta, 0.2);
    const double w_var = mean.w_var.in(m * m / (s * s));
    EXPECT_DOUBLE_EQ(w_var, 10.0);
    const double expected_skew = 36.0 / std::pow(10.0, 1.5);
    EXPECT_NEAR(mean.w_skew, expected_skew, 1.0E-12);
    const double theta_var = mean.theta_var.in(K * K);
    EXPECT_NEAR(theta_var, 0.16, 1.0E-15);
    const double w_theta = mean.w_theta.in(K * m / s);
    EXPECT_DOUBLE_EQ(w_theta, 1.2);
    const double u = mean.u.in(m / s);
    EXPECT_DOUBLE_EQ(u, 0.5);
    const double v = mean.v.in(m / s);
    EXPECT_DOUBLE_EQ(v, -0.25);
    const double w_max = mean.w_max.in(m / s);
    EXPECT_EQ(w_max, 10.0);
    const double w_min = mean.w_min.in(m / s);
    EXPECT_EQ(w_min, 1.0);
}

// no variance: the skewness 0 (no division by 0); no cell: an error
// Regression for defect 1: a non-finite vertical speed in one cell makes the layer's extremes not finite, so the
// blow-up check sees it (the running max and min dropped a NaN at the next finite cell)
TEST(LayerMoments, ANonFiniteSpeedMakesTheExtremesNotFinite) {
    LayerMoments layer{};
    const si::Speed not_a_number = std::numeric_limits<double>::quiet_NaN() * m / s;
    layer.add(0.0 * K, not_a_number, 0.0 * m / s, 0.0 * m / s);
    layer.add(0.0 * K, 1.0 * m / s, 0.0 * m / s, 0.0 * m / s);
    layer.add(0.0 * K, -1.0 * m / s, 0.0 * m / s, 0.0 * m / s);
    const LayerMeans means = layer.means(3.0);
    EXPECT_TRUE(au::isnan(means.w_max));
    EXPECT_TRUE(au::isnan(means.w_min));
    const BlowUpCheck check{};
    const std::optional<std::string> failure = check.failure(10.0 * s, means.w_min, means.w_max);
    EXPECT_TRUE(failure.has_value());
}

TEST(LayerMoments, NoVarianceNoCell) {
    LayerMoments layer{};
    for(int i = 0; i < 4; i++) {
        layer.add(0.0 * K, 2.0 * m / s, 0.0 * m / s, 0.0 * m / s);
    }
    const double w_var = layer.means(4.0).w_var.in(m * m / (s * s));
    EXPECT_EQ(w_var, 0.0);
    const double w_skew = layer.means(4.0).w_skew;
    EXPECT_EQ(w_skew, 0.0);
    EXPECT_THROW(LayerMoments().means(4.0), DiagnosticsError);
}

/// The floor's heat flux of the tests: the threshold of an entrainment flux is -1e-3 of it, -2.4e-4 K m/s.
const si::KinematicHeatFlux floor_flux = 0.24 * K * m / s;

// the inversion: the most negative flux above a tenth of the column and below the sponge; the start's zi before
TEST(BoundaryLayerDiagnostics, InversionHeight) {
    const LevelGrid grid(100u, 10.0 * m); // the lid at 985 m, a tenth 98.5 m
    BoundaryLayerDiagnostics d(grid, 500.0 * m, std::nullopt, floor_flux);
    std::vector<LayerMeans> means = flat(100u);
    const double start = d.inversion_height(means).in(m);
    EXPECT_EQ(start, 500.0); // no negative flux yet
    means[5].w_theta = -9.0 * K * m / s;   // 45 m: below a tenth of the column
    means[40].w_theta = -0.02 * K * m / s; // 395 m
    means[60].w_theta = -0.05 * K * m / s; // 595 m: the minimum
    means[80].w_theta = -0.01 * K * m / s;
    const double at_minimum = d.inversion_height(means).in(m);
    const double height_60 = grid.height(60u).in(m);
    EXPECT_EQ(at_minimum, height_60);
    means[90].w_theta = -1.0 * K * m / s; // 895 m: in the sponge from 800 m below, not searched there
    const BoundaryLayerDiagnostics with_sponge(grid, 500.0 * m, 800.0 * m, floor_flux);
    const double below_sponge = with_sponge.inversion_height(means).in(m);
    EXPECT_EQ(below_sponge, height_60);
    const double without_sponge = d.inversion_height(means).in(m);
    const double height_90 = grid.height(90u).in(m);
    EXPECT_EQ(without_sponge, height_90);
}

// before the convection, the most negative flux is rounding noise (-2e-13 K m/s at 20 s in a 10 m run): it is not an
// inversion, zi stays the start's; a flux counts only below -1e-3 of the floor's (the regression of 2026-10-27, an
// arbitrary early zi from noise)
// Regression for defect 5: a floor flux of 0 or less is refused (it made the entrainment threshold 0 or more, so
// rounding noise could count as zi again)
TEST(BoundaryLayerDiagnostics, RefusesAFloorFluxOfZeroOrLess) {
    const LevelGrid grid(100u, 10.0 * m);
    const si::KinematicHeatFlux no_flux = 0.0 * K * m / s;
    EXPECT_THROW(BoundaryLayerDiagnostics(grid, 500.0 * m, std::nullopt, no_flux), SetupError);
    const si::KinematicHeatFlux cooling = -0.05 * K * m / s;
    EXPECT_THROW(BoundaryLayerDiagnostics(grid, 500.0 * m, std::nullopt, cooling), SetupError);
}

TEST(BoundaryLayerDiagnostics, NoiseIsNotAnInversion) {
    const LevelGrid grid(100u, 10.0 * m);
    const BoundaryLayerDiagnostics d(grid, 500.0 * m, std::nullopt, floor_flux);
    std::vector<LayerMeans> means = flat(100u);
    means[30].w_theta = -2.0e-13 * K * m / s; // rounding noise
    means[70].w_theta = -1.0e-4 * K * m / s;  // above the threshold -2.4e-4: not entrainment yet
    const double with_noise = d.inversion_height(means).in(m);
    EXPECT_EQ(with_noise, 500.0);
    means[50].w_theta = -3.0e-4 * K * m / s; // below the threshold: an entrainment flux
    const double with_entrainment = d.inversion_height(means).in(m);
    const double height_50 = grid.height(50u).in(m);
    EXPECT_EQ(with_entrainment, height_50);
}

// the mixed layer's top: the lowest level above 100 m 0.5 K over the 100-600 m mean
TEST(BoundaryLayerDiagnostics, MixedLayerTop) {
    const LevelGrid grid(150u, 10.0 * m);
    const BoundaryLayerDiagnostics d(grid, 900.0 * m, std::nullopt, floor_flux);
    const si::Length inversion = 900.0 * m;
    const si::LapseRate lapse = 3.0 * K / km;
    std::vector<LayerMeans> means = flat(150u);
    for(std::uint32_t z = 1u; z < 149u; z++) {
        const si::Length height = grid.height(z);
        si::TemperatureDifference theta = 0.0 * K;
        if(height > inversion) {
            const si::Length above = height - inversion;
            theta = si::TemperatureDifference(lapse * above);
        }
        means[z].theta = theta;
    }
    // 0.5 K above 0: the first level with 0.003 (h - 900) > 0.5, h > 1066.7 m: 1075 m
    const double top = d.mixed_layer_top(means).in(m);
    EXPECT_EQ(top, 1075.0);
    means[3].theta = 5.0 * K; // 25 m: below 100 m, not a top
    const double top_with_low_excess = d.mixed_layer_top(means).in(m);
    EXPECT_EQ(top_with_low_excess, 1075.0);
    const std::vector<LayerMeans> no_inversion = flat(150u);
    EXPECT_THROW(d.mixed_layer_top(no_inversion), DiagnosticsError); // no inversion
    const LevelGrid low_grid(10u, 10.0 * m);
    const BoundaryLayerDiagnostics low(low_grid, 50.0 * m, std::nullopt, floor_flux);
    const std::vector<LayerMeans> low_means = flat(10u);
    EXPECT_THROW(low.mixed_layer_top(low_means), DiagnosticsError); // no 100-600 m level
}

/// @param column the moist column
/// @param theta the air's theta_l (K)
/// @param qt the air's total water (kg/kg)
/// @param levels the column's levels
/// @return the lowest fluid level where qt exceeds qsat at theta_l's temperature there (0: none)
std::uint32_t first_saturated_level(const MoistColumn& column, double theta, double qt, std::uint32_t levels) {
    for(std::uint32_t z = 1u; z + 1u < levels; z++) {
        const double p = column.pressure(z).in(Pa);
        const double temperature = theta * moist_air::exner(p);
        const double saturation = moist_air::qsat(temperature, p);
        if(qt > saturation) {
            return z;
        }
    }
    return 0u;
}

/// @param column the moist column
/// @param theta the air's theta_l (K)
/// @param qt the air's total water (kg/kg)
/// @return the Espy estimate: 125 m per K of dew point depression at the floor's pressure
double espy_estimate(const MoistColumn& column, double theta, double qt) {
    const double p0 = column.pressure(1u).in(Pa);
    const double vapour_numerator = qt * p0;
    const double one_minus_eps = 1.0 - moist_air::Eps;
    const double vapour_part = one_minus_eps * qt;
    const double vapour_denominator = moist_air::Eps + vapour_part;
    const double e_pa = vapour_numerator / vapour_denominator;
    const double e = e_pa / 100.0; // hPa
    const double log_ratio = std::log(e / 6.112);
    const double dew_numerator = 243.5 * log_ratio;
    const double dew_denominator = 17.67 - log_ratio;
    const double dew_celsius = dew_numerator / dew_denominator;
    const double dew = dew_celsius + 273.15;
    const double floor_temperature = theta * moist_air::exner(p0);
    const double depression = floor_temperature - dew;
    const double espy = 125.0 * depression;
    return espy;
}

// the condensation level of well-mixed air: the lowest level where qt exceeds qsat at theta_l's temperature there
// (found here from moist_air's qsat directly), and near the Espy estimate 125 m per K of dew point depression
TEST(BoundaryLayerDiagnostics, CondensationLevel) {
    const double theta = 300.0; // K: the raw model's units
    const double qt = 0.012;    // kg/kg: the raw model's units
    const si::Temperature t = si::kelvin_point(theta);
    const si::WaterContent q = qt * kg_w / kg;
    const std::vector<TemperaturePoint> thl_points{ { 0.0 * m, t }, { 5000.0 * m, t } };
    const std::vector<WaterContentPoint> qt_points{ { 0.0 * m, q }, { 5000.0 * m, q } };
    const TemperatureProfile thl = TemperatureProfile::from_points(thl_points);
    const WaterContentProfile total_water = WaterContentProfile::from_points(qt_points);
    const Sounding sounding = Sounding::tabulated(thl, total_water, t);
    const LevelGrid grid(400u, 10.0 * m);
    const MoistColumn column = MoistColumn::hydrostatic(sounding, grid);
    const BoundaryLayerDiagnostics d(grid, 1000.0 * m, std::nullopt, floor_flux);
    const std::vector<si::WaterContent> water(400u, q);
    const std::vector<LayerMeans> means = flat(400u);
    const std::optional<si::Length> lcl = d.condensation_level(means, water, column, t, 1000.0 * m);
    ASSERT_TRUE(lcl.has_value());
    const std::uint32_t expected = first_saturated_level(column, theta, qt, 400u);
    ASSERT_GT(expected, 0u);
    const double lcl_m = lcl->in(m);
    const double expected_m = grid.height(expected).in(m);
    EXPECT_EQ(lcl_m, expected_m);
    // Espy: the dew point of qt at the floor's pressure, 125 m per K of depression
    const double espy = espy_estimate(column, theta, qt);
    const double tolerance = 0.05 * espy;
    EXPECT_NEAR(lcl_m, espy, tolerance);
    // dry air: no level below the lid
    const std::vector<si::WaterContent> dry(400u, 0.0001 * kg_w / kg);
    const std::optional<si::Length> dry_lcl = d.condensation_level(means, dry, column, t, 1000.0 * m);
    EXPECT_FALSE(dry_lcl.has_value());
    // an empty window
    const std::optional<si::Length> empty_window = d.condensation_level(means, water, column, t, 1.0 * m);
    EXPECT_FALSE(empty_window.has_value());
}

/// @brief One recorded interval of a capture: its header's zi and the level means.
struct CapturedInterval {
    si::Length zi{ au::ZERO };       ///< the header's zi_m
    std::vector<LayerMeans> means{}; ///< per level (index z)
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

/// @param v a row's numbers: z, theta, w_var, w_skew, theta_var, w_theta, u, v, w_max, w_min
/// @return the row's level means
LayerMeans means_of(const std::vector<double>& v) {
    LayerMeans l{};
    l.theta = v[1] * K;
    l.w_var = v[2] * m * m / (s * s);
    l.w_skew = v[3];
    l.theta_var = v[4] * K * K;
    l.w_theta = v[5] * K * m / s;
    l.u = v[6] * m / s;
    l.v = v[7] * m / s;
    l.w_max = v[8] * m / s;
    l.w_min = v[9] * m / s;
    return l;
}

/// @param path an interval_*.csv
/// @param levels the column's levels
/// @return the interval
CapturedInterval read_interval(const std::string& path, std::uint32_t levels) {
    std::ifstream in(path);
    if(!in) {
        throw std::runtime_error("no " + path);
    }
    CapturedInterval c{};
    c.means.resize(levels);
    std::string line{};
    std::getline(in, line); // # time_s <hex> <dec> zi_m <hex> <dec>
    std::istringstream header(line);
    std::string hash{};
    std::string time_key{};
    std::string time_hex{};
    std::string time_dec{};
    std::string zi_key{};
    std::string zi_hex{};
    header >> hash >> time_key >> time_hex >> time_dec >> zi_key >> zi_hex;
    const double zi_m = std::strtod(zi_hex.c_str(), nullptr);
    c.zi = zi_m * m;
    std::getline(in, line); // the column names
    while(std::getline(in, line)) {
        const std::vector<double> v = numbers_of(line);
        if(v.size() < 10u) {
            continue;
        }
        const std::size_t z = (std::size_t)v[0];
        c.means.at(z) = means_of(v);
    }
    return c;
}

/// @param g the capture
/// @param fields its raw fields of one interval (u.x, u.y, u.z, T)
/// @param interval the same interval's level means
void check_interval(const GoldenCapture& g, const std::string& fields, const std::string& interval) {
    const std::uint32_t nx = (std::uint32_t)g.setting("Nx");
    const std::uint32_t ny = (std::uint32_t)g.setting("Ny");
    const std::uint32_t nz = (std::uint32_t)g.setting("Nz");
    const std::size_t n = (std::size_t)nx * ny * nz;
    std::vector<float> ux(n);
    std::vector<float> uy(n);
    std::vector<float> uz(n);
    std::vector<float> temperature(n);
    const std::string fields_path = g.path(fields);
    std::ifstream bin(fields_path, std::ios::binary);
    ASSERT_TRUE(bin.good());
    const std::streamsize field_bytes = (std::streamsize)(n * sizeof(float));
    for(std::vector<float>* a : { &ux, &uy, &uz, &temperature }) {
        bin.read(reinterpret_cast<char*>(a->data()), field_bytes);
    }
    ASSERT_TRUE(bin.good());
    const si::Length dx = g.setting("dx") * m;
    const si::Duration dt = g.setting("dt") * s;
    const si::LatticeConverter converter(dx, dt); // the fields' speeds and temperatures (AtmosphereFields)
    const LevelGrid grid(nz, g.setting("dx") * m);
    std::vector<LayerMeans> means(nz);
    for(std::uint32_t z = 1u; z + 1u < nz; z++) {
        LayerMoments layer{};
        for(std::uint32_t y = 0u; y < ny; y++) {
            for(std::uint32_t x = 0u; x < nx; x++) {
                const std::size_t row = (std::size_t)y + (std::size_t)z * ny;
                const std::size_t i = (std::size_t)x + row * nx;
                const si::TemperatureDifference theta = converter.si_excess((double)temperature[i]);
                const si::Speed w = converter.si_velocity((double)uz[i]);
                const si::Speed u = converter.si_velocity((double)ux[i]);
                const si::Speed v = converter.si_velocity((double)uy[i]);
                layer.add(theta, w, u, v);
            }
        }
        const double cells = (double)nx * (double)ny;
        means[z] = layer.means(cells);
    }
    const CapturedInterval run = read_interval(g.path(interval), nz);
    for(std::uint32_t z = 1u; z + 1u < nz; z++) {
        const LayerMeans& a = means[z];
        const LayerMeans& b = run.means[z];
        const double a_theta = a.theta.in(K);
        const double b_theta = b.theta.in(K);
        EXPECT_EQ(a_theta, b_theta) << fields << " z " << z;
        const double a_w_var = a.w_var.in(m * m / (s * s));
        const double b_w_var = b.w_var.in(m * m / (s * s));
        EXPECT_EQ(a_w_var, b_w_var) << fields << " z " << z;
        EXPECT_EQ(a.w_skew, b.w_skew) << fields << " z " << z;
        const double a_theta_var = a.theta_var.in(K * K);
        const double b_theta_var = b.theta_var.in(K * K);
        EXPECT_EQ(a_theta_var, b_theta_var) << fields << " z " << z;
        const double a_w_theta = a.w_theta.in(K * m / s);
        const double b_w_theta = b.w_theta.in(K * m / s);
        EXPECT_EQ(a_w_theta, b_w_theta) << fields << " z " << z;
        const double a_u = a.u.in(m / s);
        const double b_u = b.u.in(m / s);
        EXPECT_EQ(a_u, b_u) << fields << " z " << z;
        const double a_v = a.v.in(m / s);
        const double b_v = b.v.in(m / s);
        EXPECT_EQ(a_v, b_v) << fields << " z " << z;
        const double a_w_max = a.w_max.in(m / s);
        const double b_w_max = b.w_max.in(m / s);
        EXPECT_EQ(a_w_max, b_w_max) << fields << " z " << z;
        const double a_w_min = a.w_min.in(m / s);
        const double b_w_min = b.w_min.in(m / s);
        EXPECT_EQ(a_w_min, b_w_min) << fields << " z " << z;
    }
    std::optional<si::Length> sponge{};
    if(g.setting("sponge_from") > 0.0) {
        sponge = g.setting("sponge_from") * m;
    }
    const si::KinematicHeatFlux captured_flux = g.setting("flux") * K * m / s;
    const BoundaryLayerDiagnostics d(grid, g.setting("zi") * m, sponge, captured_flux);
    const double zi = d.inversion_height(means).in(m);
    const double run_zi = run.zi.in(m);
    EXPECT_EQ(zi, run_zi);
}

// the captured raw fields of a run (FLUIDX3D_CBL_FIELDS: the quick case's capture folder with fields.bin, outside the
// repository): the layer means and zi from them against the run's own interval_1.csv and interval_last.csv, bit for bit
TEST(BoundaryLayerDiagnostics, CapturedFieldsGiveTheRunsMeans) {
    const std::optional<std::string> folder = GoldenCapture::environment("FLUIDX3D_CBL_FIELDS");
    if(!folder) {
        GTEST_SKIP() << "FLUIDX3D_CBL_FIELDS is not set";
    }
    const GoldenCapture g = GoldenCapture::at(*folder);
    check_interval(g, "fields.bin", "interval_1.csv");
    check_interval(g, "fields_last.bin", "interval_last.csv");
}

} // namespace
