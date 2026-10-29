#pragma once

/// @file profile_csv.hpp
/// @brief A CSV file of the horizontal layer means of every interval of an atmosphere run (the CBL example's
/// profiles.csv).
///
/// Header-side: it needs the build's defines (the EXTENSIONS) and the core.

#include "setup/atmosphere/interval.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/units/units.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#if !defined(TEMPERATURE) || !defined(LEVEL_FORCING)
#error "setup/atmosphere/profile_csv.hpp needs TEMPERATURE and LEVEL_FORCING in the EXTENSIONS"
#endif // TEMPERATURE, LEVEL_FORCING

/// @brief Writes the horizontal layer means of every interval of a run into a CSV file (the CBL example's
/// profiles.csv).
///
/// An IntervalObserver of the high and the middle layer: give it to ConvectiveBoundaryLayer::observe() or
/// AtmosphereSimulation::observe(). It is not owned by the run, so it must outlive run(). Each interval, the spin-up
/// too, adds one row per fluid level (from the lowest to the highest), and the file is flushed after each interval.
///
/// The columns (decimal text; the unit is in each name):
/// - time_s: the simulated time at the interval's end, %.1f;
/// - z_m: the level centre's height above the floor's top, %.2f;
/// - theta_K: the layer-mean potential temperature minus theta_0 (a difference, not a reading; in a moist run the
///   liquid water potential temperature theta_l minus theta_0), %.5f;
/// - w_var_m2ps2: the variance of w, %.5f; w_theta_Kmps: the resolved heat flux w'θ', %.6f; u_mps, v_mps: the mean
///   horizontal wind, %.5f each; w_max_mps, w_min_mps: the largest and the smallest w of the level, %.4f each;
///   theta_var_K2: the variance of theta, %.6f; w_skew: the skewness of w (a ratio, 0 without variance), %.4f;
/// - moist runs only: qt_kgkg and ql_kgkg (the mean total and liquid water, %.7f) and cloud_fraction (the share of
///   the level's cells with liquid water, %.6f).
///
/// @code
/// ProfileCsv csv(folder / "profiles.csv", description.moist());
/// run.observe(csv);
/// @endcode
class ProfileCsv final : public IntervalObserver {
public:
    /// @brief Creates the file (an existing file is replaced) and writes the header line.
    /// @param path the file's path; its folder must exist
    /// @param moist true to write the cloud columns. It must match the run: on_interval() refuses a moist observer on
    /// a dry run, which has no cloud levels.
    /// @throws SetupError when the file cannot be opened for writing, or the header cannot be written
    ProfileCsv(const std::filesystem::path& path, bool moist) : out_(path), path_(path), moist_(moist) {
        if(!out_) {
            throw SetupError("ProfileCsv: cannot write " + path.string());
        }
        const char* cloud_header = moist ? ",qt_kgkg,ql_kgkg,cloud_fraction" : "";
        out_ << "time_s,z_m,theta_K,w_var_m2ps2,w_theta_Kmps,u_mps,v_mps,w_max_mps,w_min_mps,theta_var_K2,w_skew" << cloud_header << "\n";
        out_.flush();
        check_stream();
    }

    /// @brief Writes one row per fluid level of the interval, then flushes the file.
    /// @param s the interval, valid during the call
    /// @throws SetupError for a moist observer when the interval has no cloud levels (a dry run), when a row cannot be
    /// formatted, or when the rows cannot be written
    void on_interval(const IntervalSnapshot& s) override {
        const bool has_clouds = s.profiles.clouds.has_value();
        if(moist_ && !has_clouds) {
            throw SetupError("ProfileCsv: the observer writes the cloud columns, but the run is dry (no cloud levels)");
        }
        const LevelGrid& levels = s.fields.grid().levels();
        for(std::uint32_t z = levels.first_fluid(); z <= levels.last_fluid(); z++) {
            const si::Length height = levels.height(z);
            const std::string means = means_row(s.time, height, s.profiles.means[z]);
            out_ << means;
            if(moist_) {
                const std::string clouds = cloud_columns(*s.profiles.clouds, z);
                out_ << clouds;
            }
            out_ << "\n";
        }
        out_.flush();
        check_stream();
    }

    /// @brief Closes the file at the run's end.
    /// @param summary the run's summary (not used)
    /// @throws SetupError when the file cannot be closed (its last bytes are not written)
    void on_finish(const RunSummary& summary) override {
        (void)summary;
        out_.close();
        check_stream();
    }

private:
    static constexpr std::size_t line_capacity = 256u; ///< the size in characters of the buffer for one part of a row (a longer part is refused: check_printed())

    /// @brief Checks the file's stream after writes: its fail bit stays set after a failed write.
    /// @throws SetupError that names the file when a write failed
    void check_stream() const {
        if(!out_) {
            throw SetupError("ProfileCsv: writing " + path_.string() + " failed");
        }
    }

    /// @brief Checks the result of std::snprintf(): the text was formatted and fits the buffer (it was not cut).
    /// @param length the return value of std::snprintf()
    /// @param capacity the buffer's size in characters
    /// @param what the text's name, for the message
    /// @throws SetupError when the length is negative (an encoding error) or not below the capacity (the text was cut)
    static void check_printed(int length, std::size_t capacity, const char* what) {
        const bool failed = length < 0;
        const std::size_t needed = failed ? 0u : static_cast<std::size_t>(length);
        const bool cut = needed >= capacity;
        if(failed || cut) {
            throw SetupError(std::string("ProfileCsv: cannot format ") + what);
        }
    }

    /// @brief Makes the columns from time_s to w_skew of one row.
    /// @param time the simulated time at the interval's end
    /// @param height the level centre's height above the floor's top
    /// @param means the level's layer means
    /// @return the columns in the file's units and formats (see the class), joined by commas
    /// @throws SetupError when the columns do not fit line_capacity (check_printed())
    static std::string means_row(si::Duration time, si::Length height, const LayerMeans& means) {
        using namespace si::symbols;
        const double time_s = time.in(s);
        const double height_m = height.in(m);
        const double theta = means.theta.in(K);
        const double w_var = means.w_var.in(m * m / (s * s));
        const double w_theta = means.w_theta.in(K * m / s);
        const double u = means.u.in(m / s);
        const double v = means.v.in(m / s);
        const double w_max = means.w_max.in(m / s);
        const double w_min = means.w_min.in(m / s);
        const double theta_var = means.theta_var.in(K * K);
        const double w_skew = means.w_skew;
        char line[line_capacity]{};
        const int length = std::snprintf(line, sizeof(line), "%.1f,%.2f,%.5f,%.5f,%.6f,%.5f,%.5f,%.4f,%.4f,%.6f,%.4f", time_s, height_m, theta, w_var,
                                         w_theta, u, v, w_max, w_min, theta_var, w_skew);
        check_printed(length, sizeof(line), "a row's means");
        const std::string row = line;
        return row;
    }

    /// @brief Makes the cloud columns of one row.
    /// @param c the interval's cloud levels
    /// @param z the level's index, from 0 (the floor)
    /// @return ",qt_kgkg,ql_kgkg,cloud_fraction" in the file's formats (see the class), with the leading comma
    /// @throws SetupError when the columns do not fit line_capacity (check_printed())
    static std::string cloud_columns(const CloudLevels& c, std::uint32_t z) {
        using namespace si::symbols;
        const double qt = c.qt[z].in(kg_w / kg);
        const double ql = c.ql[z].in(kg_w / kg);
        const double fraction = c.fraction[z];
        char line[line_capacity]{};
        const int length = std::snprintf(line, sizeof(line), ",%.7f,%.7f,%.6f", qt, ql, fraction);
        check_printed(length, sizeof(line), "a row's cloud columns");
        const std::string columns = line;
        return columns;
    }

    std::ofstream out_;           ///< the open CSV file
    std::filesystem::path path_{}; ///< the file's path, for the messages
    bool moist_{};      ///< true when the cloud columns are written
};
