#pragma once

/// @file stage_capture.hpp
/// @brief The stage capture of a convective boundary layer run: the exact values of each stage of the set-up, the
/// level means of the first three intervals and of the last one, and the raw fields of the first and the last
/// interval, written into a folder. The golden data (tests/golden/cbl) is made from it.
///
/// Header-side: it needs the build's defines (the EXTENSIONS) and the core.

#include "setup/atmosphere/convective_boundary_layer.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/units/units.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#if !defined(VOLUME_FORCE) || !defined(TEMPERATURE) || !defined(LEVEL_FORCING)
#error "setup/atmosphere/stage_capture.hpp needs VOLUME_FORCE, TEMPERATURE and LEVEL_FORCING in the EXTENSIONS"
#endif // VOLUME_FORCE, TEMPERATURE, LEVEL_FORCING

/// @brief Writes a number of a capture file in an exact form that is the same on every compiler.
///
/// The form is a hex float made from the double's bits: an optional "-", "0x1." for a normal number or "0x0." for
/// zero and subnormals, the 52-bit mantissa as exactly 13 lower-case hex digits, "p" and the binary exponent with its
/// sign. Examples: 10 is 0x1.4000000000000p+3, 0 is 0x0.0000000000000p+0, a subnormal has the exponent -1022.
/// Infinity and NaN keep the exponent field 2047: infinity is 0x1.0000000000000p+1024 (with "-" when negative), and
/// a NaN is 0x1., its nonzero mantissa, then p+1024. The
/// printf %a form is not used because its digit count and its subnormal form differ between compilers. A float
/// widens to double exactly, so a float value has the same form as its double.
class CaptureNumber {
public:
    /// @brief Writes a number as a hex float with 13 hex digits (see the class).
    /// @param v the number (a float widens exactly)
    /// @return the hex float text, for example "0x1.4000000000000p+3"
    /// @throws SetupError when the text does not fit its buffer (check_printed(); not possible for a double)
    static std::string hex(double v) {
        const std::uint64_t bits = std::bit_cast<std::uint64_t>(v);
        const std::uint64_t sign = bits >> sign_shift;
        const bool negative = sign != 0u;
        const std::uint64_t shifted = bits >> mantissa_bits;
        const std::uint64_t exponent_field = shifted & exponent_mask;
        const int biased = static_cast<int>(exponent_field);
        const std::uint64_t mantissa = bits & mantissa_mask;
        const bool subnormal_or_zero = biased == 0;
        const bool zero = subnormal_or_zero && mantissa == 0u;
        const int leading = subnormal_or_zero ? 0 : 1;                           // subnormals and zero: 0x0.
        int exponent{ 0 };
        if(zero) {
            exponent = 0;
        } else if(subnormal_or_zero) {
            exponent = 1 - exponent_bias;
        } else {
            exponent = biased - exponent_bias;
        }
        const char* sign_text = negative ? "-" : "";
        const unsigned long long mantissa_digits = static_cast<unsigned long long>(mantissa);
        constexpr std::size_t text_capacity{ 48u }; // "-0x1." + 13 digits + "p+1024" with room
        char text[text_capacity]{};
        const int length = std::snprintf(text, sizeof(text), "%s0x%d.%013llxp%+d", sign_text, leading, mantissa_digits, exponent);
        check_printed(length, sizeof(text), "a hex float");
        const std::string hex_text = text;
        return hex_text;
    }
    /// @brief Writes a number as its hex float and, for a reader, its decimal with 9 significant digits (%.9g).
    /// @param v the number
    /// @return "<hex float> <decimal>", for example "0x1.4000000000000p+3 10"; the hex float is the exact value
    /// @throws SetupError when a text does not fit its buffer (check_printed(); not possible for a double)
    static std::string exact(double v) {
        constexpr std::size_t decimal_capacity{ 32u }; // %.9g of a double: at most 16 characters
        char decimal[decimal_capacity]{};
        const int length = std::snprintf(decimal, sizeof(decimal), "%.9g", v);
        check_printed(length, sizeof(decimal), "a decimal");
        const std::string hex_text = hex(v);
        const std::string both = hex_text + " " + decimal;
        return both;
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
            throw SetupError(std::string("CaptureNumber: cannot format ") + what);
        }
    }

private:
    static constexpr unsigned sign_shift = 63u;                       ///< the position of a double's sign bit
    static constexpr unsigned mantissa_bits = 52u;                    ///< the number of a double's mantissa bits (the shift to its exponent field)
    static constexpr std::uint64_t exponent_mask = 0x7FFu;            ///< the mask of a double's 11 exponent bits, after the shift
    static constexpr std::uint64_t mantissa_mask = (1ull << 52) - 1u; ///< the mask of a double's 52 mantissa bits
    static constexpr int exponent_bias = 1023;                        ///< the bias of a double's exponent field
};

/// @brief Writes the stage capture of a ConvectiveBoundaryLayer run (the CBL example's --capture): the exact values of
/// each stage of the set-up and of the first three and the last interval, into a folder.
///
/// An IntervalObserver of the high layer (L3). It only reads: the run is the same with and without it (the same
/// fingerprint and profiles.csv). Make it after the run's constructor and before run(), and give it to observe(). The
/// constructor writes the set-up files at once; on_interval() writes the interval files. It holds a reference to the
/// run, so the run must outlive it. The numbers are in the exact form of CaptureNumber.
///
/// The files (in the order they are written):
/// - settings.txt: one line per value. A count is "key value"; a number is "key hex decimal" (CaptureNumber::exact()).
///   The keys: the cells (Nx, Ny, Nz) and the domains along x (Dx), patch_layers (the patch's flux factor,
///   HeatedPatch::flux_factor(); 0 without a patch), patch_surface (1 for PatchMode::SURFACE_LAYER), the noise seed
///   (0 without noise), moist (1 or 0); the
///   description's physical values in m, s, K, K/m and K m/s (dx, dt, theta_0, zi, jump, jump_depth, lapse, flux,
///   patch, patch_north, noise, noise_top, spinup, record, interval, bowen, sponge_from, sponge_tau, wind_speed,
///   wind_height, wind_exponent, wind_from_deg, nudge_tau; a part that the run does not have is 0); the core's lattice
///   numbers (nu, alpha, inv_prandtl_t, fz, speed in m/s, beta, heat_rate, moisture_flux in kg/kg m/s,
///   moisture_rate, nudge_per_step; moisture_flux and moisture_rate are 0 for a dry run without a Bowen ratio); the step counts (steps_interval, steps_spinup, steps_record), patch_cells and
///   noise_draws.
/// - levels.csv: one row per level, the walls too: z (the level index from 0, the floor, a decimal integer), then
///   hex floats: height_m (the level centre above the floor's top), start_theta_K (the sounding's theta - theta_0 at
///   that height, without the start noise and an excess), T_ref and sponge_rate (the core's level arrays),
///   wind_target_x_mps and wind_target_y_mps (the held wind's target at that height), start_ux and start_uy (the
///   start wind that the core gets, in cells per step: the target at a fluid level, 0 in the floor and the lid rows,
///   because the start builder writes the wind only into fluid cells), qt_kgkg,
///   pressure_Pa and theta_v_K (the moist column; 0 in a dry run).
/// - patch.csv: the x,y indices (from 0) of the floor cells under the patch, y in the outer loop.
/// - noise.csv: the start noise's draws in the start's order (z, then y, then x) as "index,hex float in K", the first
///   written_draws and the last one. noise_hash.txt: "count hash", the FNV-1a hash (16 hex digits) over all the draws
///   as floats. Only StartNoise::per_cell() makes draws: with StartNoise::blocks() (and without noise) noise.csv
///   holds only its header, the count is 0 and the hash is the FNV-1a offset basis.
/// - fingerprint.txt: the set-up's FNV-1a hash (AtmosphereSimulation::fingerprint()), 16 hex digits.
/// - interval_1.csv, interval_2.csv, interval_3.csv (the first three intervals, spin-up included) and
///   interval_last.csv: see write_interval().
/// - fields.bin and fields.txt (the first interval), fields_last.bin and fields_last.txt (the last): see
///   write_fields().
class StageCapture final : public IntervalObserver {
public:
    /// The writer of one settings line: number(key, value) writes "key hex decimal".
    using NumberWriter = std::function<void(const char* key, double value)>;

    /// The number of intervals, from the first one, whose level means are written (interval_1.csv to interval_3.csv);
    /// the last interval is written too.
    static constexpr std::uint32_t intervals = 3u;
    /// The number of noise draws, from the first one, that noise.csv holds; the last draw is written too, and
    /// noise_hash.txt covers all of them.
    static constexpr std::size_t written_draws = 4096u;

    /// @brief Writes the set-up files of a built run: settings.txt, levels.csv, patch.csv, noise.csv,
    /// noise_hash.txt and fingerprint.txt.
    /// @param folder the capture folder (created when it is missing)
    /// @param run the run, built and not yet run; it must outlive the capture
    /// @throws SetupError when a file cannot be opened or written, a text cannot be formatted, or a moist run's Bowen
    /// ratio is not more than 0 (write_moisture())
    /// @throws std::filesystem::filesystem_error when the folder cannot be created
    StageCapture(const std::filesystem::path& folder, ConvectiveBoundaryLayer& run) : folder_(folder), run_(run) {
        std::filesystem::create_directories(folder_);
        const CblDescription& d = run.description();
        const std::optional<WindForcing>& held_wind = d.wind();
        if(held_wind.has_value()) {
            const WindForcing& held = *held_wind;
            const std::shared_ptr<const WindTarget> target = std::make_shared<PowerLawWind>(held.wind);
            nudging_ = std::make_unique<WindNudging>(target, held.nudge_tau);
        }
        write_settings();
        write_levels();
        write_patch();
        write_noise();
        std::ofstream f = open("fingerprint.txt");
        const std::uint64_t fingerprint = run.atmosphere().fingerprint();
        const unsigned long long fingerprint_bits = static_cast<unsigned long long>(fingerprint);
        constexpr std::size_t line_capacity{ 40u }; // 16 hex digits and a newline
        char t[line_capacity]{};
        const int length = std::snprintf(t, sizeof(t), "%016llx\n", fingerprint_bits);
        CaptureNumber::check_printed(length, sizeof(t), "the fingerprint");
        f << t;
        close(f, "fingerprint.txt");
    }

    /// @brief Writes the files of an interval: fields.bin/.txt for the first interval, interval_N.csv for the first
    /// three, and fields_last.bin/.txt and interval_last.csv for the last (the interval that ends at or after the
    /// spin-up's and the recorded time's steps).
    /// @param s the interval, valid during the call
    /// @throws SetupError when a file cannot be opened or written
    void on_interval(const IntervalSnapshot& s) override {
        const CblDescription& d = run_.description();
        const std::uint64_t all_steps = d.spinup_steps() + d.record_steps();
        const bool last = s.step >= all_steps;
        if(seen_ == 0u) {
            write_fields(s, "fields");
        }
        if(seen_ < intervals) {
            const std::uint32_t number = seen_ + 1u;
            const std::string name = "interval_" + std::to_string(number);
            write_interval(s, name);
        }
        if(last) {
            write_fields(s, "fields_last");
            write_interval(s, "interval_last");
        }
        seen_++;
    }
    /// @brief Does nothing: every file is written by the end of the last interval.
    /// @param summary the run's summary (not used)
    void on_finish(const RunSummary& summary) override { (void)summary; }

private:
    /// @brief Opens a file of the capture folder for writing (an existing file is replaced).
    /// @param name the file's name in the folder
    /// @param binary true for binary mode (the .bin fields), false for text mode
    /// @return the open file
    /// @throws SetupError when the file cannot be opened
    std::ofstream open(const std::string& name, bool binary = false) const {
        const std::filesystem::path path = folder_ / name;
        const std::ios::openmode mode = binary ? std::ios::binary : std::ios::out;
        std::ofstream out(path, mode);
        if(!out) {
            throw SetupError("StageCapture: cannot write " + path.string());
        }
        return out;
    }

    /// @brief Closes a file of the capture folder and checks that every write to it succeeded: the stream's fail bit
    /// stays set after a failed write, and close() writes the buffered bytes.
    /// @param out the file, open() gave it
    /// @param name the file's name in the folder, for the message
    /// @throws SetupError that names the file when a write or the close failed
    void close(std::ofstream& out, const std::string& name) const {
        out.close();
        if(!out) {
            const std::filesystem::path path = folder_ / name;
            throw SetupError("StageCapture: writing " + path.string() + " failed");
        }
    }

    /// @brief Writes settings.txt: one line per value, a count as "key value", a number as "key hex decimal" (the keys
    /// are listed in the class description).
    /// @throws SetupError when the file cannot be opened or written, or a moist run's Bowen ratio is not more than 0
    void write_settings() const {
        std::ofstream out = open("settings.txt");
        const auto number = [&](const char* key, double v) { out << key << " " << CaptureNumber::exact(v) << "\n"; };
        const auto count = [&](const char* key, unsigned long long v) { out << key << " " << v << "\n"; };
        const CblDescription& d = run_.description();
        const AtmosphereGrid g = d.grid();
        const HeatedPatch& patch = d.surface().patch;
        const LatticeParameters p = run_.parameters();
        count("Nx", g.nx());
        count("Ny", g.ny());
        count("Nz", g.levels().levels());
        count("Dx", run_.lbm().get_Dx());
        std::uint32_t patch_layers{ 0u };
        if(patch.active()) {
            patch_layers = patch.flux_factor();
        }
        count("patch_layers", patch_layers);
        const bool surface_patch = patch.active() && patch.mode() == PatchMode::SURFACE_LAYER;
        const unsigned patch_surface = surface_patch ? 1u : 0u;
        count("patch_surface", patch_surface);
        unsigned seed{ 0u };
        if(d.noise().active()) {
            seed = d.noise().seed();
        }
        count("seed", seed);
        const unsigned moist = d.moist() ? 1u : 0u;
        count("moist", moist);
        write_physical(number);
        number("nu", p.nu);
        number("alpha", p.alpha);
        number("inv_prandtl_t", p.inv_prandtl);
        number("fz", p.fz);
        const double speed = p.speed.in(au::meters / au::second);
        number("speed", speed);
        number("beta", p.beta);
        number("heat_rate", p.heat_rate);
        write_moisture(number);
        number("nudge_per_step", p.nudge_per_step);
        count("steps_interval", d.interval_steps());
        count("steps_spinup", d.spinup_steps());
        count("steps_record", d.record_steps());
        count("patch_cells", run_.atmosphere().sources().patch_cells);
        const std::vector<double> draws = noise_draws();
        count("noise_draws", draws.size());
        close(out, "settings.txt");
    }

    /// @brief Writes the floor's moisture lines: moisture_flux (in kg of water per kg of air, times m/s) and
    /// moisture_rate (in lattice units), from the heat flux and the Bowen ratio (SurfaceForcing::moisture_flux()).
    /// A dry run uses no Bowen ratio: when it is not more than 0, both lines are 0.
    /// @param number the writer of one "key hex decimal" line: number(key, value)
    /// @throws SetupError for a moist run whose Bowen ratio is not more than 0 (SurfaceForcing::moisture_flux())
    void write_moisture(const NumberWriter& number) const {
        const CblDescription& d = run_.description();
        const bool has_bowen = d.surface().bowen > 0.0;
        const bool moisture_known = d.moist() || has_bowen;
        double moisture_flux_value{ 0.0 };
        float moisture_rate{ 0.0f };
        if(moisture_known) {
            const si::MoistureFlux moisture_flux = d.surface().moisture_flux();
            moisture_flux_value = moisture_flux.in(si::kilograms_of_water / si::kilograms * au::meters / au::second);
            const si::LatticeConverter converter = d.converter();
            moisture_rate = converter.moisture_rate(moisture_flux);
        }
        number("moisture_flux", moisture_flux_value);
        number("moisture_rate", moisture_rate);
    }

    /// @brief Writes the description's physical values to settings.txt, in m, s, K, K/m and K m/s (a part that the
    /// run does not have, such as the sponge or the wind, is 0).
    /// @param number the writer of one "key hex decimal" line: number(key, value)
    void write_physical(const NumberWriter& number) const {
        const CblDescription& d = run_.description();
        const double dx = d.cell().in(au::meters);
        number("dx", dx);
        const double dt = d.converter().step().in(au::seconds);
        number("dt", dt);
        const double theta_0 = si::in_kelvin(d.sounding().reference());
        number("theta_0", theta_0);
        const double zi = d.inversion().in(au::meters);
        number("zi", zi);
        write_profile(number);
        const double flux = d.surface().flux.in(au::kelvins * au::meters / au::second);
        number("flux", flux);
        write_patch_and_noise(number);
        const RunSchedule& schedule = d.schedule();
        const double spinup = schedule.spinup.in(au::seconds);
        number("spinup", spinup);
        const double record = schedule.record.in(au::seconds);
        number("record", record);
        const double interval = schedule.interval.in(au::seconds);
        number("interval", interval);
        number("bowen", d.surface().bowen);
        write_sponge(number);
        write_wind(number);
    }

    /// @brief Writes the sponge's lines: sponge_from (its lowest height above the floor's top, in m) and sponge_tau
    /// (its time scale at the lid, in s); both 0 without a sponge.
    /// @param number the writer of one "key hex decimal" line: number(key, value)
    void write_sponge(const NumberWriter& number) const {
        const SpongeProfile& sponge = run_.description().sponge();
        double sponge_from{ 0.0 };
        if(sponge.start()) {
            sponge_from = sponge.start()->in(au::meters);
        }
        number("sponge_from", sponge_from);
        double sponge_tau{ 0.0 };
        if(sponge.tau()) {
            sponge_tau = sponge.tau()->in(au::seconds);
        }
        number("sponge_tau", sponge_tau);
    }

    /// @brief Writes the idealized sounding's lines: jump (the inversion's step in K), jump_depth (in m) and lapse (the
    /// gradient above, in K/m); all 0 for a tabulated sounding.
    /// @param number the writer of one "key hex decimal" line: number(key, value)
    void write_profile(const NumberWriter& number) const {
        const std::optional<IdealizedProfile>& ideal = run_.description().sounding().idealized_profile();
        double jump{ 0.0 };
        double jump_depth{ 0.0 };
        double lapse{ 0.0 };
        if(ideal) {
            jump = ideal->jump.in(au::kelvins);
            jump_depth = ideal->jump_depth.in(au::meters);
            lapse = ideal->lapse.in(au::kelvins / au::meter);
        }
        number("jump", jump);
        number("jump_depth", jump_depth);
        number("lapse", lapse);
    }

    /// @brief Writes the patch's and the noise's lines: patch (the diameter in m), patch_north (the centre's distance
    /// north of the box's middle in m), noise (the largest perturbation in K) and noise_top (in m); a part that the run
    /// does not have is 0.
    /// @param number the writer of one "key hex decimal" line: number(key, value)
    void write_patch_and_noise(const NumberWriter& number) const {
        const CblDescription& d = run_.description();
        const HeatedPatch& patch = d.surface().patch;
        double diameter{ 0.0 };
        double north{ 0.0 };
        if(patch.active()) {
            diameter = patch.diameter().in(au::meters);
            north = patch.north_of_middle().in(au::meters);
        }
        number("patch", diameter);
        number("patch_north", north);
        const StartNoise& noise = d.noise();
        double amplitude{ 0.0 };
        double noise_top{ 0.0 };
        if(noise.active()) {
            amplitude = noise.amplitude().in(au::kelvins);
            noise_top = noise.top().in(au::meters);
        }
        number("noise", amplitude);
        number("noise_top", noise_top);
    }

    /// @brief Writes the held wind's lines: wind_speed (the power law's speed in m/s at wind_height in m),
    /// wind_exponent (a ratio), wind_from_deg (the direction it comes from, clockwise from north) and nudge_tau (in s);
    /// all 0 without wind.
    /// @param number the writer of one "key hex decimal" line: number(key, value)
    void write_wind(const NumberWriter& number) const {
        const std::optional<WindForcing>& wind = run_.description().wind();
        if(!wind) {
            for(const char* key : { "wind_speed", "wind_height", "wind_exponent", "wind_from_deg", "nudge_tau" }) {
                number(key, 0.0);
            }
            return;
        }
        const double speed = wind->wind.reference_speed().in(au::meters / au::second);
        number("wind_speed", speed);
        const double height = wind->wind.reference_height().in(au::meters);
        number("wind_height", height);
        const double exponent = wind->wind.exponent();
        number("wind_exponent", exponent);
        const double from = wind->wind.from().in(au::degrees);
        number("wind_from_deg", from);
        const double tau = wind->nudge_tau.in(au::seconds);
        number("nudge_tau", tau);
    }

    /// @brief Writes levels.csv: a header, then one row per level from the floor (z 0) to the lid, the walls too (the
    /// columns are listed in the class description).
    /// @throws SetupError when the file cannot be opened or written
    void write_levels() const {
        std::ofstream out = open("levels.csv");
        out << "z,height_m,start_theta_K,T_ref,sponge_rate,wind_target_x_mps,wind_target_y_mps,start_ux,start_uy,qt_kgkg,pressure_Pa,theta_v_K\n";
        const CblDescription& d = run_.description();
        const AtmosphereGrid grid = d.grid();
        const LevelGrid& levels = grid.levels();
        const LevelArrays& a = run_.atmosphere().level_forcing().lattice_levels();
        const si::LatticeConverter& c = run_.atmosphere().converter();
        for(std::uint32_t z = 0u; z < levels.levels(); z++) {
            const si::Length h = levels.height(z);
            const bool fluid = z >= levels.first_fluid() && z <= levels.last_fluid();
            const std::string row = level_row(z, h, fluid, a, c);
            out << z << "," << row << "\n";
        }
        close(out, "levels.csv");
    }

    /// @brief Makes one row of levels.csv after its z column.
    /// @param z the level's index, from 0 (the floor)
    /// @param h the level centre's height above the floor's top (negative for the floor)
    /// @param fluid true for a fluid level, false for the floor and the lid (their start wind is 0)
    /// @param a the core's level arrays (LevelForcing::lattice_levels())
    /// @param c the run's conversions (for the start wind in cells per step)
    /// @return the columns from height_m to theta_v_K as hex floats, joined by commas
    std::string level_row(std::uint32_t z, si::Length h, bool fluid, const LevelArrays& a, const si::LatticeConverter& c) const {
        const CblDescription& d = run_.description();
        HorizontalWind w{ .x = si::Speed{ au::ZERO }, .y = si::Speed{ au::ZERO } };
        const std::optional<WindForcing>& held_wind = d.wind();
        if(held_wind.has_value()) {
            w = held_wind->wind.at(h);
        }
        const double height_m = h.in(au::meters);
        const si::TemperatureDifference start_excess = d.sounding().theta_excess(h);
        const double start_theta = start_excess.in(au::kelvins);
        const float t_ref_value = a.T_ref[z];
        const float rate_value = a.rate[z];
        const double target_x_mps = w.x.in(au::meters / au::second);
        const double target_y_mps = w.y.in(au::meters / au::second);
        float start_ux_value{ 0.0f }; // a wall cell gets no start wind (AtmosphereStartBuilder)
        float start_uy_value{ 0.0f };
        if(fluid) {
            start_ux_value = c.velocity(w.x);
            start_uy_value = c.velocity(w.y);
        }
        const std::string height = CaptureNumber::hex(height_m);
        const std::string theta = CaptureNumber::hex(start_theta);
        const std::string t_ref = CaptureNumber::hex(t_ref_value);
        const std::string rate = CaptureNumber::hex(rate_value);
        const std::string target_x = CaptureNumber::hex(target_x_mps);
        const std::string target_y = CaptureNumber::hex(target_y_mps);
        const std::string start_ux = CaptureNumber::hex(start_ux_value);
        const std::string start_uy = CaptureNumber::hex(start_uy_value);
        const std::string moist = moist_columns(z, h);
        const std::string row = height + "," + theta + "," + t_ref + "," + rate + "," + target_x + "," + target_y + "," + start_ux + "," +
                                start_uy + "," + moist;
        return row;
    }

    /// @brief Makes the moist columns of one row of levels.csv. A wall level takes the total water of the nearest
    /// fluid height (its height is clamped to the fluid).
    /// @param z the level's index, from 0 (the floor)
    /// @param h the level centre's height above the floor's top
    /// @return qt_kgkg (the sounding's total water), pressure_Pa and theta_v_K (the moist column) as hex floats; all 0
    /// in a dry run, and pressure and theta_v 0 in a build without MOISTURE
    std::string moist_columns(std::uint32_t z, si::Length h) const {
        const CblDescription& d = run_.description();
        if(!d.moist()) {
            const std::string zero = CaptureNumber::hex(0.0);
            const std::string columns = zero + "," + zero + "," + zero;
            return columns;
        }
        const AtmosphereGrid grid = d.grid();
        const LevelGrid& levels = grid.levels();
        const si::Length fluid_top = levels.height(levels.last_fluid());
        const si::Length clamped = au::clamp(h, au::meters(0.0), fluid_top); // the walls hold the nearest fluid's water
        const si::WaterContent water = d.sounding().total_water(clamped);
        const double water_kgkg = water.in(si::kilograms_of_water / si::kilograms);
        std::string columns = CaptureNumber::hex(water_kgkg);
#ifdef MOISTURE
        const MoistColumn& column = run_.atmosphere().moist_air()->column();
        const double pressure = column.pressure(z).in(au::pascals);
        const double theta_v = si::in_kelvin(column.theta_v(z));
        const std::string pressure_text = CaptureNumber::hex(pressure);
        const std::string theta_v_text = CaptureNumber::hex(theta_v);
        columns += "," + pressure_text + "," + theta_v_text;
#else
        (void)z;
        const std::string zero = CaptureNumber::hex(0.0);
        columns += "," + zero + "," + zero;
#endif // MOISTURE
        return columns;
    }

    /// @brief Writes patch.csv: the header "x,y", then the indices (from 0) of each floor cell whose centre is inside
    /// the patch, y in the outer loop and x in the inner loop (the heat sources' order). Only the header without a
    /// patch.
    /// @throws SetupError when the file cannot be opened or written
    void write_patch() const {
        std::ofstream out = open("patch.csv");
        out << "x,y\n";
        const CblDescription& d = run_.description();
        const AtmosphereGrid g = d.grid();
        const HeatedPatch& patch = d.surface().patch;
        const si::Length cell = d.cell();
        for(std::uint32_t y = 0u; y < g.ny(); y++) {
            write_patch_row(out, y, g, patch, cell);
        }
        close(out, "patch.csv");
    }

    /// @brief Writes the patch.csv lines of one row of floor cells: "x,y" for each cell whose centre is inside the
    /// patch, x from 0.
    /// @param out the open patch.csv
    /// @param y the row's index along y, from 0
    /// @param g the box's cells
    /// @param patch the heated patch
    /// @param cell the edge of one cubic cell
    static void write_patch_row(std::ofstream& out, std::uint32_t y, const AtmosphereGrid& g, const HeatedPatch& patch, si::Length cell) {
        const std::uint32_t nx{ g.nx() };
        const std::uint32_t ny{ g.ny() };
        for(std::uint32_t x = 0u; x < nx; x++) {
            const bool covered = patch.covers(x, y, nx, ny, cell);
            if(covered) {
                out << x << "," << y << "\n";
            }
        }
    }

    /// @brief Repeats the start noise's sequence over the fluid cells and keeps each new draw. A cell above the
    /// noise's top gets no draw. StartNoise::blocks() uses no draws (its values come from a hash of the block), so
    /// with block noise the list is empty.
    /// @return the draws in K, in the start builder's order: z from the first fluid level, then y, then x
    std::vector<double> noise_draws() const {
        const CblDescription& d = run_.description();
        const AtmosphereGrid g = d.grid();
        const LevelGrid& levels = g.levels();
        StartNoise::Sequence sequence = d.noise().sequence(d.cell());
        std::vector<double> draws{};
        for(std::uint32_t z = levels.first_fluid(); z <= levels.last_fluid(); z++) {
            add_level_draws(sequence, g, z, draws);
        }
        return draws;
    }

    /// @brief Repeats the start noise's sequence over one fluid level (y, then x) and keeps each new draw.
    /// @param sequence the start noise's sequence, at the level's first cell
    /// @param g the box's cells
    /// @param z the level, from 0 at the floor cell
    /// @param draws the draws in K so far; the level's new draws are added at its end
    static void add_level_draws(StartNoise::Sequence& sequence, const AtmosphereGrid& g, std::uint32_t z, std::vector<double>& draws) {
        const std::uint32_t ny{ g.ny() };
        for(std::uint32_t y = 0u; y < ny; y++) {
            add_row_draws(sequence, g, y, z, draws);
        }
    }

    /// @brief Repeats the start noise's sequence over one row of a level (x from 0) and keeps each new draw.
    /// @param sequence the start noise's sequence, at the row's first cell
    /// @param g the box's cells
    /// @param y the row's index along y, from 0
    /// @param z the level, from 0 at the floor cell
    /// @param draws the draws in K so far; the row's new draws are added at its end
    static void add_row_draws(StartNoise::Sequence& sequence, const AtmosphereGrid& g, std::uint32_t y, std::uint32_t z, std::vector<double>& draws) {
        const std::uint32_t nx{ g.nx() };
        for(std::uint32_t x = 0u; x < nx; x++) {
            const std::size_t before = sequence.draws();
            const CellCentre centre = g.centre(x, y, z);
            const si::TemperatureDifference v = sequence.at(centre);
            const std::size_t after = sequence.draws();
            const bool drawn{ after > before };
            if(drawn) {
                const double kelvins = v.in(au::kelvins);
                draws.push_back(kelvins);
            }
        }
    }

    /// @brief Writes noise.csv (the header "draw,value", then "index,hex float in K" for the first written_draws draws
    /// and the last one, indices from 0) and noise_hash.txt ("count hash": the number of draws and their FNV-1a hash
    /// as 16 hex digits).
    /// @throws SetupError when a file cannot be opened or written
    void write_noise() const {
        const std::vector<double> draws = noise_draws();
        std::ofstream out = open("noise.csv");
        out << "draw,value\n";
        for(std::size_t i = 0u; i < draws.size(); i++) {
            const bool first_draws = i < written_draws;
            const bool last_draw = i + 1u == draws.size();
            if(first_draws || last_draw) {
                const double draw = draws[i];
                const std::string value = CaptureNumber::hex(draw);
                out << i << "," << value << "\n";
            }
        }
        close(out, "noise.csv");
        const std::uint64_t h = noise_hash(draws);
        const std::size_t count = draws.size();
        const unsigned long long hash_bits = static_cast<unsigned long long>(h);
        std::ofstream sum = open("noise_hash.txt");
        constexpr std::size_t line_capacity{ 64u }; // a count, a space, 16 hex digits and a newline
        char t[line_capacity]{};
        const int length = std::snprintf(t, sizeof(t), "%zu %016llx\n", count, hash_bits);
        CaptureNumber::check_printed(length, sizeof(t), "the noise hash");
        sum << t;
        close(sum, "noise_hash.txt");
    }

    /// @brief Hashes the draws with 64-bit FNV-1a over the bytes of each draw as a float, in their order (the draws
    /// are floats in StartNoise, so the float is exact).
    /// @param draws the start noise's draws in K
    /// @return the hash
    static std::uint64_t noise_hash(const std::vector<double>& draws) {
        constexpr std::uint64_t offset = 14695981039346656037ull; // FNV-1a
        constexpr std::uint64_t prime = 1099511628211ull;         // FNV-1a
        std::uint64_t h{ offset };
        for(const double d : draws) {
            const float v = static_cast<float>(d); // the draws are floats (StartNoise)
            const std::array<unsigned char, sizeof(float)> b = std::bit_cast<std::array<unsigned char, sizeof(float)>>(v); // the bytes in memory order
            for(std::size_t i = 0u; i < b.size(); i++) {
                const unsigned char byte_value = b[i];
                h ^= byte_value;
                h *= prime;
            }
        }
        return h;
    }

    /// @brief Writes one field of all cells as float32, in the core's cell order n = x + (y + z Ny) Nx.
    ///
    /// The caller checks the stream after the last array (write_fields()).
    /// @param out the open .bin file
    /// @param values the field's lattice value of each cell, by the cell index n
    static void write_array(std::ofstream& out, const std::vector<float>& values) {
        const char* bytes = reinterpret_cast<const char*>(values.data());
        const std::size_t byte_count{ values.size() * sizeof(float) };
        const std::streamsize size{ static_cast<std::streamsize>(byte_count) };
        out.write(bytes, size);
    }

    /// @brief Writes the raw lattice fields of an interval: the .bin file holds the arrays u.x, u.y, u.z and T (and Q in
    /// a moist run) one after the other, each as float32 in the host's byte order (no header), in the core's cell order
    /// n = x + (y + z Ny) Nx, in lattice units. The .txt file holds the layout: the lines "Nx", "Ny", "Nz", "arrays", "order", and "speed" (the speed
    /// of one cell per time step in m/s, as "hex decimal").
    /// @param s the interval, valid during the call
    /// @param stem the files' name without the extension ("fields" or "fields_last")
    /// @throws SetupError when a file cannot be opened or written
    void write_fields(const IntervalSnapshot& s, const std::string& stem) const {
        const AtmosphereGrid& g = s.fields.grid();
        const std::uint32_t nx = g.nx();
        const std::uint32_t ny = g.ny();
        const std::uint32_t nz = g.levels().levels();
        const ulong nx_wide{ static_cast<ulong>(nx) };
        const ulong ny_wide{ static_cast<ulong>(ny) };
        const ulong nz_wide{ static_cast<ulong>(nz) };
        const ulong cells{ nx_wide * ny_wide * nz_wide };
        std::ofstream out = open(stem + ".bin", true);
        std::vector<float> buffer(cells);
        const AtmosphereFields& fields = s.fields;
        for(ulong n = 0u; n < cells; n++) {
            buffer[n] = fields.lattice_ux(n);
        }
        write_array(out, buffer);
        for(ulong n = 0u; n < cells; n++) {
            buffer[n] = fields.lattice_uy(n);
        }
        write_array(out, buffer);
        for(ulong n = 0u; n < cells; n++) {
            buffer[n] = fields.lattice_uz(n);
        }
        write_array(out, buffer);
        for(ulong n = 0u; n < cells; n++) {
            buffer[n] = fields.lattice_t(n);
        }
        write_array(out, buffer);
#ifdef MOISTURE
        if(fields.moist()) {
            for(ulong n = 0u; n < cells; n++) {
                buffer[n] = fields.lattice_q(n);
            }
            write_array(out, buffer);
        }
#endif // MOISTURE
        close(out, stem + ".bin");
        std::ofstream layout = open(stem + ".txt");
        const char* q_array = fields.moist() ? " Q" : "";
        const double speed = fields.converter().lattice_speed().in(au::meters / au::second);
        const std::string speed_text = CaptureNumber::exact(speed);
        layout << "Nx " << nx << "\nNy " << ny << "\nNz " << nz << "\narrays u.x u.y u.z T" << q_array << "\norder n = x + (y + z Ny) Nx, float32\nspeed "
               << speed_text << "\n";
        close(layout, stem + ".txt");
    }

    /// @brief Computes the nudging forces that the interval's layer means give (a nudging made from the description
    /// like the run's own, so the run's nudging is not changed).
    /// @param s the interval, valid during the call
    /// @return the lattice forces along x and y per level; 0 at every level without wind
    WindNudging::Forces nudging_forces(const IntervalSnapshot& s) const {
        const LevelGrid& levels = s.fields.grid().levels();
        if(nudging_ != nullptr) {
            const WindNudging::Forces forces = nudging_->forces(s.profiles.means, levels, s.fields.converter());
            return forces;
        }
        const std::uint32_t nz = levels.levels();
        const WindNudging::Forces none{ .fx = std::vector<float>(nz, 0.0f), .fy = std::vector<float>(nz, 0.0f) };
        return none;
    }

    /// @brief Writes an interval's file: the line "# time_s hex decimal zi_m hex decimal" (the interval's end and its
    /// inversion height), the header, then one row per fluid level: z (the level index, a decimal integer), then the
    /// hex float columns theta (theta - theta_0 in K), w_var (m²/s²), w_skew (a ratio), theta_var (K²), w_theta (K m/s), u, v, w_max,
    /// w_min (m/s), nudge_fx, nudge_fy (lattice forces), qt, ql (kg/kg) and cloud_fraction (a ratio; the clouds are 0
    /// in a dry run).
    /// @param s the interval, valid during the call
    /// @param name the file's name without ".csv"
    /// @throws SetupError when the file cannot be opened or written
    void write_interval(const IntervalSnapshot& s, const std::string& name) const {
        std::ofstream out = open(name + ".csv");
        const double time_s = s.time.in(au::seconds);
        const double zi_m = s.inversion.in(au::meters);
        const std::string time_text = CaptureNumber::exact(time_s);
        const std::string zi_text = CaptureNumber::exact(zi_m);
        out << "# time_s " << time_text << " zi_m " << zi_text << "\n";
        out << "z,theta,w_var,w_skew,theta_var,w_theta,u,v,w_max,w_min,nudge_fx,nudge_fy,qt,ql,cloud_fraction\n";
        const LevelGrid& levels = s.fields.grid().levels();
        const WindNudging::Forces f = nudging_forces(s);
        for(std::uint32_t z = levels.first_fluid(); z <= levels.last_fluid(); z++) {
            const std::string means = means_columns(s.profiles.means[z]);
            const float fx_value = f.fx[z];
            const float fy_value = f.fy[z];
            const std::string fx = CaptureNumber::hex(fx_value);
            const std::string fy = CaptureNumber::hex(fy_value);
            const std::string clouds = cloud_columns(s, z);
            out << z << "," << means << "," << fx << "," << fy << "," << clouds << "\n";
        }
        close(out, name + ".csv");
    }

    /// @brief Makes the means columns of one row of an interval's file.
    /// @param m the level's layer means
    /// @return theta, w_var, w_skew, theta_var, w_theta, u, v, w_max and w_min as hex floats in K, m²/s², a ratio,
    /// K², K m/s and m/s, joined by commas
    static std::string means_columns(const LayerMeans& m) {
        const auto mps = au::meters / au::second;
        const double theta_value = m.theta.in(au::kelvins);
        const double w_var_value = m.w_var.in(au::squared(mps));
        const double w_skew_value = m.w_skew;
        const double theta_var_value = m.theta_var.in(au::squared(au::kelvins));
        const double w_theta_value = m.w_theta.in(au::kelvins * mps);
        const double u_value = m.u.in(mps);
        const double v_value = m.v.in(mps);
        const double w_max_value = m.w_max.in(mps);
        const double w_min_value = m.w_min.in(mps);
        const std::string theta = CaptureNumber::hex(theta_value);
        const std::string w_var = CaptureNumber::hex(w_var_value);
        const std::string w_skew = CaptureNumber::hex(w_skew_value);
        const std::string theta_var = CaptureNumber::hex(theta_var_value);
        const std::string w_theta = CaptureNumber::hex(w_theta_value);
        const std::string u = CaptureNumber::hex(u_value);
        const std::string v = CaptureNumber::hex(v_value);
        const std::string w_max = CaptureNumber::hex(w_max_value);
        const std::string w_min = CaptureNumber::hex(w_min_value);
        const std::string columns = theta + "," + w_var + "," + w_skew + "," + theta_var + "," + w_theta + "," + u + "," + v + "," + w_max + "," + w_min;
        return columns;
    }

    /// @brief Makes the cloud columns of one row of an interval's file.
    /// @param s the interval, valid during the call
    /// @param z the level's index, from 0 (the floor)
    /// @return qt and ql (the level's mean total and liquid water in kg/kg) and cloud_fraction (the share of its cells
    /// with liquid water) as hex floats, joined by commas; all 0 when the interval has no cloud levels (a dry run)
    static std::string cloud_columns(const IntervalSnapshot& s, std::uint32_t z) {
        if(!s.profiles.clouds.has_value()) {
            const std::string zero = CaptureNumber::hex(0.0);
            const std::string columns = zero + "," + zero + "," + zero;
            return columns;
        }
        const CloudLevels& c = *s.profiles.clouds;
        const auto kgpkg = si::kilograms_of_water / si::kilograms;
        const double qt_value = c.qt[z].in(kgpkg);
        const double ql_value = c.ql[z].in(kgpkg);
        const double fraction_value = c.fraction[z];
        const std::string qt = CaptureNumber::hex(qt_value);
        const std::string ql = CaptureNumber::hex(ql_value);
        const std::string fraction = CaptureNumber::hex(fraction_value);
        const std::string columns = qt + "," + ql + "," + fraction;
        return columns;
    }

    std::filesystem::path folder_;              ///< the capture folder
    ConvectiveBoundaryLayer& run_;              ///< the captured run (not owned; it must outlive the capture)
    std::unique_ptr<WindNudging> nudging_{};    ///< a nudging made from the description like the run's own, for each interval's forces (none without wind)
    std::uint32_t seen_ = 0u;                   ///< the number of intervals written so far
};
