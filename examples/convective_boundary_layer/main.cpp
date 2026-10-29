/// @file main.cpp
/// @brief The convective boundary layer example: a layer of air heated from the floor, optionally with a heated patch
/// that anchors a thermal, built with the Setup API's atmosphere classes (lib/setup/atmosphere, SETUP_API.md
/// "Atmospheric convection").
///
/// Each case is a CblDescription in physical units. ConvectiveBoundaryLayer converts it to lattice units, builds the
/// LBM and runs it. This file only reads the command line and puts the cases together.
///
/// Command line (the order of the positional arguments is fixed; an option can be anywhere):
/// @verbatim
///     convective_boundary_layer_fv <output folder> quick [surface]
///     convective_boundary_layer_fv <output folder> sp2011 <cell m> <cells wide> [seed [domains [patch m [surface]]]]
///     convective_boundary_layer_fv <output folder> sounding <cell m> <cells wide> <profile.csv>
///                                  [seed [domains [lid in zi [sponge from zi [wind m/s at 100 m [from deg]]]]]]
///     options: --diffusivity <lattice units> --zi <m> --moist --spinup <s> --record <s> --estimate --capture
/// @endverbatim
///
/// The cases:
/// - quick: a 320 m square box with 2.5 m cells for 100 s of simulated time. Use it to check that the outputs are
///   written.
/// - sp2011: the dry convective boundary layer of Sullivan and Patton 2011 (J. Atmos. Sci. 68): a floor heat flux of
///   0.24 K m/s, 300 K up to 974 m, +8 K over the next 100 m, 3 K/km above, the lid at 2048 m.
/// - sounding: a tabulated sounding from a CSV file (columns z_m,thl_K, and qt_kgkg for a moist run). theta_0 is the
///   median of the profile between 0.2 zi and 0.8 zi. The floor heat flux is 0.262 K m/s, with a 400 m patch at the
///   core's 4x rate in the surface layer. The lid is at 2 zi, and the sponge starts at 1.6 zi (time scale 450 s at
///   the lid). With wind, the box is twice as long north, the run uses 2 domains along x, and the patch is 2000 m
///   north of the middle (upwind of a wind from the north).
///
/// The options:
/// - --diffusivity: the lattice diffusivity of heat and water, in place of the build's floor value.
/// - --zi: the inversion height of the sounding case (default 1864 m).
/// - --moist: a moist run (needs MOISTURE in the build and a qt_kgkg column). It adds the sounding's total water, a
///   floor moisture flux from the Bowen ratio 0.5, the moist buoyancy and the clouds. The profiles then have the
///   columns qt, ql and the cloud fraction, and stdout has the CLOUD lines and the RESULT CLOUD check.
/// - --spinup, --record: the spin-up time and the recorded time, in place of the case's times.
/// - --estimate: prints the cells, the device memory per domain and the time steps, then stops without a run.
/// - --capture: writes the stage capture (StageCapture) into the subfolder "capture" of the output folder. The golden data comes from it.
///
/// Outputs: profiles.csv in the output folder (the horizontal means of every interval, also of the spin-up), and the
/// SETUP, FINGERPRINT, PROGRESS, CLOUD, WIND and RESULT lines on stdout. T and Q move by the conservative link-flux
/// finite volumes (SCALAR_FV). The target convective_boundary_layer_fv stores the DDFs as FP32,
/// convective_boundary_layer_fv_fp16s stores them as FP16S. The scalars T and Q are FP32 in both targets.

#include "setup/setup.hpp"
#include "setup/atmosphere/convective_boundary_layer.hpp"
#include "setup/atmosphere/profile_csv.hpp"
#include "setup/atmosphere/stage_capture.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace {

using namespace si::symbols;

/// @brief The parsed command line: the output folder, the case name, the case's positional arguments and the options.
///
/// The constructor removes every option (and its value) from the list. What stays is positional:
/// [0] the output folder, [1] the case name, [2...] the case's arguments. The accessors has(), number() and text()
/// take the position of a case argument counted from 0 after the case name, so position 0 is the first argument
/// after "quick", "sp2011" or "sounding". An optional argument is read where it is used: its default first, then
/// number() when has() says that the command line gives it.
///
/// The command line gives numbers without units. The unit of a positional argument comes from its position (see
/// usage()). The unit of an option comes from the option (--zi is in metres).
class CommandLine {
public:
    /// @brief Reads the options, removes them from the list, and checks that a case name is there.
    ///
    /// Ends the program with print_error() when an option's value is missing or not more than 0, when an unknown
    /// option ("--..." that is not listed in usage()) is there, or when the output folder or the case name is missing.
    /// @param arguments the program's arguments without the program name (the core's main_arguments)
    /// @throws std::invalid_argument from std::stod when an option's value does not start with a number
    explicit CommandLine(std::vector<std::string> arguments) : a_(std::move(arguments)) {
        const std::optional<double> diffusivity_value = take_value("--diffusivity");
        if(diffusivity_value) {
            diffusivity = si::LatticeDiffusivity(*diffusivity_value);
        }
        const std::optional<double> zi_value = take_value("--zi");
        if(zi_value) {
            zi = *zi_value * m;
        }
        const std::optional<double> spinup_value = take_value("--spinup");
        if(spinup_value) {
            spinup = *spinup_value * s;
        }
        const std::optional<double> record_value = take_value("--record");
        if(record_value) {
            record = *record_value * s;
        }
        moist = take_flag("--moist");
        estimate = take_flag("--estimate");
        capture = take_flag("--capture");
        reject_unknown_options();
        if(a_.size() <= name_at) {
            print_error(usage());
        }
    }

    /// @brief The usage text that print_error() shows for a wrong command line.
    /// @return the usage line: the three cases with their positional arguments, then the options
    static std::string usage() {
        return "convective_boundary_layer <output folder> [quick [surface] | sp2011 <cell m> <cells wide> [seed [domains [patch m [surface]]]] | "
               "sounding <cell m> <cells wide> <profile.csv> [seed [domains [lid zi [sponge from zi [wind m/s at 100 m [from deg]]]]]]] "
               "[--diffusivity <lattice units>] [--zi <m>] [--moist] [--spinup <s>] [--record <s>] [--estimate] [--capture]";
    }
    /// @brief The folder that receives profiles.csv and the capture (created by main_setup() when it is missing).
    /// @return the first positional argument
    const std::string& folder() const { return a_[folder_at]; }
    /// @brief The case to run: "quick", "sp2011" or "sounding" (Cases::from() checks the name).
    /// @return the second positional argument
    const std::string& name() const { return a_[name_at]; }
    /// @brief Tells whether the command line gives the case argument at position i.
    ///
    /// The case arguments are optional from the end: a later argument needs every earlier one. So has(i) is true
    /// when the command line has at least i + 1 arguments after the case name.
    /// @param i the position of a case argument, counted from 0 after the case name
    /// @return true when the argument is there
    bool has(std::size_t i) const { return a_.size() > i + first_argument_at; }
    /// @brief Reads the case argument at position i as a number. The caller attaches the unit.
    /// @param i the position of a case argument, counted from 0 after the case name
    /// @return the leading number of the argument (std::stod: "3.5" and "3.5x" both give 3.5)
    /// @throws std::out_of_range when the argument is not there (check with has() first), or the number does not fit
    /// a double
    /// @throws std::invalid_argument when the argument does not start with a number
    double number(std::size_t i) const { return std::stod(a_.at(i + first_argument_at)); }
    /// @brief Reads the case argument at position i as text (a file path, "surface", a seed).
    /// @param i the position of a case argument, counted from 0 after the case name
    /// @return the argument as given
    /// @throws std::out_of_range when the argument is not there (check with has() first)
    const std::string& text(std::size_t i) const { return a_.at(i + first_argument_at); }

    std::optional<si::LatticeDiffusivity> diffusivity{}; ///< --diffusivity: the lattice diffusivity of heat and water (cells² per time step), or none for the build's floor
    std::optional<si::Length> zi{};                      ///< --zi: the sounding case's inversion height at the start, or none for SoundingCase's 1864 m
    std::optional<si::Duration> spinup{};                ///< --spinup: the time before the first record, or none for the case's time
    std::optional<si::Duration> record{};                ///< --record: the recorded time after the spin-up, or none for the case's time
    bool moist = false;                                  ///< --moist: true for a moist run (water, moist buoyancy, clouds)
    bool estimate = false;                               ///< --estimate: true to print the run's cost and stop before the build
    bool capture = false;                                ///< --capture: true to write the stage capture

private:
    static constexpr std::size_t folder_at = 0u;          ///< the position of the output folder in the positional list
    static constexpr std::size_t name_at = 1u;            ///< the position of the case name in the positional list
    static constexpr std::size_t first_argument_at = 2u;  ///< the position of the first case argument in the positional list
    static constexpr std::size_t value_option_words = 2u; ///< the list entries of an option with a value: the name and the value

    /// @brief Finds the option name with its value, removes both from the list, and returns the value.
    ///
    /// Ends the program with print_error() when the value is missing or not more than 0.
    /// @param name the option, for example "--zi"
    /// @return the value, more than 0; none when the option is not there. Only the first occurrence is read.
    /// @throws std::invalid_argument from std::stod when the value does not start with a number
    std::optional<double> take_value(const std::string& name) {
        for(std::size_t k = 0u; k < a_.size(); k++) {
            if(a_[k] != name) {
                continue;
            }
            const std::size_t value_at = k + 1u;
            if(value_at >= a_.size()) {
                print_error(name + " needs a value");
            }
            const double value = std::stod(a_[value_at]);
            if(!(value > 0.0)) {
                print_error(name + " must be more than 0");
            }
            const std::ptrdiff_t first_at = static_cast<std::ptrdiff_t>(k);
            const std::ptrdiff_t last_at = static_cast<std::ptrdiff_t>(k + value_option_words);
            const std::vector<std::string>::iterator first = a_.begin() + first_at;
            const std::vector<std::string>::iterator last = a_.begin() + last_at;
            a_.erase(first, last);
            return value;
        }
        return std::nullopt;
    }
    /// @brief Finds the option name (an option without a value) and removes it from the list.
    /// @param name the option, for example "--moist"
    /// @return true when the option was there. Only the first occurrence is removed, so a second one is reported
    /// as unknown by reject_unknown_options().
    bool take_flag(const std::string& name) {
        for(std::size_t k = 0u; k < a_.size(); k++) {
            if(a_[k] == name) {
                const std::ptrdiff_t flag_at = static_cast<std::ptrdiff_t>(k);
                const std::vector<std::string>::iterator at = a_.begin() + flag_at;
                a_.erase(at);
                return true;
            }
        }
        return false;
    }
    /// @brief Ends the program with print_error() and the usage line when an entry that starts with "--" is left in
    /// the list after the known options were removed. A negative number ("-5") is not an option.
    void reject_unknown_options() const {
        for(const std::string& a : a_) {
            if(a.rfind("--", 0u) == 0u) { // not a positional argument
                const std::string message = "unknown option " + a + " (" + usage() + ")";
                print_error(message);
            }
        }
    }

    std::vector<std::string> a_{}; ///< the positional arguments: the program's arguments with the options removed
};

/// @brief The parts that more than one case uses: the start noise, the heated patch, the seed and the domains, with
/// their shared constants.
class CaseParts {
public:
    static constexpr si::TemperatureDifference noise_amplitude = au::kelvins(0.1); ///< the largest theta perturbation of the start noise
    static constexpr si::Length noise_top = au::meters(300.0);                     ///< the start noise's top for sp2011 and sounding: cells below it get a perturbation
    static constexpr si::Length centred = au::meters(0.0);                         ///< the patch offset north of the box's middle for a centred patch
    static constexpr double bowen_ratio = 0.5;                                      ///< the Bowen ratio (sensible over latent heat flux, a ratio) that gives a moist run's floor moisture flux
    static constexpr std::uint32_t one_domain = 1u;                                 ///< the number of domains along y and z (the cases split only along x)

    /// @brief Tells whether the case argument at position at is the word "surface" (the patch heats the surface
    /// layer at the core's 4x rate).
    /// @param c the command line
    /// @param at the position of the case argument, counted from 0 after the case name
    /// @return true when the argument is there and is "surface"; false when it is missing or another word
    static bool is_surface(const CommandLine& c, std::size_t at) {
        if(!c.has(at)) {
            return false;
        }
        const bool surface = c.text(at) == "surface";
        return surface;
    }

    /// @brief Makes the round heated patch of a case.
    /// @param diameter the patch's diameter, more than 0
    /// @param north the distance of the patch's centre north of the box's middle (negative: south)
    /// @param surface true: PatchMode::SURFACE_LAYER (the floor's fluid cells under the patch heat at the core's 4x
    /// rate); false: PatchMode::HEATED_LAYERS (HeatedPatch::default_layers layers, each at the floor's rate)
    /// @return the patch
    /// @throws SetupError from HeatedPatch::disc() when the diameter is not more than 0
    static HeatedPatch patch(si::Length diameter, si::Length north, bool surface) {
        const PatchMode mode = surface ? PatchMode::SURFACE_LAYER : PatchMode::HEATED_LAYERS;
        const HeatedPatch result = HeatedPatch::disc(diameter, north, mode);
        return result;
    }

    /// @brief Applies the optional seed and domain arguments of a case: a seed replaces the start noise with a
    /// per-cell noise of that seed (amplitude noise_amplitude, below noise_top); a domain count splits the box along x.
    /// @param d the case, before the two arguments
    /// @param c the command line
    /// @param seed_at the position of the seed argument, counted from 0 after the case name
    /// @param domains_at the position of the argument that gives the number of domains along x
    /// @return a copy of d with the seed's start noise and the domains, each only when the command line gives it
    /// @throws std::invalid_argument from std::stoull when the seed or the domain count is not a number
    /// @throws std::out_of_range from std::stoull when the number does not fit 64 bits
    /// @throws SetupError when the seed or the domain count does not fit 32 bits
    static CblDescription with_seed_and_domains(CblDescription d, const CommandLine& c, std::size_t seed_at, std::size_t domains_at) {
        if(c.has(seed_at)) {
            const std::uint32_t seed = whole_32(c.text(seed_at), "the seed");
            const StartNoise noise = StartNoise::per_cell(noise_amplitude, noise_top, seed);
            d = d.with_noise(noise);
        }
        if(c.has(domains_at)) {
            const std::uint32_t domains_x = whole_32(c.text(domains_at), "the domain count");
            const uint3 domains = uint3(domains_x, one_domain, one_domain);
            d = d.with_domains(domains);
        }
        return d;
    }

private:
    /// @brief Reads a whole number of the command line that must fit 32 bits.
    /// @param text the argument, for example "7" (std::stoull reads its leading number: "7x" gives 7)
    /// @param what the argument's name for the error message, for example "the seed"
    /// @return the number
    /// @throws std::invalid_argument from std::stoull when the text does not start with a number
    /// @throws std::out_of_range from std::stoull when the number does not fit 64 bits
    /// @throws SetupError that names the argument and its text when the number does not fit 32 bits
    static std::uint32_t whole_32(const std::string& text, const std::string& what) {
        const unsigned long long value = std::stoull(text);
        const unsigned long long largest = std::numeric_limits<std::uint32_t>::max();
        if(value > largest) {
            throw SetupError(what + " " + text + " does not fit 32 bits");
        }
        const std::uint32_t value_32 = static_cast<std::uint32_t>(value);
        return value_32;
    }
};

/// @brief The quick case: a small, short run to check that the outputs are written.
///
/// A 320 m square box with 2.5 m cells and the lid at 245 m. The start: 300 K up to 180 m, +1 K at 180 m, 5 K/km
/// above. A floor heat flux of 0.262 K m/s with a 40 m patch in the middle. 60 s of spin-up and 40 s of records, one
/// record every 10 s, with a fixed time step of 0.04 s.
class QuickCase {
public:
    static constexpr std::size_t surface_at = 0u; ///< the position of the optional word "surface", counted from 0 after the case name

    /// @brief Makes the quick case.
    /// @param surface true: the patch heats the surface layer at the core's 4x rate; false: it heats 4 layers
    /// (HeatedPatch::default_layers), each at the floor's rate
    /// @return the case
    static CblDescription of(bool surface) {
        CblNumerics numerics{};
        numerics.time_step = TimeStepRule::fixed(time_step);
        CblDescription d = CblDescription();
        d = d.with_numerics(numerics);
        const IdealizedProfile profile = IdealizedProfile{ .inversion_base = inversion, .jump = jump, .jump_depth = jump_depth, .lapse = lapse };
        const Sounding sounding = Sounding::idealized(mixed_theta, profile);
        d = d.with_sounding(sounding, inversion);
        d = d.with_cell(cell);
        const LidRule lid_rule = LidRule::at(lid);
        const ColumnExtent extent = ColumnExtent{ .east = box, .north = box, .lid = lid_rule };
        d = d.with_extent(extent);
        const HeatedPatch patch = CaseParts::patch(patch_diameter, CaseParts::centred, surface);
        const SurfaceForcing forcing = SurfaceForcing{ .flux = flux, .patch = patch, .bowen = CaseParts::bowen_ratio };
        d = d.with_surface(forcing);
        const StartNoise noise = StartNoise::per_cell(CaseParts::noise_amplitude, noise_top, seed);
        d = d.with_noise(noise);
        const RunSchedule schedule = RunSchedule{ .spinup = spinup, .record = record, .interval = interval };
        d = d.with_schedule(schedule);
        return d;
    }

private:
    static constexpr si::Duration time_step = au::seconds(0.04);                          ///< the time step at every cell size (TimeStepRule::fixed)
    static constexpr si::Temperature mixed_theta = au::kelvins_pt(300.0);                 ///< the potential temperature of the mixed layer (a temperature reading)
    static constexpr si::Length inversion = au::meters(180.0);                            ///< zi: the inversion height at the start
    static constexpr si::TemperatureDifference jump = au::kelvins(1.0);                   ///< the potential temperature step at the inversion
    static constexpr si::Length jump_depth = au::meters(0.0);                             ///< the depth of the jump: 0 m is a sharp step
    static constexpr si::LapseRate lapse = (au::kelvins / au::meter)(0.005);              ///< the potential temperature gradient above the inversion (5 K/km)
    static constexpr si::Length cell = au::meters(2.5);                                   ///< the cell size
    static constexpr si::Length box = au::meters(320.0);                                  ///< the box's extent west to east and south to north
    static constexpr si::Length lid = au::meters(245.0);                                  ///< the lid's height above the floor
    static constexpr si::KinematicHeatFlux flux = (au::kelvins * au::meters / au::second)(0.262); ///< the floor's kinematic heat flux
    static constexpr si::Length patch_diameter = au::meters(40.0);                        ///< the heated patch's diameter
    static constexpr si::Length noise_top = au::meters(30.0);                             ///< the start noise's top: cells below it get a perturbation
    static constexpr unsigned seed = 2u;                                                  ///< the start noise's random seed
    static constexpr si::Duration spinup = au::seconds(60.0);                             ///< the time before the first record
    static constexpr si::Duration record = au::seconds(40.0);                             ///< the recorded time after the spin-up
    static constexpr si::Duration interval = au::seconds(10.0);                           ///< the time between two records (and two rows of profiles.csv)
};

/// @brief The Sullivan and Patton 2011 case: CblDescription's defaults, with the cell size and the box width from
/// the command line.
///
/// Case arguments, by position after the case name: 0 the cell size in m, 1 the box width in cells, then optional
/// 2 the noise seed, 3 the domains along x, 4 a centred heated patch's diameter in m, 5 the word "surface" (the
/// patch heats the surface layer at the core's 4x rate). The box is square and the lid is at 2048 m.
class Sp2011Case {
public:
    /// @brief Tells whether the command line gives the case's two required arguments (the cell size and the width).
    /// @param c the command line
    /// @return true when the command line has at least two case arguments
    static bool given(const CommandLine& c) { return c.has(cells_wide_at); }

    /// @brief Makes the Sullivan and Patton 2011 case from its command line arguments.
    /// @param c the command line, with at least the two required case arguments (check with given())
    /// @return the case
    /// @throws std::invalid_argument when a numeric argument is not a number
    /// @throws SetupError when the cell size or the patch diameter is not more than 0, or the seed or the domain count
    /// does not fit 32 bits
    static CblDescription of(const CommandLine& c) {
        const si::Length cell = c.number(cell_at) * m;
        const double cells_wide = c.number(cells_wide_at);
        const si::Length wide = cell * cells_wide;
        CblDescription d = CblDescription();
        d = d.with_cell(cell);
        const LidRule lid_rule = LidRule::at(lid);
        const ColumnExtent extent = ColumnExtent{ .east = wide, .north = wide, .lid = lid_rule };
        d = d.with_extent(extent);
        d = CaseParts::with_seed_and_domains(d, c, seed_at, domains_at);
        if(c.has(patch_at)) {
            const si::Length diameter = c.number(patch_at) * m;
            const bool surface = CaseParts::is_surface(c, surface_at);
            const HeatedPatch patch = CaseParts::patch(diameter, CaseParts::centred, surface);
            const si::KinematicHeatFlux flux = d.surface().flux;
            const SurfaceForcing forcing = SurfaceForcing{ .flux = flux, .patch = patch, .bowen = CaseParts::bowen_ratio };
            d = d.with_surface(forcing);
        }
        return d;
    }

private:
    static constexpr std::size_t cell_at = 0u;       ///< the position of the cell size in m (required), counted from 0 after the case name
    static constexpr std::size_t cells_wide_at = 1u; ///< the position of the box width in cells (required)
    static constexpr std::size_t seed_at = 2u;       ///< the position of the start noise's seed (optional)
    static constexpr std::size_t domains_at = 3u;    ///< the position of the number of domains along x (optional)
    static constexpr std::size_t patch_at = 4u;      ///< the position of the centred patch's diameter in m (optional)
    static constexpr std::size_t surface_at = 5u;    ///< the position of the word "surface" (optional)
    static constexpr si::Length lid = au::meters(2048.0); ///< the lid's height above the floor (Sullivan and Patton 2011)
};

/// @brief The tabulated sounding case: a measured or modelled profile from a CSV file.
///
/// Case arguments, by position after the case name: 0 the cell size in m, 1 the box width in cells, 2 the profile's
/// CSV file, then optional 3 the noise seed, 4 the domains along x, 5 the lid's height over zi, 6 the sponge's start
/// over zi, 7 the wind speed at 100 m in m/s, 8 the direction the wind comes from in degrees. The floor heat flux is 0.262 K m/s, with a 400 m patch at the core's 4x rate in the surface
/// layer. The lid and the sponge's start are multiples of zi (default 2 and 1.6). A wind speed above 0 holds a power
/// law wind by nudging: the box is then twice as long north, the run uses 2 domains along x, and the patch is
/// 2000 m north of the middle.
class SoundingCase {
public:
    /// @brief Tells whether the command line gives the case's three required arguments (cell size, width, profile).
    /// @param c the command line
    /// @return true when the command line has at least three case arguments
    static bool given(const CommandLine& c) { return c.has(profile_at); }

    /// @brief Makes the sounding case from its command line arguments.
    /// @param c the command line, with at least the three required case arguments (check with given())
    /// @return the case
    /// @throws std::invalid_argument when a numeric argument is not a number
    /// @throws SetupError when the profile file cannot be read (Sounding::from_csv()), when no profile point is
    /// between 0.2 zi and 0.8 zi, when the cell size is not more than 0, when the lid is not above zi (lid in zi
    /// not more than 1), or when the seed or the domain count does not fit 32 bits
    static CblDescription of(const CommandLine& c) {
        const Arguments a = Arguments::of(c);
        const bool windy = a.wind > au::ZERO;
        const si::Length north = north_extent(a.wide, windy);
        const si::Length patch_north = windy ? upwind_patch_north : CaseParts::centred;
        const HeatedPatch patch = HeatedPatch::disc(patch_diameter, patch_north, PatchMode::SURFACE_LAYER);
        const SpongeProfile sponge = sponge_of(a);
        CblDescription d = CblDescription();
        d = d.with_cell(a.cell);
        const Sounding tabulated = Sounding::from_csv(c.text(profile_at));
        const Sounding sounding = tabulated.with_reference_from_mixed_layer(a.zi);
        d = d.with_sounding(sounding, a.zi);
        const LidRule lid_rule = LidRule::in_inversion_heights(a.lid_in_zi);
        const ColumnExtent extent = ColumnExtent{ .east = a.wide, .north = north, .lid = lid_rule };
        d = d.with_extent(extent);
        const SurfaceForcing forcing = SurfaceForcing{ .flux = flux, .patch = patch, .bowen = CaseParts::bowen_ratio };
        d = d.with_surface(forcing);
        d = d.with_sponge(sponge);
        const RunSchedule schedule = RunSchedule{ .spinup = spinup, .record = record, .interval = interval };
        d = d.with_schedule(schedule);
        d = CaseParts::with_seed_and_domains(d, c, seed_at, domains_at);
        if(windy) {
            // with wind: 2 domains along x, in place of a domains argument (the box twice as long north holds about
            // twice the cells)
            const PowerLawWind wind(a.wind, wind_height, wind_exponent, a.from);
            const WindForcing forcing_wind = WindForcing{ .wind = wind, .nudge_tau = nudge_tau };
            d = d.with_wind(forcing_wind);
            const uint3 domains = uint3(windy_domains_x, CaseParts::one_domain, CaseParts::one_domain);
            d = d.with_domains(domains);
        }
        return d;
    }

private:
    static constexpr std::size_t cell_at = 0u;       ///< the position of the cell size in m (required), counted from 0 after the case name
    static constexpr std::size_t cells_wide_at = 1u; ///< the position of the box width in cells (required)
    static constexpr std::size_t profile_at = 2u;    ///< the position of the profile's CSV file (required)
    static constexpr std::size_t seed_at = 3u;       ///< the position of the start noise's seed (optional)
    static constexpr std::size_t domains_at = 4u;    ///< the position of the number of domains along x (optional)
    static constexpr std::size_t lid_at = 5u;        ///< the position of the lid's height over zi (optional)
    static constexpr std::size_t sponge_at = 6u;     ///< the position of the sponge's start over zi (optional)
    static constexpr std::size_t wind_at = 7u;       ///< the position of the wind speed at wind_height in m/s (optional)
    static constexpr std::size_t from_at = 8u;       ///< the position of the direction the wind comes from in degrees (optional)
    static constexpr si::Length default_zi = au::meters(1864.0);                      ///< zi when the command line has no --zi
    static constexpr double default_lid_in_zi = 2.0;                                  ///< the lid's height over zi (a ratio) when the command line does not give it
    static constexpr double default_sponge_in_zi = 1.6;                               ///< the sponge's start over zi (a ratio; 0 or less: no sponge) when the command line does not give it
    static constexpr si::Speed default_wind = (au::meters / au::second)(0.0);         ///< the wind speed at wind_height when the command line does not give it: no wind
    static constexpr si::Angle default_from = au::degrees(0.0);                       ///< the direction the wind comes from when the command line does not give it: north (PowerLawWind's convention)
    static constexpr double windy_north_factor = 2.0;                                 ///< the box's length north over its width (a ratio), with wind
    static constexpr std::uint32_t windy_domains_x = 2u;                              ///< the number of domains along x, with wind
    static constexpr si::KinematicHeatFlux flux = (au::kelvins * au::meters / au::second)(0.2618979); ///< the floor's kinematic heat flux
    static constexpr si::Length patch_diameter = au::meters(400.0);                   ///< the heated patch's diameter
    static constexpr si::Length upwind_patch_north = au::meters(2000.0);              ///< the patch centre's distance north of the box's middle, with wind
    static constexpr si::Duration sponge_tau = au::seconds(450.0);                    ///< the sponge's damping time scale at the lid
    static constexpr si::Length wind_height = au::meters(100.0);                      ///< the height at which the given wind speed applies
    static constexpr double wind_exponent = 0.25;                                     ///< the power law's exponent: u(z) = u(wind_height) (z / wind_height)^0.25
    static constexpr si::Duration nudge_tau = au::seconds(600.0);                     ///< the nudging's time scale: the layer-mean wind relaxes to the target over it
    static constexpr si::Duration spinup = au::seconds(900.0);                        ///< the time before the first record
    static constexpr si::Duration record = au::seconds(1800.0);                       ///< the recorded time after the spin-up
    static constexpr si::Duration interval = au::seconds(20.0);                       ///< the time between two records (and two rows of profiles.csv)

    /// @brief The case's numeric arguments in their units, each one from the command line or from its default.
    struct Arguments {
        si::Length cell{ au::ZERO };  ///< the cell size
        si::Length wide{ au::ZERO };  ///< the box's width west to east: the cell size times the cells wide
        si::Length zi{ au::ZERO };    ///< the inversion height at the start: --zi or default_zi
        double lid_in_zi{};           ///< the lid's height over zi (a ratio)
        double sponge_in_zi{};        ///< the sponge's start over zi (a ratio; 0 or less: no sponge)
        si::Speed wind{ au::ZERO };   ///< the wind speed at wind_height (0: no wind)
        si::Angle from{ au::ZERO };   ///< the direction the wind comes from, clockwise from north

        /// @brief Reads the case's numeric arguments and fills in the default of each optional one that is missing.
        /// @param c the command line, with at least the three required case arguments
        /// @return the arguments in their units
        /// @throws std::invalid_argument when a numeric argument is not a number
        static Arguments of(const CommandLine& c) {
            const si::Length cell = c.number(cell_at) * m;
            const double cells_wide = c.number(cells_wide_at);
            const si::Length wide = cell * cells_wide;
            si::Length zi{ default_zi };
            if(c.zi) {
                zi = *c.zi;
            }
            double lid_in_zi{ default_lid_in_zi };
            if(c.has(lid_at)) {
                lid_in_zi = c.number(lid_at);
            }
            double sponge_in_zi{ default_sponge_in_zi };
            if(c.has(sponge_at)) {
                sponge_in_zi = c.number(sponge_at);
            }
            si::Speed wind{ default_wind };
            if(c.has(wind_at)) {
                const double wind_mps{ c.number(wind_at) };
                wind = wind_mps * m / s;
            }
            si::Angle from{ default_from };
            if(c.has(from_at)) {
                const double from_deg{ c.number(from_at) };
                from = from_deg * deg;
            }
            return Arguments{ .cell = cell, .wide = wide, .zi = zi, .lid_in_zi = lid_in_zi, .sponge_in_zi = sponge_in_zi, .wind = wind, .from = from };
        }
    };

    /// @brief Gives the box's length south to north: longer with wind, so that a thermal from the upwind patch has
    /// room to drift downwind.
    /// @param wide the box's width west to east
    /// @param windy true when the case has wind
    /// @return windy_north_factor times wide with wind; wide without wind
    static si::Length north_extent(si::Length wide, bool windy) {
        si::Length north{ wide };
        if(windy) {
            north = wide * windy_north_factor;
        }
        return north;
    }

    /// @brief Makes the case's damping layer below the lid.
    /// @param a the case's arguments (zi, sponge_in_zi)
    /// @return a quadratic sponge from sponge_in_zi times zi up to the lid, with sponge_tau at the lid; no sponge when
    /// sponge_in_zi is 0 or less
    /// @throws SetupError from SpongeProfile::quadratic() for a start or a time scale that is not more than 0
    static SpongeProfile sponge_of(const Arguments& a) {
        if(!(a.sponge_in_zi > 0.0)) {
            return SpongeProfile::none();
        }
        const si::Length sponge_from = a.zi * a.sponge_in_zi;
        const SpongeProfile sponge = SpongeProfile::quadratic(sponge_from, sponge_tau);
        return sponge;
    }
};

/// @brief Selects the case that the command line names and applies the options to it.
class Cases {
public:
    /// @brief Makes the description to run: the named case, then --diffusivity, --spinup, --record and --moist.
    ///
    /// Ends the program with print_error() and the usage line for an unknown case name or missing required case
    /// arguments, when --zi is given to a case other than sounding, and when --moist is given without a qt_kgkg
    /// column in the profile.
    /// @param c the command line
    /// @return the case with the options applied (not yet validated: ConvectiveBoundaryLayer validates it)
    /// @throws std::invalid_argument when a numeric case argument is not a number
    /// @throws SetupError when a case argument is out of its range (see the case's of())
    static CblDescription from(const CommandLine& c) {
        const CblDescription d = named(c);
        const bool zi_given = c.zi.has_value();
        if(zi_given && c.name() != "sounding") {
            print_error("--zi applies to the sounding case only");
        }
        const CblDescription result = with_options(d, c);
        return result;
    }

private:
    /// @brief Makes the case that the case name selects.
    ///
    /// Ends the program with print_error() and the usage line for an unknown case name or missing required case
    /// arguments.
    /// @param c the command line
    /// @return the quick, sp2011 or sounding case
    /// @throws std::invalid_argument when a numeric case argument is not a number (from the case's of())
    /// @throws SetupError when a case argument is out of its range (from the case's of())
    static CblDescription named(const CommandLine& c) {
        const std::string& name = c.name();
        if(name == "quick") {
            const bool surface = CaseParts::is_surface(c, QuickCase::surface_at);
            return QuickCase::of(surface);
        }
        if(name == "sp2011" && Sp2011Case::given(c)) {
            return Sp2011Case::of(c);
        }
        if(name == "sounding" && SoundingCase::given(c)) {
            return SoundingCase::of(c);
        }
        print_error(CommandLine::usage());
        return CblDescription(); // not reached: print_error ends the program
    }

    /// @brief Applies the options that change a case: --diffusivity replaces the scalar diffusivity, --spinup and
    /// --record replace the schedule's times, --moist makes the run moist.
    ///
    /// Ends the program with print_error() when --moist is given and the sounding has no total water.
    /// @param d the case
    /// @param c the command line
    /// @return a copy of d with the options applied
    static CblDescription with_options(CblDescription d, const CommandLine& c) {
        if(c.diffusivity) {
            CblNumerics n = d.numerics();
            n.scalar = Diffusivity::lattice(*c.diffusivity);
            d = d.with_numerics(n);
        }
        RunSchedule schedule = d.schedule();
        if(c.spinup) {
            schedule.spinup = *c.spinup;
        }
        if(c.record) {
            schedule.record = *c.record;
        }
        d = d.with_schedule(schedule);
        if(!c.moist) {
            return d;
        }
        const bool has_water = d.sounding().has_water();
        if(!has_water) {
            print_error("--moist needs the sounding case with a qt_kgkg column in its profile");
        }
        const CblDescription moist = d.with_moisture(true);
        return moist;
    }
};

/// @brief The example's two ways to end: the cost of the case (--estimate), or the run.
class Example {
public:
    /// @brief Prints the ESTIMATE line of a valid case: the cells, the device memory per domain and the time steps.
    /// No device memory is used.
    /// @param d the case
    /// @throws SetupError when the case is not valid (CblDescription::validate()) or cannot be estimated
    static void estimate(const CblDescription& d) {
        d.validate();
        const LatticeMemory lattice = Simulation::build_lattice_memory();
        const RunEstimate estimate = d.estimate(lattice, std::nullopt);
        const std::string estimate_text = estimate.describe();
        std::printf("ESTIMATE %s\n", estimate_text.c_str());
    }

    /// @brief Builds and runs the case. The order: the SETUP line, the FINGERPRINT line (a hash of the whole set-up
    /// before the first step), the observers (profiles.csv, and the capture with --capture), then the run with its
    /// PROGRESS, CLOUD and WIND lines, and the RESULT line at the end.
    ///
    /// ConvectiveBoundaryLayer ends the program with print_error() when the case is not valid.
    /// @param c the command line: the output folder and --capture
    /// @param d the case
    /// @throws std::filesystem::filesystem_error when the output folder cannot be created
    /// @throws SetupError when a part of the case cannot be built (ConvectiveBoundaryLayer's constructor) or a file
    /// cannot be written, or when the SETUP line does not fit its buffer (setup_line())
    static void run(const CommandLine& c, const CblDescription& d) {
        const std::filesystem::path folder(c.folder());
        std::filesystem::create_directories(folder);
        ConvectiveBoundaryLayer run(d);
        const std::string setup = setup_line(run, d);
        std::printf("%s\n", setup.c_str());
        const unsigned long long fingerprint = static_cast<unsigned long long>(run.atmosphere().fingerprint());
        std::printf("FINGERPRINT setup %016llx\n", fingerprint);
        std::fflush(stdout);
        const std::filesystem::path profiles_path = folder / "profiles.csv";
        const bool moist = d.moist();
        ProfileCsv csv(profiles_path, moist);
        run.observe(csv);
        std::unique_ptr<StageCapture> capture{};
        if(c.capture) {
            const std::filesystem::path capture_folder = folder / "capture";
            capture = std::make_unique<StageCapture>(capture_folder, run);
            run.observe(*capture);
        }
        const RunSummary summary = run.run();
        const std::uint32_t intervals = summary.intervals;
        const std::uint32_t recorded = summary.recorded;
        const std::string& output_folder = c.folder();
        std::printf("RESULT %u intervals (%u recorded) into %s\n", intervals, recorded, output_folder.c_str());
    }

private:
    static constexpr std::size_t setup_line_capacity = 400u; ///< the SETUP line's buffer, in characters with the end zero

    /// @brief Makes the SETUP line on stdout, which states the lattice that the run uses.
    /// @param run the built run
    /// @param d the run's description
    /// @return the line: the cells (nx x ny x levels), the cell size, the time step, the lattice buoyancy factor beta,
    /// the floor's heat rate per time step (lattice units), the patch's floor cells, and the sounding's description
    /// @throws SetupError when the line does not fit setup_line_capacity (a long sounding description) or
    /// std::snprintf fails
    static std::string setup_line(ConvectiveBoundaryLayer& run, const CblDescription& d) {
        const LatticeParameters p = run.parameters();
        const AtmosphereGrid grid = d.grid();
        const std::uint32_t nx = grid.nx();
        const std::uint32_t ny = grid.ny();
        const std::uint32_t levels = grid.levels().levels();
        const double dx_m = p.dx.in(m);
        const double dt_s = p.dt.in(s);
        const double beta = static_cast<double>(p.beta);
        const double heat_rate = static_cast<double>(p.heat_rate);
        const unsigned long long patch_cells = static_cast<unsigned long long>(run.atmosphere().sources().patch_cells);
        const std::string sounding = d.sounding().describe();
        char text[setup_line_capacity]{};
        const int written = std::snprintf(text, sizeof(text), "SETUP %u x %u x %u cells of %.2f m, dt %.3f s, beta %.4f, heat rate %.4e per step, patch %llu cells; %s",
                                          nx, ny, levels, dx_m, dt_s, beta, heat_rate, patch_cells, sounding.c_str());
        const bool fits = written >= 0 && static_cast<std::size_t>(written) < sizeof(text);
        if(!fits) {
            throw SetupError("the SETUP line does not fit " + std::to_string(setup_line_capacity) + " characters (snprintf gave " + std::to_string(written) + ")");
        }
        const std::string line = text;
        return line;
    }
};

} // namespace

/// @brief The example's entry point, called by the core's main(): reads the command line, then prints the estimate
/// (--estimate) or builds and runs the case.
///
/// This is the program's boundary: every error of the set-up or the run (a number that is not a number, a value out
/// of its range, a file that cannot be written) ends the program with print_error() and its message, prefixed with
/// "convective_boundary_layer: ". Nothing is thrown out of this function.
void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE SUBGRID THERMAL_DIFFUSIVITY_FIX LEVEL_FORCING MOISTURE SCALAR_FV
    try {
        const CommandLine c(main_arguments);
        const CblDescription d = Cases::from(c);
        if(c.estimate) {
            Example::estimate(d);
        } else {
            Example::run(c, d);
        }
    } catch(const std::exception& error) { // the program's boundary: report the cause and end
        const std::string message = std::string("convective_boundary_layer: ") + error.what();
        print_error(message);
    }
}
