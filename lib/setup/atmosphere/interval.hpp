#pragma once

/// @file interval.hpp
/// @brief The middle layer's (L2) interval pipeline of an atmosphere run: the record of one interval, the observers
/// that read it and the checks that judge it. Header-side: it needs the build's defines and the core.

#include "setup/atmosphere/atmosphere_fields.hpp"
#include "setup/atmosphere/boundary_layer_diagnostics.hpp"
#include "setup/atmosphere/level_forcing.hpp"
#include "setup/atmosphere/run_checks.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/units/units.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#if !defined(TEMPERATURE) || !defined(LEVEL_FORCING)
#error "setup/atmosphere/interval.hpp needs TEMPERATURE and LEVEL_FORCING in the EXTENSIONS"
#endif // TEMPERATURE, LEVEL_FORCING

/// @brief The record of one interval of a run, given to the observers and the checks.
///
/// AtmosphereSimulation::run() makes it after each interval. It refers to that interval's fields and profiles, so it
/// is valid only during the observer's or the check's call: do not keep it.
struct IntervalSnapshot {
    si::Duration time{ au::ZERO };               ///< the simulated time at the interval's end, from the start of the run
    std::uint64_t step = 0u;                     ///< the core's time step count at the interval's end
    bool recording = false;                      ///< true for a recorded interval (after the spin-up)
    const AtmosphereFields& fields;              ///< the interval's fields on the host
    const LevelProfiles& profiles;               ///< the interval's layer means (and, in a moist run, the cloud levels)
    si::Length inversion{ au::ZERO };            ///< the inversion height zi: the level of the most negative resolved heat flux below entrainment_floor times the floor's flux, in the search window of BoundaryLayerDiagnostics::inversion_height(); the start's zi until such a flux exists
    std::optional<si::Length> mixed_layer_top{}; ///< the mixed layer's top (recorded intervals only), or none
    si::Speed w_min{ au::ZERO };                 ///< the smallest of the levels' smallest vertical speeds, the floor's and the lid's zero entries included: 0 or less; not a number when a cell's w is not finite
    si::Speed w_max{ au::ZERO };                 ///< the largest of the levels' largest vertical speeds, the floor's and the lid's zero entries included: 0 or more; not a number when a cell's w is not finite
};

/// @brief What a run did, given to the observers at its end and returned by the run.
struct RunSummary {
    std::uint32_t intervals = 0u;       ///< the number of intervals run, the spin-up included
    std::uint32_t recorded = 0u;        ///< the number of recorded intervals (after the spin-up)
    std::vector<CheckResult> results{}; ///< the end results of the checks that give one, in the order the checks were added
    bool passed = true;                 ///< true when every check's end result passed (a check that fails an interval ends the program, so no summary is returned then)
};

/// @brief The interface of a reader of a run's intervals (a profile writer, a recorder, a capture).
///
/// The run does not own an observer: it must outlive the run. After each interval the run calls the observers'
/// on_interval() in the order they were added, then updates the level forcing, then prints the PROGRESS line, then
/// runs the checks. At the end it gives the checks' results, then calls on_finish() once. When a check fails an
/// interval, the program ends (exit 1) and on_finish() is not called.
class IntervalObserver {
public:
    virtual ~IntervalObserver() = default;
    /// @brief Reads one interval.
    /// @param snapshot the interval's record, valid only during the call
    virtual void on_interval(const IntervalSnapshot& snapshot) = 0;
    /// @brief Reads the end of the run.
    /// @param summary what the run did
    virtual void on_finish(const RunSummary& summary) = 0;
};

/// @brief The interface of a check that judges a run: after each interval and at the end (a RESULT line, or none).
/// The run owns its checks.
///
/// A FAILED line from on_interval() is printed and the program ends at once with std::exit(1): the run does not
/// return, no RunSummary is made and no on_finish() is called. A failed result from on_finish() only makes
/// RunSummary::passed false.
class RunCheck {
public:
    virtual ~RunCheck() = default;
    /// @brief Judges one interval.
    /// @param snapshot the interval's record, valid only during the call
    /// @return a FAILED line, after which the program ends (exit 1), or none to continue
    virtual std::optional<std::string> on_interval(const IntervalSnapshot& snapshot) = 0;
    /// @brief Judges the whole run at its end.
    /// @return the result with its RESULT line, or none for a check that only guards the intervals
    virtual std::optional<CheckResult> on_finish() = 0;
};

/// @brief The check that stops a run that blew up: a vertical speed that is not finite (NaN or infinite), or one
/// beyond the limit (40 m/s by default, BlowUpCheck).
///
/// It judges IntervalSnapshot::w_min and w_max, which are not a number when any cell's w is not finite.
class BlowUpGuard final : public RunCheck {
public:
    /// @brief Makes the guard.
    /// @param check the largest |w| of a healthy run
    explicit BlowUpGuard(BlowUpCheck check = BlowUpCheck()) : check_(check) {}
    /// @brief Judges the interval's vertical speed extremes.
    /// @param s the interval's record
    /// @return the FAILED line when w_min or w_max is not finite or beyond the limit, else none
    std::optional<std::string> on_interval(const IntervalSnapshot& s) override {
        const std::optional<std::string> failure = check_.failure(s.time, s.w_min, s.w_max);
        return failure;
    }
    /// @return none: the guard gives no RESULT line
    std::optional<CheckResult> on_finish() override { return std::nullopt; }

private:
    BlowUpCheck check_; ///< the largest |w| of a healthy run
};

#ifdef MOISTURE
/// @brief The check of a moist run's clouds: after each interval it prints the CLOUD line (the cloud extent and the
/// mixed layer's condensation level); at the end it gives the CloudBaseCheck of the recorded intervals.
class CloudBaseReport final : public RunCheck {
public:
    /// @brief Makes the report.
    /// @param column the moist column (each level's pressure, for the condensation level); copied
    /// @param theta_0 the reference potential temperature of the layer means
    /// @param zi the case's mixed layer depth: the condensation level averages theta_l and qt between 0.25 zi and
    /// 0.75 zi
    /// @param grid the column's levels
    /// @param floor_flux the floor's kinematic heat flux, more than 0 (the scale of an entrainment flux in the
    /// diagnostics)
    /// @param min_fraction the share of wet cells (a ratio) at which a level counts as cloudy
    /// @throws SetupError when the floor's heat flux is not more than 0 (BoundaryLayerDiagnostics)
    CloudBaseReport(const MoistColumn& column, si::Temperature theta_0, si::Length zi, const LevelGrid& grid, si::KinematicHeatFlux floor_flux,
                    double min_fraction = CloudExtent::default_min_fraction)
        : column_(column), theta_0_(theta_0), zi_(zi), diagnostics_(grid, zi, std::nullopt, floor_flux), grid_(grid), min_fraction_(min_fraction) {}

    /// @brief Prints the interval's CLOUD line and adds the interval to the end check. A dry interval (no cloud
    /// levels) is skipped.
    /// @param s the interval's record
    /// @return none: the report never stops a run
    /// @throws std::out_of_range from std::vector::at() when the cloud levels or the means have fewer levels than the grid
    std::optional<std::string> on_interval(const IntervalSnapshot& s) override {
        if(!s.profiles.clouds.has_value()) {
            return std::nullopt;
        }
        const CloudLevels& clouds = *s.profiles.clouds;
        const CloudExtent e = CloudExtent::of(clouds, grid_, min_fraction_);
        const std::optional<si::Length> level = diagnostics_.condensation_level(s.profiles.means, clouds.qt, column_, theta_0_, zi_);
        const double time_s = s.time.in(au::seconds);
        const double base_m = metres_or_none(e.base);
        const double top_m = metres_or_none(e.top);
        const double cover = e.cover;
        const double ql_max = e.ql_max.in(si::kilograms_of_water / si::kilograms);
        const double level_m = metres_or_none(level);
        std::printf("CLOUD t %.0f s: base %.0f m, top %.0f m, largest cloud fraction %.4f, largest mean ql %.3e kg/kg; the mixed layer's condensation level %.0f m\n",
                    time_s, base_m, top_m, cover, ql_max, level_m);
        check_.add(s.recording, e.base, level);
        return std::nullopt;
    }
    /// @brief Gives the cloud base check of the recorded intervals.
    /// @return the result and its RESULT CLOUD line; FAILED when no recorded interval had a cloud
    std::optional<CheckResult> on_finish() override {
        CheckResult result = check_.result();
        return result;
    }

private:
    /// The CLOUD line's number for no height.
    static constexpr double none_metres = -1.0;
    /// @brief Gives a height as the CLOUD line prints it.
    /// @param h a height, or none
    /// @return the height in metres, or -1 for none
    static double metres_or_none(const std::optional<si::Length>& h) {
        double metres{ none_metres };
        if(h) {
            metres = h->in(au::meters);
        }
        return metres;
    }

    MoistColumn column_;                                ///< the moist column: each level's pressure
    si::Temperature theta_0_{ si::kelvin_point(0.0) };  ///< the reference potential temperature of the layer means
    si::Length zi_{ au::ZERO };                         ///< the mixed layer depth for the condensation level's window
    BoundaryLayerDiagnostics diagnostics_;              ///< computes the condensation level
    LevelGrid grid_;                                    ///< the column's levels
    double min_fraction_{};                             ///< the share of wet cells at which a level is cloudy (a ratio)
    CloudBaseCheck check_;                              ///< the end check: the mean cloud base against the mean condensation level
};
#endif // MOISTURE

/// @brief The report of a held wind (information, not a check): at the end, the largest layer-mean deviation below zi
/// from the target, from the NudgingTerm's last interval. It always passes.
class WindHoldReport final : public RunCheck {
public:
    /// @brief Makes the report of a nudging term.
    /// @param term the nudging term; the run's LevelForcing owns it and outlives this check
    explicit WindHoldReport(const NudgingTerm& term) : term_(term) {}
    /// @brief Does nothing per interval: the report reads the nudging term only at the end.
    /// @return none: the report never stops a run
    std::optional<std::string> on_interval(const IntervalSnapshot&) override { return std::nullopt; }
    /// @brief Gives the WIND line: the last deviation in per cent of the target's reference speed.
    /// @return a passed result with the WIND line
    /// @throws SetupError when the WIND line does not fit its buffer
    std::optional<CheckResult> on_finish() override {
        const double deviation_percent = percent * term_.deviation();
        const WindNudging& nudging = term_.nudging();
        const double reference_speed = nudging.target().reference_speed().in(au::meters / au::second);
        const double tau = nudging.tau().in(au::seconds);
        constexpr std::size_t text_capacity{ 320u };
        char text[text_capacity]{};
        const int length = std::snprintf(text, sizeof(text),
                      "WIND the largest layer-mean deviation below zi from the target at the end: %.2f %% of %.2f m/s (information, not a check: with tau %.0f s "
                      "the nudged wind does not reach the target; compare it with an LES reference profile)",
                      deviation_percent, reference_speed, tau);
        const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
        if(cut) {
            throw SetupError("WindHoldReport: the WIND line does not fit its buffer");
        }
        CheckResult result{ .passed = true, .line = text };
        return result;
    }

private:
    static constexpr double percent = 100.0; ///< per cent per ratio

    const NudgingTerm& term_; ///< the nudging term whose last deviation is reported
};
