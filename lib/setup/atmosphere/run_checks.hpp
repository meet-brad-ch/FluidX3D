#pragma once

#include "setup/domain/level_grid.hpp"
#include "setup/units/units.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

/// @file run_checks.hpp
/// @brief The values of the atmosphere checks (no core needed): the cloud extent of an interval, the cloud base check
/// and the blow-up check. interval.hpp wraps them as RunCheck objects.

/// @brief The outcome of a run check at the end of a run: whether it passed, and its RESULT line.
struct CheckResult {
    bool passed = false; ///< true when the check passed
    std::string line{};  ///< the RESULT line for the run's log
};

/// @brief The moist air of every level: the horizontal means of qt and ql, and the share of wet cells. Each vector
/// is indexed by the level z, 0 (the floor) to levels - 1 (the lid).
struct CloudLevels {
    std::vector<si::WaterContent> qt{}; ///< the mean total water of each level
    std::vector<si::WaterContent> ql{}; ///< the mean liquid water of each level
    std::vector<double> fraction{};     ///< the share of each level's cells that hold liquid water (a ratio)
};

/// @brief The cloud layer of one interval: the lowest and the highest cloudy level (at least a minimum share of wet
/// cells), the largest cloud fraction and the largest mean liquid water (the CBL example's CLOUD line).
struct CloudExtent {
    /// A level is cloudy at this share of wet cells or more (the CBL example's 1e-3).
    static constexpr double default_min_fraction = 1.0E-3;

    std::optional<si::Length> base{};      ///< the centre height of the lowest cloudy level above the floor's top, or none
    std::optional<si::Length> top{};       ///< the centre height of the highest cloudy level, or none
    double cover = 0.0;                    ///< the largest share of wet cells of a fluid level (a ratio)
    si::WaterContent ql_max{ au::ZERO };   ///< the largest mean liquid water of a fluid level

    /// @brief Finds the cloud layer of one interval among the fluid levels.
    /// @param clouds the levels' moist air, indexed by z
    /// @param grid the column's levels (the fluid levels are searched)
    /// @param min_fraction the share of wet cells (a ratio) at which a level counts as cloudy
    /// @return the extent; base and top are none when no level is cloudy
    /// @throws std::out_of_range from std::vector::at() when clouds has fewer levels than the grid
    static CloudExtent of(const CloudLevels& clouds, const LevelGrid& grid, double min_fraction = default_min_fraction);
};

/// @brief The check of a moist run: the time-mean cloud base of the recorded intervals within a limit of the
/// time-mean condensation level of the mixed layer's air; FAILED when no recorded interval had a cloud.
class CloudBaseCheck {
public:
    /// The largest relative difference by default (the CBL example's 10 %).
    static constexpr double default_limit = 0.10;

    /// @brief Makes the check.
    /// @param limit the largest relative difference |base / level - 1| that passes, more than 0 (a ratio)
    /// @throws SetupError when the limit is not more than 0
    explicit CloudBaseCheck(double limit = default_limit);
    /// @brief Adds one interval. Only a recorded interval with a cloud base and a condensation level, both above 0,
    /// adds to the means; every recorded interval is counted.
    /// @param recording true for a recorded interval (after the spin-up); the others are ignored
    /// @param base the interval's cloud base, or none without a cloud
    /// @param condensation_level the interval's condensation level, or none
    void add(bool recording, std::optional<si::Length> base, std::optional<si::Length> condensation_level);
    /// @brief Judges the intervals added so far.
    /// @return the outcome and its RESULT CLOUD line; FAILED when no recorded interval had a cloud
    /// @throws SetupError when the line does not fit its buffer
    CheckResult result() const;

private:
    /// @brief Judges the means of the cloudy intervals (at least one).
    /// @return the outcome and its RESULT CLOUD line
    /// @throws SetupError when the line does not fit its buffer
    CheckResult cloudy_result() const;

    double limit_{};                          ///< the largest relative difference that passes (a ratio)
    si::Length base_sum_{ au::ZERO };         ///< the sum of the cloud bases of the cloudy recorded intervals
    si::Length level_sum_{ au::ZERO };        ///< the sum of the condensation levels of the same intervals
    std::uint32_t cloudy_ = 0u;               ///< the number of recorded intervals with a cloud and a condensation level
    std::uint32_t recorded_ = 0u;             ///< the number of recorded intervals
};

/// @brief The stop of a run that blew up: a non-finite vertical speed extreme, or one beyond a limit.
///
/// It judges the two extremes it is given. BlowUpGuard gives it IntervalSnapshot::w_min and w_max, which are not a
/// number when any cell's w is not finite.
class BlowUpCheck {
public:
    /// The largest |w| of a healthy run by default (the CBL example's).
    static constexpr si::Speed default_limit = (au::meters / au::second)(40.0);

    /// @brief Makes the check.
    /// @param limit the largest |w| of a healthy run, more than 0
    /// @throws SetupError when the limit is not more than 0
    explicit BlowUpCheck(si::Speed limit = default_limit);
    /// @brief Judges one interval's vertical speed extremes.
    /// @param time the simulated time at the interval's end (for the line)
    /// @param w_min the smallest vertical speed of the interval's fields
    /// @param w_max the largest vertical speed of the interval's fields
    /// @return the FAILED line when an extreme is not finite or beyond the limit, none when the flow is healthy
    /// @throws SetupError when the line does not fit its buffer
    std::optional<std::string> failure(si::Duration time, si::Speed w_min, si::Speed w_max) const;

private:
    si::Speed limit_{ au::ZERO }; ///< the largest |w| of a healthy run
};
