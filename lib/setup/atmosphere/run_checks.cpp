#include "setup/atmosphere/run_checks.hpp"
#include "setup/core/setup_error.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>

namespace {

/// Per cent per ratio, for the RESULT line.
constexpr double percent = 100.0;

} // namespace

CloudExtent CloudExtent::of(const CloudLevels& clouds, const LevelGrid& grid, double min_fraction) {
    std::optional<si::Length> cloud_base{};
    std::optional<si::Length> cloud_top{};
    double largest_cover{ 0.0 };
    si::WaterContent largest_ql{ au::ZERO };
    const std::uint32_t first{ grid.first_fluid() };
    const std::uint32_t last{ grid.last_fluid() };
    for(std::uint32_t z = first; z <= last; z++) { // the example's report_cloud()
        const double fraction{ clouds.fraction.at(z) };
        const bool cloudy{ fraction >= min_fraction };
        const bool lowest_cloud{ cloudy && !cloud_base.has_value() };
        if(lowest_cloud) {
            cloud_base = grid.height(z);
        }
        if(cloudy) {
            cloud_top = grid.height(z);
        }
        largest_cover = std::max(largest_cover, fraction);
        const si::WaterContent ql{ clouds.ql.at(z) };
        largest_ql = au::max(largest_ql, ql);
    }
    const CloudExtent extent{ .base = cloud_base, .top = cloud_top, .cover = largest_cover, .ql_max = largest_ql };
    return extent;
}

CloudBaseCheck::CloudBaseCheck(double limit) : limit_(limit) {
    if(!(limit > 0.0)) {
        throw SetupError("CloudBaseCheck: the limit must be more than 0");
    }
}

void CloudBaseCheck::add(bool recording, std::optional<si::Length> base, std::optional<si::Length> condensation_level) {
    if(!recording) {
        return;
    }
    recorded_++;
    const bool both_known{ base && condensation_level };
    if(both_known) {
        const bool base_positive{ *base > au::ZERO };
        const bool level_positive{ *condensation_level > au::ZERO };
        if(base_positive && level_positive) {
            base_sum_ += *base;
            level_sum_ += *condensation_level;
            cloudy_++;
        }
    }
}

CheckResult CloudBaseCheck::cloudy_result() const {
    const double cloudy_count{ static_cast<double>(cloudy_) };
    const si::Length base{ base_sum_ / cloudy_count };
    const si::Length level{ level_sum_ / cloudy_count };
    const auto ratio = base / level;
    const double base_over_level{ au::as_raw_number(ratio) };
    const double difference{ base_over_level - 1.0 };
    const double error{ std::fabs(difference) };
    const bool passed{ error <= limit_ };
    const double base_m{ base.in(au::meters) };
    const double level_m{ level.in(au::meters) };
    const double error_percent{ percent * error };
    const double limit_percent{ percent * limit_ };
    const char* const verdict{ passed ? "PASSED" : "FAILED" };
    const unsigned cloudy_intervals{ cloudy_ };
    const unsigned recorded_intervals{ recorded_ };
    constexpr std::size_t text_capacity{ 320u };
    char text[text_capacity]{};
    const int length = std::snprintf(text, sizeof(text),
                  "RESULT CLOUD the mean cloud base %.0f m against the mean condensation level %.0f m: %.1f %% (limit %.0f %%) %s; a cloud in %u of %u recorded intervals",
                  base_m, level_m, error_percent, limit_percent, verdict, cloudy_intervals, recorded_intervals);
    const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
    if(cut) {
        throw SetupError("CloudBaseCheck: the line does not fit its buffer");
    }
    const CheckResult outcome{ .passed = passed, .line = text };
    return outcome;
}

CheckResult CloudBaseCheck::result() const {
    CheckResult outcome{};
    if(cloudy_ == 0u) {
        const unsigned recorded_intervals{ recorded_ };
        constexpr std::size_t text_capacity{ 80u };
        char text[text_capacity]{};
        const int length = std::snprintf(text, sizeof(text), "RESULT CLOUD FAILED: no cloud in the %u recorded intervals", recorded_intervals);
        const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
        if(cut) {
            throw SetupError("CloudBaseCheck: the line does not fit its buffer");
        }
        outcome = CheckResult{ .passed = false, .line = text };
    } else {
        outcome = cloudy_result();
    }
    return outcome;
}

BlowUpCheck::BlowUpCheck(si::Speed limit) : limit_(limit) {
    if(!(limit_ > au::ZERO)) {
        throw SetupError("BlowUpCheck: the limit must be more than 0 m/s");
    }
}

std::optional<std::string> BlowUpCheck::failure(si::Duration time, si::Speed w_min, si::Speed w_max) const {
    const bool w_max_finite{ !au::isnan(w_max) && !au::isinf(w_max) };
    const bool w_min_finite{ !au::isnan(w_min) && !au::isinf(w_min) };
    const si::Speed minus_limit{ -limit_ };
    const bool w_max_within{ w_max <= limit_ };
    const bool w_min_within{ w_min >= minus_limit };
    const bool healthy{ w_max_finite && w_min_finite && w_max_within && w_min_within };
    std::optional<std::string> failure_line{};
    if(!healthy) {
        const double time_s{ time.in(au::seconds) };
        const double w_min_m_s{ w_min.in(au::meters / au::second) };
        const double w_max_m_s{ w_max.in(au::meters / au::second) };
        constexpr std::size_t text_capacity{ 160u };
        char text[text_capacity]{};
        const int length = std::snprintf(text, sizeof(text), "FAILED: the flow blew up at t %.0f s (w %.2f..%.2f m/s)", time_s, w_min_m_s, w_max_m_s);
        const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
        if(cut) {
            throw SetupError("BlowUpCheck: the line does not fit its buffer");
        }
        failure_line = std::string(text);
    }
    return failure_line;
}
