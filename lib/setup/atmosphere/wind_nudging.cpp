#include "setup/atmosphere/wind_nudging.hpp"
#include "setup/core/setup_error.hpp"

#include <algorithm>
#include <string>

WindNudging::WindNudging(std::shared_ptr<const WindTarget> target, si::Duration tau) : target_(std::move(target)), tau_(tau) {
    if(!target_) {
        throw SetupError("WindNudging: no wind target");
    }
    if(!(tau > au::ZERO)) {
        throw SetupError("WindNudging: the time scale must be more than 0 s");
    }
}

void WindNudging::check_means(const std::vector<LayerMeans>& means, const LevelGrid& grid) {
    const std::size_t needed{ static_cast<std::size_t>(grid.last_fluid()) + 1u };
    if(means.size() < needed) {
        const std::string given_text{ std::to_string(means.size()) };
        const std::string needed_text{ std::to_string(needed) };
        throw SetupError("WindNudging: " + given_text + " layer means for a column that needs " + needed_text + " (one per level up to the last fluid level)");
    }
}

WindNudging::Forces WindNudging::forces(const std::vector<LayerMeans>& means, const LevelGrid& grid, const si::LatticeConverter& converter) const {
    check_means(means, grid);
    const std::uint32_t level_count{ grid.levels() };
    std::vector<float> fx(level_count, 0.0f);
    std::vector<float> fy(level_count, 0.0f);
    const auto step_ratio = converter.step() / tau_;
    const double per_step{ au::as_raw_number(step_ratio) };
    const std::uint32_t first{ grid.first_fluid() };
    const std::uint32_t last{ grid.last_fluid() };
    for(std::uint32_t z = first; z <= last; z++) { // the example's (target - <u>) dt/tau, made a lattice speed with dt/dx
        const si::Length h{ grid.height(z) };
        const HorizontalWind t{ target_->at(h) };
        const LayerMeans& layer{ means.at(z) };
        const si::Speed error_x{ t.x - layer.u };
        const si::Speed error_y{ t.y - layer.v };
        const si::Speed nudge_x{ error_x * per_step };
        const si::Speed nudge_y{ error_y * per_step };
        fx[z] = converter.velocity(nudge_x);
        fy[z] = converter.velocity(nudge_y);
    }
    Forces f{ .fx = std::move(fx), .fy = std::move(fy) };
    return f;
}

double WindNudging::deviation(const std::vector<LayerMeans>& means, const LevelGrid& grid, si::Length zi) const {
    check_means(means, grid);
    double worst{ 0.0 };
    const std::uint32_t first{ grid.first_fluid() };
    const std::uint32_t last{ grid.last_fluid() };
    for(std::uint32_t z = first; z <= last; z++) {
        const si::Length h{ grid.height(z) };
        if(!(h < zi)) {
            continue;
        }
        const HorizontalWind t{ target_->at(h) };
        const LayerMeans& layer{ means.at(z) };
        const si::Speed error_x{ t.x - layer.u };
        const si::Speed error_y{ t.y - layer.v };
        const auto error = au::hypot(error_x, error_y);
        const si::Speed reference{ target_->reference_speed() };
        const auto ratio = error / reference;
        const double relative{ au::as_raw_number(ratio) };
        worst = std::max(worst, relative);
    }
    return worst;
}
