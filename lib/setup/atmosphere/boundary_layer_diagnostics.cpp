#include "setup/atmosphere/boundary_layer_diagnostics.hpp"
#include "setup/core/setup_error.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

void LayerMoments::add(si::TemperatureDifference theta, si::Speed w, si::Speed u, si::Speed v) { // the example's layer_means() sums, in its order
    t_ += theta;
    const auto theta_squared = theta * theta;
    tt_ += theta_squared;
    w_ += w;
    const auto w_squared = w * w;
    ww_ += w_squared;
    const auto w_cubed = w_squared * w;
    www_ += w_cubed;
    const auto w_theta = w * theta;
    wt_ += w_theta;
    u_ += u;
    v_ += v;
    const bool w_finite{ !au::isnan(w) && !au::isinf(w) };
    if(!w_finite) {
        non_finite_w_ = true; // a max or a min would drop it at the next finite cell
    }
    w_max_ = au::max(w_max_, w);
    w_min_ = au::min(w_min_, w);
    added_++;
}

LayerMeans LayerMoments::means(double cells) const {
    const bool none_added{ added_ == 0u };
    const bool no_cells{ !(cells > 0.0) };
    if(none_added || no_cells) {
        throw DiagnosticsError("LayerMoments: no cell");
    }
    const si::TemperatureDifference tm{ t_ / cells };
    const si::Speed u_mean{ u_ / cells };
    const si::Speed v_mean{ v_ / cells };
    const si::Speed wm{ w_ / cells };
    const auto ww_mean = ww_ / cells;
    const auto wm_squared = wm * wm;
    const si::SpeedVariance w_var{ ww_mean - wm_squared };
    const auto tt_mean = tt_ / cells;
    const auto tm_squared = tm * tm;
    const si::TemperatureVariance theta_var{ tt_mean - tm_squared };
    // the central third moment from the raw ones: www / n - 3 wm ww / n + 2 wm^3
    const auto www_mean = www_ / cells;
    const auto three_wm = 3.0 * wm;
    const auto three_wm_ww = three_wm * ww_;
    const auto cross = three_wm_ww / cells;
    const auto two_wm = 2.0 * wm;
    const auto two_wm_wm = two_wm * wm;
    const auto two_wm_cubed = two_wm_wm * wm;
    const auto raw_minus_cross = www_mean - cross;
    const si::SpeedCubed w3{ raw_minus_cross + two_wm_cubed };
    double w_skew{ 0.0 };
    if(w_var > au::ZERO) {
        const auto var_squared = w_var * w_var;
        const auto var_cubed = var_squared * w_var;
        const auto spread_cubed = au::sqrt(var_cubed);
        const auto ratio = w3 / spread_cubed;
        w_skew = au::as_raw_number(ratio);
    }
    const auto wt_mean = wt_ / cells;
    const auto wm_tm = wm * tm;
    const si::KinematicHeatFlux w_theta{ wt_mean - wm_tm };
    const si::Speed not_finite{ (au::meters / au::second)(std::numeric_limits<double>::quiet_NaN()) };
    const si::Speed w_max{ non_finite_w_ ? not_finite : w_max_ };
    const si::Speed w_min{ non_finite_w_ ? not_finite : w_min_ };
    const LayerMeans m{
        .theta = tm,
        .w_var = w_var,
        .w_skew = w_skew,
        .theta_var = theta_var,
        .w_theta = w_theta,
        .u = u_mean,
        .v = v_mean,
        .w_max = w_max,
        .w_min = w_min,
    };
    return m;
}

BoundaryLayerDiagnostics::BoundaryLayerDiagnostics(const LevelGrid& grid, si::Length start_zi, std::optional<si::Length> sponge_from,
                                                   si::KinematicHeatFlux floor_flux)
    : grid_(grid), start_zi_(start_zi), sponge_from_(sponge_from), entrainment_threshold_(floor_flux * entrainment_floor) {
    if(!(floor_flux > si::KinematicHeatFlux{ au::ZERO })) {
        throw SetupError("BoundaryLayerDiagnostics: the floor's heat flux must be more than 0 K m/s (zi is the inversion of a heated floor)");
    }
}

si::Length BoundaryLayerDiagnostics::inversion_height(const std::vector<LayerMeans>& means) const {
    si::KinematicHeatFlux lowest{ entrainment_threshold_ }; // a flux counts only below the threshold (not rounding noise)
    si::Length zi{ start_zi_ };
    const std::uint32_t nz{ grid_.levels() };
    const si::Length top_height{ grid_.height(nz - 1u) };
    const si::Length search_floor{ top_height * inversion_search_floor };
    for(std::uint32_t z = 2u; z + 2u < nz; z++) { // the example's inversion_height(): two levels off each wall
        const si::Length h{ grid_.height(z) };
        if(h < search_floor) {
            continue;
        }
        const bool in_sponge{ sponge_from_ && h >= *sponge_from_ };
        if(in_sponge) {
            break;
        }
        const si::KinematicHeatFlux flux{ means.at(z).w_theta };
        if(flux < lowest) {
            lowest = flux;
            zi = h;
        }
    }
    return zi;
}

si::TemperatureDifference BoundaryLayerDiagnostics::mixed_layer_theta(const std::vector<LayerMeans>& means) const {
    si::TemperatureDifference sum{ au::ZERO };
    double n{ 0.0 };
    const std::uint32_t first{ grid_.first_fluid() };
    const std::uint32_t last{ grid_.last_fluid() };
    for(std::uint32_t z = first; z <= last; z++) {
        const si::Length h{ grid_.height(z) };
        const bool in_window{ h >= mixed_window_bottom && h <= mixed_window_top };
        if(in_window) {
            sum += means.at(z).theta;
            n += 1.0;
        }
    }
    if(n == 0.0) {
        throw DiagnosticsError("BoundaryLayerDiagnostics::mixed_layer_top(): no level between 100 and 600 m");
    }
    const si::TemperatureDifference mixed{ sum / n };
    return mixed;
}

si::Length BoundaryLayerDiagnostics::mixed_layer_top(const std::vector<LayerMeans>& means) const {
    const std::uint32_t nz{ grid_.levels() };
    const si::TemperatureDifference mixed{ mixed_layer_theta(means) };
    const si::TemperatureDifference threshold{ mixed + mixed_top_excess };
    for(std::uint32_t z = grid_.first_fluid(); z + 1u < nz; z++) {
        const si::Length h{ grid_.height(z) };
        const bool above_window_bottom{ h > mixed_window_bottom };
        if(!above_window_bottom) {
            continue;
        }
        const si::TemperatureDifference theta{ means.at(z).theta };
        if(theta > threshold) {
            return h;
        }
    }
    throw DiagnosticsError("BoundaryLayerDiagnostics::mixed_layer_top(): theta nowhere 0.5 K above the mixed layer below the lid");
}

BoundaryLayerDiagnostics::WindowAir BoundaryLayerDiagnostics::window_air(const std::vector<LayerMeans>& means,
                                                                        const std::vector<si::WaterContent>& total_water, si::Length zi) const {
    WindowAir air{ .thl_excess = si::TemperatureDifference{ au::ZERO }, .qt = si::WaterContent{ au::ZERO }, .levels = 0.0 };
    const std::uint32_t first{ grid_.first_fluid() };
    const std::uint32_t last{ grid_.last_fluid() };
    const si::Length window_bottom{ zi * condensation_window_bottom };
    const si::Length window_top{ zi * condensation_window_top };
    for(std::uint32_t z = first; z <= last; z++) { // the example's condensation_level()
        const si::Length h{ grid_.height(z) };
        const bool in_window{ h >= window_bottom && h <= window_top };
        if(in_window) {
            air.thl_excess += means.at(z).theta;
            air.qt += total_water.at(z);
            air.levels += 1.0;
        }
    }
    return air;
}

std::optional<si::Length> BoundaryLayerDiagnostics::lowest_liquid_level(const MoistColumn& column, si::Temperature thl, si::WaterContent qt) const {
    const std::uint32_t first{ grid_.first_fluid() };
    const std::uint32_t last{ grid_.last_fluid() };
    for(std::uint32_t z = first; z <= last; z++) {
        const si::WaterContent liquid{ column.liquid(thl, qt, z) };
        if(liquid > au::ZERO) {
            const si::Length height{ grid_.height(z) };
            return height;
        }
    }
    return std::nullopt;
}

std::optional<si::Length> BoundaryLayerDiagnostics::condensation_level(const std::vector<LayerMeans>& means, const std::vector<si::WaterContent>& total_water,
                                                                       const MoistColumn& column, si::Temperature theta_0, si::Length zi) const {
    const WindowAir air{ window_air(means, total_water, zi) };
    if(air.levels == 0.0) {
        return std::nullopt;
    }
    const si::TemperatureDifference mean_excess{ air.thl_excess / air.levels };
    const si::Temperature thl{ theta_0 + mean_excess };
    const si::WaterContent mean_qt{ air.qt / air.levels };
    const std::optional<si::Length> level{ lowest_liquid_level(column, thl, mean_qt) };
    return level;
}
