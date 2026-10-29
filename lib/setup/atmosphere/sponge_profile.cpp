#include "setup/atmosphere/sponge_profile.hpp"
#include "setup/core/setup_error.hpp"

#include <cstddef>
#include <cstdio>

SpongeProfile SpongeProfile::quadratic(si::Length from, si::Duration tau_at_lid) {
    if(!(from > au::ZERO)) {
        throw SetupError("SpongeProfile: the layer's start must be more than 0 m (no sponge: SpongeProfile::none())");
    }
    if(!(tau_at_lid > au::ZERO)) {
        throw SetupError("SpongeProfile: the time scale must be more than 0 s");
    }
    SpongeProfile sponge;
    sponge.layer_ = Layer{ .from = from, .tau = tau_at_lid };
    return sponge;
}

float SpongeProfile::lattice_rate(si::Length h, si::Length lid, const si::LatticeConverter& converter) const {
    float rate{ 0.0f };
    if(layer_) {
        const si::Length from{ layer_->from };
        if(!(lid > from)) {
            throw SetupError("SpongeProfile: the lid must be above the sponge's start");
        }
        if(!(h < from)) {
            const si::Length into_layer{ h - from };
            const si::Length depth{ lid - from };
            const auto layer_ratio = into_layer / depth;
            const double f{ au::as_raw_number(layer_ratio) };
            const auto step_ratio = converter.step() / layer_->tau;
            const double per_tau{ au::as_raw_number(step_ratio) };
            const double per_tau_f{ per_tau * f };
            const double per_tau_f_f{ per_tau_f * f }; // the example's dt / tau * f * f
            rate = static_cast<float>(per_tau_f_f);
        }
    }
    return rate;
}

std::vector<float> SpongeProfile::lattice_rates(const LevelGrid& grid, const si::LatticeConverter& converter) const {
    const std::uint32_t level_count{ grid.levels() };
    std::vector<float> rates(level_count, 0.0f);
    const si::Length lid{ grid.lid_height() };
    for(std::uint32_t z = 0u; z < level_count; z++) {
        const si::Length h{ grid.height(z) };
        rates[z] = lattice_rate(h, lid, converter);
    }
    return rates;
}

std::string SpongeProfile::describe() const {
    std::string description{ "no sponge" };
    if(layer_) {
        const double from_m{ layer_->from.in(au::meters) };
        const double tau_s{ layer_->tau.in(au::seconds) };
        constexpr std::size_t text_capacity{ 120u };
        char text[text_capacity]{};
        const int length = std::snprintf(text, sizeof(text), "sponge from %.0f m, quadratic to 1/%.0f s at the lid", from_m, tau_s);
        const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
        if(cut) {
            throw SetupError("SpongeProfile::describe(): the text does not fit its buffer");
        }
        description = text;
    }
    return description;
}
