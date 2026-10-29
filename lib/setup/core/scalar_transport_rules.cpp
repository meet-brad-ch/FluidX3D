#include "setup/core/scalar_transport_rules.hpp"
#include "setup/core/setup_error.hpp"

#include <cmath>
#include <cstdio>

float ScalarTransportRules::core_alpha(float diffusivity, const TransportBuild& build) {
    const bool valid{ std::isfinite(diffusivity) && diffusivity >= 0.0f };
    if(!valid) {
        throw SetupError("ScalarTransportRules::core_alpha(): the diffusivity must be a finite number, 0 or more");
    }
    if(build.finite_volumes && !build.diffusivity_fix) {
        throw SetupError("ScalarTransportRules::core_alpha(): SCALAR_FV needs THERMAL_DIFFUSIVITY_FIX");
    }
    const float doubled{ 2.0f * diffusivity }; // without the fix the core's rate halves alpha
    const float alpha{ build.diffusivity_fix ? diffusivity : doubled };
    return alpha;
}

float ScalarTransportRules::relaxation_time(float diffusivity) {
    const float scaled{ lattice_rate_factor * diffusivity };
    const float tau{ scaled + 0.5f };
    return tau;
}

std::optional<std::string> ScalarTransportRules::refusal(float diffusivity, const TransportBuild& build, const ScalarContext& context) {
    if(build.finite_volumes) {
        return std::nullopt;
    }
    if(build.fp16_ddfs) {
        return std::string("the D3Q7 thermal lattice with FP16S DDFs does not diffuse a temperature wave in a flow (tests/physics/thermal_diffusivity.cpp): use SCALAR_FV, or FP32 DDFs");
    }
    const float tau{ relaxation_time(diffusivity) };
    const bool unsafe{ context.solid_walls && context.buoyant && tau < wall_safe_relaxation_time };
    if(!unsafe) {
        return std::nullopt;
    }
    const double tau_d{ static_cast<double>(tau) };
    const double safe_tau{ static_cast<double>(wall_safe_relaxation_time) };
    const float safe_diffusivity{ (wall_safe_relaxation_time - 0.5f) / lattice_rate_factor };
    const double safe_diffusivity_d{ static_cast<double>(safe_diffusivity) };
    constexpr std::size_t text_capacity{ 320 };
    char text[text_capacity]{};
    const int written{ std::snprintf(text, sizeof(text), "the D3Q7 thermal lattice at tauT %.4f (below %.3f) creates heat next to solid walls in a buoyant flow (tests/physics/heat_conservation.cpp): use SCALAR_FV, or a scalar diffusivity of at least %.4f in lattice units",
                                     tau_d, safe_tau, safe_diffusivity_d) };
    if(written < 0 || static_cast<std::size_t>(written) >= text_capacity) {
        throw SetupError("ScalarTransportRules::refusal(): the reason does not fit its text buffer");
    }
    std::optional<std::string> reason{ text };
    return reason;
}

std::vector<std::string> ScalarTransportRules::warnings(float diffusivity, const TransportBuild& build, const ScalarContext& context) {
    std::vector<std::string> lines{};
    if(build.finite_volumes) {
        return lines;
    }
    const float tau{ relaxation_time(diffusivity) };
    if(context.buoyant && tau < warning_relaxation_time) {
        const double tau_d{ static_cast<double>(tau) };
        const double warning_tau{ static_cast<double>(warning_relaxation_time) };
        constexpr std::size_t text_capacity{ 240 };
        char text[text_capacity]{};
        const int written{ std::snprintf(text, sizeof(text), "the D3Q7 thermal lattice at tauT %.4f (below %.2f) in a buoyant flow: next to solid walls it creates heat (tests/physics/heat_conservation.cpp)",
                                         tau_d, warning_tau) };
        if(written < 0 || static_cast<std::size_t>(written) >= text_capacity) {
            throw SetupError("ScalarTransportRules::warnings(): the warning does not fit its text buffer");
        }
        lines.emplace_back(text);
    }
    if(build.fp16_ddfs) {
        lines.emplace_back("FP16S DDFs with the D3Q7 thermal lattice: a temperature wave does not diffuse in a flow, and T above about 8 overflows (tests/physics/thermal_diffusivity.cpp)");
    }
    if(context.moist) {
        lines.emplace_back("the D3Q7 moisture lattice does not keep a uniform water content uniform in a flow (up to 8 % in 2700 s of a convective boundary layer): SCALAR_FV keeps it");
    }
    return lines;
}
