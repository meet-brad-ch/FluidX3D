#pragma once

#include "setup/core/types.hpp"

// Kelvin <-> LBM temperature (TEMPERATURE extension): T_lbm = 1 + (T_K - T_ref_K)/delta_T_K
namespace thermal_utils {

inline float32_t kelvin_to_lbm(float32_t T_K, float32_t T_ref_K, float32_t delta_T_K) {
    if (delta_T_K == 0.0f) return 1.0f;
    return 1.0f + (T_K - T_ref_K) / delta_T_K;
}

inline float32_t calc_reference_temperature(float32_t T_hot_K, float32_t T_cold_K) {
    return 0.5f * (T_hot_K + T_cold_K);
}

inline float32_t calc_delta_temperature(float32_t T_hot_K, float32_t T_cold_K) {
    return T_hot_K - T_cold_K;
}

} // namespace thermal_utils
