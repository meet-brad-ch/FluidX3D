#pragma once

#include "core/types.hpp"

/**
 * @file thermal_utils.hpp
 * @brief Temperature conversion utilities for thermal simulations
 *
 * Provides helpers for converting between SI temperatures (Kelvin)
 * and LBM temperature units used in the TEMPERATURE extension.
 */

namespace thermal_utils {

/**
 * @brief Convert temperature from Kelvin to LBM units
 *
 * LBM temperature is normalized around 1.0:
 * - T_lbm = 1.0 at reference temperature
 * - T_lbm > 1.0 for temperatures above reference
 * - T_lbm < 1.0 for temperatures below reference
 *
 * Formula: T_lbm = 1.0 + (T_K - T_ref_K) / delta_T_K
 *
 * @param T_K Temperature in Kelvin
 * @param T_ref_K Reference temperature in Kelvin (typically average of hot/cold)
 * @param delta_T_K Temperature difference (hot - cold) in Kelvin
 * @return Temperature in LBM units
 */
inline float32_t kelvin_to_lbm(float32_t T_K, float32_t T_ref_K, float32_t delta_T_K) {
    if (delta_T_K == 0.0f) return 1.0f;
    return 1.0f + (T_K - T_ref_K) / delta_T_K;
}

/**
 * @brief Convert temperature from LBM units to Kelvin
 *
 * Inverse of kelvin_to_lbm.
 * Formula: T_K = T_ref_K + (T_lbm - 1.0) * delta_T_K
 *
 * @param T_lbm Temperature in LBM units
 * @param T_ref_K Reference temperature in Kelvin
 * @param delta_T_K Temperature difference in Kelvin
 * @return Temperature in Kelvin
 */
inline float32_t lbm_to_kelvin(float32_t T_lbm, float32_t T_ref_K, float32_t delta_T_K) {
    return T_ref_K + (T_lbm - 1.0f) * delta_T_K;
}

/**
 * @brief Calculate reference temperature from hot and cold wall temperatures
 *
 * @param T_hot_K Hot wall temperature in Kelvin
 * @param T_cold_K Cold wall temperature in Kelvin
 * @return Reference temperature (average) in Kelvin
 */
inline float32_t calc_reference_temperature(float32_t T_hot_K, float32_t T_cold_K) {
    return 0.5f * (T_hot_K + T_cold_K);
}

/**
 * @brief Calculate temperature difference from hot and cold wall temperatures
 *
 * @param T_hot_K Hot wall temperature in Kelvin
 * @param T_cold_K Cold wall temperature in Kelvin
 * @return Temperature difference in Kelvin
 */
inline float32_t calc_delta_temperature(float32_t T_hot_K, float32_t T_cold_K) {
    return T_hot_K - T_cold_K;
}

} // namespace thermal_utils
