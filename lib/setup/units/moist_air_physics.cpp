#include "setup/units/moist_air_physics.hpp"

#include "setup/core/setup_error.hpp"

#include "moist_air.hpp"

#include <cmath>
#include <string>

namespace si {

namespace {

/// The factor of the virtual temperature T_v = T (1 + 0.61 q), a ratio. hydrostatic_step() applies it to the total
/// water qt, as the MicroHH reference runs do, not to the vapour alone.
constexpr double virtual_factor = 0.61;

/// @brief Gives a temperature reading as the model's number.
/// @param t a temperature reading
/// @return the reading in K
double kelvin(Temperature t) {
    const double value{ t.in(au::kelvins_pt) };
    return value;
}
/// @brief Gives a water content as the model's number.
/// @param q a water content
/// @return the content in kg of water per kg of air
double kg_per_kg(WaterContent q) {
    const double value{ q.in(kilograms_of_water / kilograms) };
    return value;
}
/// @brief Gives a pressure as the model's number.
/// @param p a pressure
/// @return the pressure in Pa
double pascal(Pressure p) {
    const double value{ p.in(au::pascals) };
    return value;
}

} // namespace

AdjustedAir MoistAirPhysics::saturation_adjust(Temperature theta_l, WaterContent total_water, Pressure pressure) {
    const double theta_l_k{ kelvin(theta_l) };
    const double total_water_ratio{ kg_per_kg(total_water) };
    const double pressure_pa{ pascal(pressure) };
    const moist_air::Adjusted a{ moist_air::saturation_adjust(theta_l_k, total_water_ratio, pressure_pa) };
    const Temperature temperature{ kelvin_point(a.kelvin) };
    const WaterContent liquid_water{ (kilograms_of_water / kilograms)(a.liquid) };
    const AdjustedAir adjusted{ .temperature = temperature, .liquid = liquid_water };
    return adjusted;
}

WaterContent MoistAirPhysics::liquid(Temperature theta_l, WaterContent total_water, Pressure pressure) {
    const AdjustedAir adjusted{ saturation_adjust(theta_l, total_water, pressure) };
    const WaterContent liquid_water{ adjusted.liquid };
    return liquid_water;
}

Temperature MoistAirPhysics::virtual_theta(Temperature theta_l, WaterContent total_water, Pressure pressure) {
    const double theta_l_k{ kelvin(theta_l) };
    const double total_water_ratio{ kg_per_kg(total_water) };
    const double pressure_pa{ pascal(pressure) };
    const double theta_v_k{ moist_air::virtual_theta(theta_l_k, total_water_ratio, pressure_pa) };
    const Temperature theta_v{ kelvin_point(theta_v_k) };
    return theta_v;
}

Pressure MoistAirPhysics::isentropic_pressure(Length height, Temperature theta, Pressure ground) {
    const double height_m{ height.in(au::meters) };
    const double theta_k{ kelvin(theta) };
    const double ground_pa{ pascal(ground) };
    const double top_m{ moist_air::Cp * theta_k / moist_air::G }; // the raw model's constants: its closed form ends there
    if(!(height_m < top_m)) {
        const std::string height_text{ std::to_string(height_m) };
        const std::string top_text{ std::to_string(top_m) };
        throw SetupError("MoistAirPhysics::isentropic_pressure(): the height " + height_text + " m is not below cp theta / g (" + top_text + " m)");
    }
    const double pressure_pa{ moist_air::hydrostatic_pressure(height_m, theta_k, ground_pa) };
    const Pressure pressure{ au::pascals(pressure_pa) };
    return pressure;
}

Pressure MoistAirPhysics::hydrostatic_step(Pressure pressure, Temperature theta_l, WaterContent total_water, Length rise) {
    const double p{ pascal(pressure) }; // the example's set_up_moist(), the same double steps
    const double theta_l_k{ kelvin(theta_l) };
    const double pressure_ratio{ p / moist_air::PRef };
    const double exner_exponent{ moist_air::Rd / moist_air::Cp };
    const double exner{ std::pow(pressure_ratio, exner_exponent) };
    const double total_water_ratio{ kg_per_kg(total_water) };
    const double virtual_part{ virtual_factor * total_water_ratio };
    const double virtual_correction{ 1.0 + virtual_part };
    const double temperature_dry{ theta_l_k * exner };
    const double t_k{ temperature_dry * virtual_correction };
    const double rise_m{ rise.in(au::meters) };
    const double minus_g{ -moist_air::G };
    const double geopotential{ minus_g * rise_m };
    const double gas_term{ moist_air::Rd * t_k };
    const double exponent{ geopotential / gas_term };
    const double decay{ std::exp(exponent) };
    const double upper_pa{ p * decay };
    const Pressure upper{ au::pascals(upper_pa) };
    return upper;
}

} // namespace si
