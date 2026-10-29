#include "setup/units/lattice_converter.hpp"
#include "setup/core/setup_error.hpp"

#include <algorithm>
#include <string>

namespace si {

namespace {

/// The rounding of a time in steps, as the core's to_ulong(): half a step up.
constexpr double half_step = 0.5;
/// The lattice density at a lattice pressure p is 1 + this factor times p (the lattice pressure is (ρ - 1)/3).
constexpr double pressure_to_density = 3.0;

} // namespace

Duration LatticeConverter::nearest_float_step(Duration step) {
    const double seconds{ step.in(au::seconds) };
    const float seconds_float{ static_cast<float>(seconds) };
    const double seconds_rounded{ static_cast<double>(seconds_float) };
    const Duration rounded{ au::seconds(seconds_rounded) };
    return rounded;
}

Length LatticeConverter::nearest_float_cell(Length cell) {
    const double metres{ cell.in(au::meters) };
    const float metres_float{ static_cast<float>(metres) };
    const double metres_rounded{ static_cast<double>(metres_float) };
    const Length rounded{ au::meters(metres_rounded) };
    return rounded;
}

LatticeConverter::LatticeConverter(Length cell, Duration step) : cell_(cell), step_(nearest_float_step(step)) {
    if(!(cell > au::ZERO)) {
        throw SetupError("si::LatticeConverter: the cell size must be above 0 m");
    }
    if(!(step > au::ZERO)) {
        throw SetupError("si::LatticeConverter: the time step must be above 0 s");
    }
    const Length float_cell{ nearest_float_cell(cell) };
    if(float_cell != cell) {
        const double metres{ cell.in(au::meters) };
        const std::string metres_text{ std::to_string(metres) };
        const std::string message{ "si::LatticeConverter: the cell " + metres_text + " m is not a float exactly (the core's cell)" };
        throw SetupError(message);
    }
}

LatticeConverter::LatticeConverter(Length cell, Duration step, Temperature theta_0) : LatticeConverter(cell, step) {
    theta_0_ = theta_0;
}

LatticeConverter LatticeConverter::from_reference(Length reference_length, double reference_cells, Speed reference_speed, LatticeMach mach) {
    const bool length_positive{ reference_length > au::ZERO };
    const bool cells_positive{ reference_cells > 0.0 };
    const bool speed_positive{ reference_speed > au::ZERO };
    if(!length_positive || !cells_positive || !speed_positive) {
        throw SetupError("si::LatticeConverter::from_reference(): the reference length, its cells and the reference speed must be above 0");
    }
    const Length exact_cell{ reference_length / reference_cells };
    const Length cell{ nearest_float_cell(exact_cell) }; // the core's cell: the derived cell as a float, by definition
    const double cells_per_step{ mach.lattice_speed() };
    const Duration cell_crossing{ cell / reference_speed };
    const Duration step{ cell_crossing * cells_per_step };
    const LatticeConverter converter(cell, step);
    return converter;
}

LatticeConverter LatticeConverter::with_density(Density density) const {
    if(!(density > au::ZERO)) {
        throw SetupError("si::LatticeConverter::with_density(): the density must be above 0 kg/m3");
    }
    LatticeConverter copy{ *this };
    copy.density_ = density;
    return copy;
}

LatticeConverter LatticeConverter::with_theta_0(Temperature theta_0) const {
    LatticeConverter copy{ with_temperature_scale(theta_0, kernel_temperature_unit) };
    return copy;
}

LatticeConverter LatticeConverter::with_temperature_scale(Temperature reference, TemperatureDifference unit) const {
    if(!(unit > au::ZERO)) {
        throw SetupError("si::LatticeConverter::with_temperature_scale(): the temperature unit must be above 0 K");
    }
    LatticeConverter copy{ *this };
    copy.theta_0_ = reference;
    copy.temperature_unit_ = unit;
    return copy;
}

Temperature LatticeConverter::theta_0() const {
    if(!theta_0_) {
        throw SetupError("si::LatticeConverter: no theta_0 (a converter without a temperature scale)");
    }
    const Temperature theta{ *theta_0_ };
    return theta;
}

Density LatticeConverter::density() const {
    if(!density_) {
        throw SetupError("si::LatticeConverter: no reference density (a converter without with_density())");
    }
    const Density rho{ *density_ };
    return rho;
}

Mass LatticeConverter::mass_unit() const {
    const Density rho{ density() };
    const Volume cell_volume{ cell_ * cell_ * cell_ };
    const Mass mass{ rho * cell_volume };
    return mass;
}

CoreUnits LatticeConverter::core_units() const {
    const double metres{ cell_.in(au::meters) };
    const double kilograms_per_cell{ mass_unit().in(kilograms) };
    const double seconds{ step_.in(au::seconds) };
    const CoreUnits units{ .m = static_cast<float>(metres), .kg = static_cast<float>(kilograms_per_cell), .s = static_cast<float>(seconds) };
    return units;
}

float LatticeConverter::velocity(Speed u) const {
    const auto distance = u * step_;
    const auto ratio = distance / cell_;
    const double cells_per_step{ au::as_raw_number(ratio) };
    const float lattice{ static_cast<float>(cells_per_step) };
    return lattice;
}

float LatticeConverter::acceleration(Acceleration a) const {
    const auto speed = a * step_;
    const auto distance = speed * step_;
    const auto ratio = distance / cell_;
    const double cells_per_step_squared{ au::as_raw_number(ratio) };
    const float lattice{ static_cast<float>(cells_per_step_squared) };
    return lattice;
}

float LatticeConverter::viscosity(KinematicViscosity nu) const {
    const auto area = nu * step_;
    const auto cell_area = cell_ * cell_;
    const auto ratio = area / cell_area;
    const double cells_squared_per_step{ au::as_raw_number(ratio) };
    const float lattice{ static_cast<float>(cells_squared_per_step) };
    return lattice;
}

float LatticeConverter::per_step(Duration tau) const {
    if(!(tau > au::ZERO)) {
        throw SetupError("si::LatticeConverter::per_step(): the time scale must be above 0 s");
    }
    const auto ratio = step_ / tau;
    const double rate{ au::as_raw_number(ratio) };
    const float lattice{ static_cast<float>(rate) };
    return lattice;
}

float LatticeConverter::frequency(Frequency f) const {
    const auto product = f * step_;
    const double per_step_value{ au::as_raw_number(product) };
    const float lattice{ static_cast<float>(per_step_value) };
    return lattice;
}

float LatticeConverter::angular_rate(AngularSpeed omega) const {
    const Angle per_step_angle{ omega * step_ };
    const double radians{ per_step_angle.in(au::radians) };
    const float lattice{ static_cast<float>(radians) };
    return lattice;
}

std::uint64_t LatticeConverter::steps_of(Duration t, Duration step) {
    const auto ratio = t / step;
    const double steps_exact{ au::as_raw_number(ratio) };
    const double steps_rounded_up{ steps_exact + half_step };
    const double steps_clamped{ std::max(steps_rounded_up, half_step) };
    const std::uint64_t step_count{ static_cast<std::uint64_t>(steps_clamped) };
    return step_count;
}

double LatticeConverter::lattice_pressure(Pressure p) const {
    const Density rho{ density() };
    const auto step_squared = step_ * step_;
    const auto cell_squared = cell_ * cell_;
    const auto numerator = p * step_squared;
    const auto denominator = rho * cell_squared;
    const auto ratio = numerator / denominator;
    const double p_lattice{ au::as_raw_number(ratio) };
    return p_lattice;
}

float LatticeConverter::pressure(Pressure p) const {
    const double p_lattice{ lattice_pressure(p) };
    const float lattice{ static_cast<float>(p_lattice) };
    return lattice;
}

float LatticeConverter::lattice_density(Pressure p) const {
    const double p_lattice{ lattice_pressure(p) };
    const double rho_lattice{ 1.0 + pressure_to_density * p_lattice };
    const float lattice{ static_cast<float>(rho_lattice) };
    return lattice;
}

Pressure LatticeConverter::si_pressure(double p_lattice) const {
    const Density rho{ density() };
    const auto unit_speed = lattice_speed();
    const auto unit_pressure = rho * unit_speed * unit_speed;
    const Pressure p{ unit_pressure * p_lattice };
    return p;
}

Density LatticeConverter::si_density(double rho_lattice) const {
    const Density rho{ density() };
    const Density value{ rho * rho_lattice };
    return value;
}

Force LatticeConverter::si_force(double f_lattice) const {
    const Mass mass{ mass_unit() };
    const auto unit_acceleration = cell_ / (step_ * step_);
    const auto unit_force = mass * unit_acceleration;
    const Force force{ unit_force * f_lattice };
    return force;
}

float LatticeConverter::surface_tension(SurfaceTension sigma) const {
    const Mass mass{ mass_unit() };
    const auto step_squared = step_ * step_;
    const auto numerator = sigma * step_squared;
    const auto ratio = numerator / mass;
    const double lattice_sigma{ au::as_raw_number(ratio) };
    const float lattice{ static_cast<float>(lattice_sigma) };
    return lattice;
}

SurfaceTension LatticeConverter::si_surface_tension(double sigma_lattice) const {
    const Mass mass{ mass_unit() };
    const auto step_squared = step_ * step_;
    const auto unit_tension = mass / step_squared;
    const SurfaceTension sigma{ unit_tension * sigma_lattice };
    return sigma;
}

float LatticeConverter::temperature(Temperature t) const {
    const Temperature theta{ theta_0() };
    const TemperatureDifference excess{ t - theta };
    const float lattice{ temperature_excess(excess) };
    return lattice;
}

float LatticeConverter::temperature_excess(TemperatureDifference excess) const {
    const auto ratio = excess / temperature_unit_;
    const double unit_count{ au::as_raw_number(ratio) };
    const double lattice_double{ 1.0 + unit_count };
    const float lattice{ static_cast<float>(lattice_double) };
    return lattice;
}

float LatticeConverter::expansion(ThermalExpansion beta) const {
    const auto product = beta * temperature_unit_;
    const double expansion_double{ au::as_raw_number(product) };
    const float lattice{ static_cast<float>(expansion_double) };
    return lattice;
}

float LatticeConverter::heat_rate(KinematicHeatFlux flux) const {
    const auto per_area = flux * step_;
    const auto per_cell = per_area / cell_;
    const auto ratio = per_cell / temperature_unit_;
    const double rate{ au::as_raw_number(ratio) };
    const float lattice{ static_cast<float>(rate) };
    return lattice;
}

float LatticeConverter::moisture(WaterContent qt) const {
    const auto ratio = qt / moisture_unit;
    const double unit_count{ au::as_raw_number(ratio) };
    const double lattice_double{ 1.0 + unit_count };
    const float lattice{ static_cast<float>(lattice_double) };
    return lattice;
}

float LatticeConverter::moisture_rate(MoistureFlux flux) const {
    const auto per_area = flux * step_;
    const auto per_cell = per_area / cell_;
    const auto ratio = per_cell / moisture_unit;
    const double rate{ au::as_raw_number(ratio) };
    const float lattice{ static_cast<float>(rate) };
    return lattice;
}

} // namespace si
