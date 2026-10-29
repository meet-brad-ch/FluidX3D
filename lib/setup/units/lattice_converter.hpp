#pragma once

#include "setup/units/lattice_mach.hpp"
#include "setup/units/units.hpp"

#include <cstdint>
#include <optional>

/// @file lattice_converter.hpp
/// @brief The one boundary between the Setup API's quantities and the LBM core's lattice numbers.

namespace si {

/// @brief A diffusivity in lattice units (cells squared per time step): the form of a numerical diffusivity that
/// scales with the grid.
///
/// At another cell size and time step the same lattice value is another physical diffusivity, D = value dx²/dt.
/// LatticeConverter::physical() gives the physical value of a run.
class LatticeDiffusivity {
public:
    /// @brief Makes a lattice diffusivity from its value.
    /// @param value the diffusivity in cells squared per time step
    constexpr explicit LatticeDiffusivity(double value) : value_(value) {}
    /// @return the diffusivity in cells squared per time step
    constexpr double value() const { return value_; }
    /// @brief Compares two lattice diffusivities by their values.
    /// @return the ordering of the values
    constexpr auto operator<=>(const LatticeDiffusivity&) const = default;

private:
    double value_{}; ///< the diffusivity in cells squared per time step
};

/// @brief The numbers that the core's global units object takes (Units::set_m_kg_s()): the cell size in m, the mass
/// of one cell at the lattice density 1 in kg, and the time step in s, each a float.
struct CoreUnits {
    float m{};  ///< the cell size in m
    float kg{}; ///< the mass of one cell at the lattice density 1, in kg
    float s{};  ///< the time step in s
};

/// @brief The conversions between the Setup API's quantities and the core's lattice numbers.
///
/// Every lattice value that the core gets is made here: in double, and cast to float once at the return. Every value
/// that the core gives back becomes a quantity here. No other class of the Setup API converts.
///
/// The lattice: the cell is 1, the time step is 1, and the lattice density 1 is the reference density. The lattice
/// pressure is (ρ - 1)/3. The lattice temperature is 1 at the reference temperature theta_0 and changes by 1 per
/// temperature unit (1 K unless set with with_temperature_scale()). The lattice moisture Q is 1 + qt / 0.01 kg/kg
/// (the kernel's def_q_scale).
///
/// The core holds the cell and the time step as floats, so the converter does too: the cell must be a float exactly
/// (else the constructor refuses it), and the time step is held as the float nearest to the given one. Every
/// converter of the same cell and step then gives the core's own numbers.
///
/// The parts that a run needs are added by copies: with_density() for pressures, forces and surface tension;
/// with_theta_0() or with_temperature_scale() for temperatures. A conversion that needs a missing part throws.
/// @code
/// using namespace si::symbols;
/// const si::LatticeConverter c = si::LatticeConverter(10.0 * m, 0.16 * s).with_density(1.225 * kg / (m * m * m));
/// const float u = c.velocity(5.0 * m / s);   // cells per time step
/// const float p = c.pressure(100.0 * Pa);     // lattice pressure
/// @endcode
class LatticeConverter {
public:
    /// The water content of one unit of the lattice moisture Q (the kernel's def_q_scale).
    static constexpr WaterContent moisture_unit = (kilograms_of_water / kilograms)(0.01);
    /// The temperature difference of one lattice temperature unit that the moist kernels assume (def_T_scale): the
    /// Boussinesq buoyancy and the moisture need a converter with this unit.
    static constexpr TemperatureDifference kernel_temperature_unit = au::kelvins(1.0);

    /// @brief Makes the converter of a cell size and a time step, without a density or a temperature scale.
    /// @param cell the cell size, more than 0 and a float exactly (2.5 m and 10 m are, 3.3 m is not)
    /// @param step the time step, more than 0 (held as the nearest float, as the core holds it)
    /// @throws SetupError when the cell or the step is not more than 0, or the cell is not a float exactly
    LatticeConverter(Length cell, Duration step);
    /// @brief Makes the converter of a cell size and a time step, with theta_0 at the lattice temperature 1 and a
    /// temperature unit of 1 K.
    /// @param cell the cell size, more than 0 and a float exactly
    /// @param step the time step, more than 0 (held as the nearest float)
    /// @param theta_0 the temperature that is the lattice temperature 1
    /// @throws SetupError when the cell or the step is not more than 0, or the cell is not a float exactly
    LatticeConverter(Length cell, Duration step, Temperature theta_0);

    /// @brief Makes the converter of a reference length and speed at a lattice Mach number: the cell is the
    /// reference length over its cells, and the step makes the reference speed the Mach number's lattice speed.
    ///
    /// The cell is derived, so it is rounded to the nearest float (the core's cell); that rounding is the definition
    /// of the cell here, not a fallback. step = mach.lattice_speed() cell / reference_speed, held as a float.
    /// @param reference_length a length of the model or the box, more than 0
    /// @param reference_cells that length in cells, more than 0 (the domain planner's number)
    /// @param reference_speed the flow's reference speed, more than 0
    /// @param mach the lattice Mach number of the reference speed
    /// @return the converter, without a density or a temperature scale
    /// @throws SetupError when a value is not more than 0
    static LatticeConverter from_reference(Length reference_length, double reference_cells, Speed reference_speed, LatticeMach mach);

    /// @brief Gives a copy with a reference density: the fluid's density at the lattice density 1.
    /// @param density the reference density, more than 0
    /// @return the copy, which can convert pressures, forces and surface tension
    /// @throws SetupError when the density is not more than 0
    LatticeConverter with_density(Density density) const;
    /// @brief Gives a copy with theta_0 at the lattice temperature 1 and a temperature unit of 1 K (the convection
    /// API's scale).
    /// @param theta_0 the temperature that is the lattice temperature 1
    /// @return the copy
    LatticeConverter with_theta_0(Temperature theta_0) const;
    /// @brief Gives a copy with a temperature scale: the reference is the lattice temperature 1, and one unit is
    /// the lattice temperature difference 1. A cold and a hot wall at reference -/+ unit/2 become 0.5 and 1.5.
    /// @param reference the temperature that is the lattice temperature 1
    /// @param unit the temperature difference that is the lattice temperature difference 1, more than 0
    /// @return the copy
    /// @throws SetupError when the unit is not more than 0
    LatticeConverter with_temperature_scale(Temperature reference, TemperatureDifference unit) const;

    /// @return the cell size (a float exactly)
    Length cell() const { return cell_; }
    /// @return the time step (the core's float)
    Duration step() const { return step_; }
    /// @brief Gives the temperature that is the lattice temperature 1.
    /// @return theta_0
    /// @throws SetupError for a converter without a temperature scale
    Temperature theta_0() const;
    /// @return the temperature difference that is the lattice temperature difference 1 (1 K by default)
    TemperatureDifference temperature_unit() const { return temperature_unit_; }
    /// @brief Gives the reference density: the density at the lattice density 1.
    /// @return the density
    /// @throws SetupError for a converter without a density
    Density density() const;
    /// @brief Gives the mass of one cell at the lattice density 1: the reference density times the cell's volume.
    /// @return the mass
    /// @throws SetupError for a converter without a density
    Mass mass_unit() const;
    /// @brief Gives the numbers for the core's global units object (Units::set_m_kg_s()).
    /// @return the cell in m, the mass unit in kg and the step in s, each a float
    /// @throws SetupError for a converter without a density
    CoreUnits core_units() const;
    /// @return the speed of one cell per time step
    Speed lattice_speed() const { return cell_ / step_; }

    /// @brief Converts a length to cells.
    /// @param x a length
    /// @return the length in cells (a double: a cell count or a lattice coordinate)
    double cells(Length x) const {
        const auto ratio = x / cell_;
        const double cell_count{ au::as_raw_number(ratio) };
        return cell_count;
    }
    /// @brief Converts a length in cells to metres.
    /// @param cells a length in cells
    /// @return the length
    Length si_length(double cells) const {
        const Length length{ cell_ * cells };
        return length;
    }
    /// @brief Converts a speed to lattice units.
    /// @param u a speed
    /// @return the speed in cells per time step
    float velocity(Speed u) const;
    /// @brief Converts a lattice speed to m/s.
    /// @param u a speed in cells per time step
    /// @return the speed
    Speed si_velocity(double u) const {
        const Speed unit_speed{ lattice_speed() };
        const Speed speed{ unit_speed * u };
        return speed;
    }
    /// @brief Converts an acceleration to lattice units. For a volume force such as gravity this is also the force
    /// per lattice volume, because the lattice density is 1.
    /// @param a an acceleration
    /// @return the acceleration in cells per time step squared
    float acceleration(Acceleration a) const;
    /// @brief Converts a kinematic viscosity or a diffusivity to lattice units.
    /// @param nu a kinematic viscosity or a diffusivity
    /// @return the value in cells squared per time step
    float viscosity(KinematicViscosity nu) const;
    /// @brief Gives the physical diffusivity that a lattice diffusivity is at this cell and step.
    /// @param d a lattice diffusivity
    /// @return d dx²/dt
    KinematicViscosity physical(LatticeDiffusivity d) const {
        const auto cell_area = cell_ * cell_;
        const auto unit_diffusivity = cell_area / step_;
        const double lattice_value{ d.value() };
        const KinematicViscosity diffusivity{ unit_diffusivity * lattice_value };
        return diffusivity;
    }
    /// @brief Gives the part of a time scale that one time step is: the rate per step of a relaxation or a damping.
    /// @param tau the time scale, more than 0
    /// @return dt / tau
    /// @throws SetupError when the time scale is not more than 0
    float per_step(Duration tau) const;
    /// @brief Converts a frequency to cycles (or events) per time step.
    /// @param f a frequency, 0 or more
    /// @return f dt
    float frequency(Frequency f) const;
    /// @brief Converts a rotation rate to radians per time step.
    /// @param omega a rotation rate
    /// @return the rotation per time step in radians
    float angular_rate(AngularSpeed omega) const;
    /// @brief Gives the number of time steps in a duration, as the core counts them.
    /// @param t a duration
    /// @return t / dt, rounded to the nearest whole step (half a step up; the core's to_ulong()); 0 for t below half a step
    std::uint64_t steps(Duration t) const {
        const std::uint64_t step_count{ steps_of(t, step_) };
        return step_count;
    }
    /// @brief Gives the number of steps of a given length in a duration, as the core counts them.
    /// @param t a duration
    /// @param step a time step, more than 0
    /// @return t / step, rounded to the nearest whole step (half a step up); 0 for t below half a step
    static std::uint64_t steps_of(Duration t, Duration step);
    /// @brief Gives the duration of a number of time steps.
    /// @param steps a number of time steps
    /// @return steps dt
    Duration time(std::uint64_t steps) const {
        const double step_count{ static_cast<double>(steps) };
        const Duration duration{ step_ * step_count };
        return duration;
    }

    /// @brief Converts a pressure difference from the fluid at rest to the lattice pressure.
    /// @param p a pressure difference
    /// @return p dt² / (ρ dx²), the lattice pressure (ρ_lattice - 1)/3
    /// @throws SetupError for a converter without a density
    float pressure(Pressure p) const;
    /// @brief Gives the lattice density of a pressure difference from the fluid at rest: 1 + 3 times the lattice
    /// pressure.
    /// @param p a pressure difference
    /// @return the lattice density
    /// @throws SetupError for a converter without a density
    float lattice_density(Pressure p) const;
    /// @brief Converts a lattice pressure to a pressure difference from the fluid at rest.
    /// @param p_lattice a lattice pressure, (ρ_lattice - 1)/3
    /// @return the pressure difference
    /// @throws SetupError for a converter without a density
    Pressure si_pressure(double p_lattice) const;
    /// @brief Converts a lattice density to a density.
    /// @param rho_lattice a lattice density (1 is the reference density)
    /// @return the density
    /// @throws SetupError for a converter without a density
    Density si_density(double rho_lattice) const;
    /// @brief Converts a lattice force to a force.
    /// @param f_lattice a force in lattice units (mass unit times cells per time step squared)
    /// @return the force
    /// @throws SetupError for a converter without a density
    Force si_force(double f_lattice) const;
    /// @brief Converts a surface tension to lattice units.
    /// @param sigma a surface tension
    /// @return sigma dt² / mass unit
    /// @throws SetupError for a converter without a density
    float surface_tension(SurfaceTension sigma) const;
    /// @brief Converts a lattice surface tension to a surface tension.
    /// @param sigma_lattice a surface tension in lattice units
    /// @return the surface tension
    /// @throws SetupError for a converter without a density
    SurfaceTension si_surface_tension(double sigma_lattice) const;

    /// @brief Converts a temperature reading to the lattice temperature.
    /// @param t a temperature reading
    /// @return 1 + (t - theta_0) / the temperature unit
    /// @throws SetupError for a converter without a temperature scale
    float temperature(Temperature t) const;
    /// @brief Converts a temperature above theta_0 to the lattice temperature.
    /// @param excess a temperature difference from theta_0
    /// @return 1 + excess / the temperature unit
    float temperature_excess(TemperatureDifference excess) const;
    /// @brief Converts a lattice temperature to the temperature above theta_0 that it stands for.
    /// @param lattice_t a lattice temperature
    /// @return (lattice_t - 1) times the temperature unit
    TemperatureDifference si_excess(double lattice_t) const {
        const double unit_count{ lattice_t - 1.0 };
        const TemperatureDifference excess{ temperature_unit_ * unit_count };
        return excess;
    }
    /// @brief Converts a lattice temperature to the temperature reading that it stands for.
    /// @param lattice_t a lattice temperature
    /// @return theta_0 + (lattice_t - 1) times the temperature unit
    /// @throws SetupError for a converter without a temperature scale
    Temperature si_temperature(double lattice_t) const {
        const Temperature theta{ theta_0() };
        const TemperatureDifference excess{ si_excess(lattice_t) };
        const Temperature reading{ theta + excess };
        return reading;
    }
    /// @brief Converts a thermal expansion coefficient to the core's lattice coefficient.
    /// @param beta a thermal expansion coefficient
    /// @return beta times the temperature unit
    float expansion(ThermalExpansion beta) const;
    /// @brief Gives the lattice temperature that a floor heat source adds per time step to a layer one cell thick:
    /// a kinematic heat flux q through its floor raises it by q dt / dx.
    /// @param flux the floor's kinematic heat flux
    /// @return q dt / (dx times the temperature unit), the core's heat_rate per time step and source cell
    float heat_rate(KinematicHeatFlux flux) const;

    /// @brief Converts a total water content to the lattice moisture.
    /// @param qt a total water content
    /// @return Q = 1 + qt / 0.01 kg/kg
    float moisture(WaterContent qt) const;
    /// @brief Converts a lattice moisture to the total water that it stands for.
    /// @param lattice_q a lattice moisture value
    /// @return (Q - 1) times 0.01 kg/kg
    WaterContent water(double lattice_q) const {
        const double unit_count{ lattice_q - 1.0 };
        const WaterContent total_water{ moisture_unit * unit_count };
        return total_water;
    }
    /// @brief Gives the lattice moisture that a floor source adds per time step to a layer one cell thick: a
    /// moisture flux m through its floor raises qt by m dt / dx.
    /// @param flux the floor's moisture flux
    /// @return m dt / (dx times 0.01 kg/kg), the core's moisture_rate per time step and source cell
    float moisture_rate(MoistureFlux flux) const;

private:
    /// @brief Gives the time step as the core holds it.
    /// @param step a time step
    /// @return the float nearest to the step, as a duration
    static Duration nearest_float_step(Duration step);
    /// @brief Gives a cell size as the core holds it.
    /// @param cell a cell size
    /// @return the float nearest to the cell size, as a length
    static Length nearest_float_cell(Length cell);
    /// @brief Converts a pressure difference to the lattice pressure, in double (pressure() and lattice_density()
    /// cast it once).
    /// @param p a pressure difference from the fluid at rest
    /// @return p dt² / (ρ dx²)
    /// @throws SetupError for a converter without a density
    double lattice_pressure(Pressure p) const;

    Length cell_{ au::ZERO };                                     ///< the cell size (a float exactly)
    Duration step_{ au::ZERO };                                   ///< the time step (the core's float)
    std::optional<Temperature> theta_0_{};                        ///< the lattice temperature 1, or none (no temperature conversions)
    TemperatureDifference temperature_unit_{ kernel_temperature_unit }; ///< the lattice temperature difference 1
    std::optional<Density> density_{};                            ///< the density at the lattice density 1, or none (no pressure, force or surface tension conversions)
};

} // namespace si
