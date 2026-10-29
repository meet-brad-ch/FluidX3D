#pragma once

#include "setup/units/units.hpp"

/// @file moist_air_physics.hpp
/// @brief The moist-air model of src/moist_air.hpp with typed values. This is the only place that calls src/moist_air.hpp:
/// it gives the model its numbers in K, kg/kg and Pa and makes quantities of its results.

namespace si {

/// @brief The state of a parcel of air after the saturation adjustment: its temperature and how much of its water
/// is liquid.
struct AdjustedAir {
    Temperature temperature{ kelvin_point(0.0) }; ///< the absolute temperature T (not a potential temperature)
    WaterContent liquid{ au::ZERO };               ///< the liquid water ql in kg of water per kg of air, 0 or more
};

/// @brief The thermodynamics of moist air, with typed values: the saturation adjustment, the virtual potential
/// temperature and the hydrostatic pressure.
///
/// The model is src/moist_air.hpp (Bolton 1980's saturation pressure, the saturation adjustment by Newton
/// iteration). This class only converts: it gives the model its numbers in K, kg/kg and Pa, and makes quantities of
/// its results. The OpenCL kernel holds the same saturation adjustment (MOISTURE).
class MoistAirPhysics {
public:
    /// @brief Finds the temperature and the liquid water of a parcel: water above the saturation content condenses,
    /// and its latent heat warms the parcel (Newton iteration, at most 20 steps, until the residual is below 1e-6 K).
    /// @param theta_l the liquid water potential temperature of the parcel
    /// @param total_water the parcel's total water qt (vapour plus liquid), in kg of water per kg of air
    /// @param pressure the parcel's absolute pressure, more than 0
    /// @return the absolute temperature and the liquid water; the liquid water is 0 for an unsaturated parcel
    static AdjustedAir saturation_adjust(Temperature theta_l, WaterContent total_water, Pressure pressure);

    /// @brief Gives only the liquid water of the saturation adjustment (see saturation_adjust()).
    /// @param theta_l the liquid water potential temperature of the parcel
    /// @param total_water the parcel's total water qt (vapour plus liquid), in kg of water per kg of air
    /// @param pressure the parcel's absolute pressure, more than 0
    /// @return the liquid water ql, 0 for an unsaturated parcel
    static WaterContent liquid(Temperature theta_l, WaterContent total_water, Pressure pressure);

    /// @brief Gives the virtual potential temperature of a parcel, with the weight of its condensate:
    /// theta_v = theta (1 + 0.61 (qt - ql) - ql), theta = T / Exner(p), after the saturation adjustment.
    /// @param theta_l the liquid water potential temperature of the parcel
    /// @param total_water the parcel's total water qt (vapour plus liquid), in kg of water per kg of air
    /// @param pressure the parcel's absolute pressure, more than 0
    /// @return the virtual potential temperature (the buoyancy's temperature)
    static Temperature virtual_theta(Temperature theta_l, WaterContent total_water, Pressure pressure);

    /// @brief Gives the hydrostatic pressure in a column of one potential temperature, in closed form:
    /// p = p0 (1 - g h / (cp theta))^(cp / Rd). For a dry mixed layer it is exact; above the inversion its error is
    /// a few hPa.
    /// @param height the height above the ground, below cp theta / g (about 30 km at 300 K)
    /// @param theta the potential temperature of the column, more than 0 K
    /// @param ground the absolute pressure at the ground
    /// @return the absolute pressure at the height
    /// @throws SetupError when the height is not below cp theta / g (there the pressure is 0, above it not a number)
    static Pressure isentropic_pressure(Length height, Temperature theta, Pressure ground);

    /// @brief Gives the pressure one step up in a hydrostatic integration, as the MicroHH reference runs integrate:
    /// T = theta_l (p / 1000 hPa)^(Rd/cp) (1 + 0.61 qt) holds over the step, with p the given (lower) pressure, and
    /// p_upper = p exp(-g rise / (Rd T)). The function uses theta_l and qt as given; MoistColumn::hydrostatic() passes
    /// the values of the upper level of the step.
    /// @param pressure the absolute pressure at the lower level
    /// @param theta_l the liquid water potential temperature that holds over the step (MoistColumn: the upper level's)
    /// @param total_water the total water qt that holds over the step, in kg of water per kg of air (MoistColumn: the
    /// upper level's)
    /// @param rise the height of the step (negative: a step down)
    /// @return the absolute pressure at the upper level
    static Pressure hydrostatic_step(Pressure pressure, Temperature theta_l, WaterContent total_water, Length rise);
};

} // namespace si
