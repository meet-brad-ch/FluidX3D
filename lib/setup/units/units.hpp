#pragma once

#include "au/au.hh"
#include <cstdint>
#include "au/units/bytes.hh"
#include "au/units/celsius.hh"
#include "au/units/degrees.hh"
#include "au/units/grams.hh"
#include "au/units/hertz.hh"
#include "au/units/hours.hh"
#include "au/units/joules.hh"
#include "au/units/kelvins.hh"
#include "au/units/meters.hh"
#include "au/units/minutes.hh"
#include "au/units/newtons.hh"
#include "au/units/pascals.hh"
#include "au/units/radians.hh"
#include "au/units/seconds.hh"

/// @file units.hpp
/// @brief The physical quantities of the Setup API, on the Au units library (aurora-opensource/au).
///
/// Every value with a unit is one of these types: in signatures, members, returns and constants. The values are
/// double. A raw number appears only where the LBM core, src/moist_air.hpp or a file format needs one:
/// si::LatticeConverter makes the core's floats, units/moist_air_physics.hpp calls the moist air functions, and the
/// output writers print numbers. Call sites write the physics with the symbols:
/// @code
/// using namespace si::symbols;
/// const si::Length lid = 2048.0 * m;
/// const si::Speed cruise = 300.0 * kmh;
/// const si::KinematicHeatFlux flux = 0.24 * K * m / s;
/// const si::WaterContent qt = 8.0 * g_w / kg;
/// const si::Temperature theta_0 = si::kelvin_point(300.0);
/// const si::MemoryAmount budget = 2000.0 * MiB;
/// @endcode
///
/// Write a double literal (2.0 * m, not 2 * m): an int literal makes a quantity that stores an int, and a float
/// literal one that stores a float.
///
/// The namespace is si, not units: the core has a global object named units. There is no symbol for minutes or hours:
/// `min` would be ambiguous with the core's min() functions (use au::minutes(1.0), au::hours(1.0)).

/// @brief The Setup API's physical quantities, their units and their symbols.
namespace si {

using Kilograms = au::Kilo<au::Grams>; ///< the unit of mass (of the air, or of a body)

/// @brief The ids of the Setup API's own base dimensions. Au orders the base dimensions of a dimension by their ids, so
/// each id must be unique in the program; these are far from the ids of Au's own base dimensions.
struct BaseDimensionIds {
    static constexpr std::int64_t water_mass = 1761609600;   ///< the id of WaterMass
    static constexpr std::int64_t lattice_work = 1761609601; ///< the id of LatticeWork
};

/// @brief The dimension of the water in the air, a base dimension of its own: kilograms of water per kilogram of air
/// are not a plain number, and a moisture flux is not a speed.
struct WaterMass : au::base_dim::BaseDimension<BaseDimensionIds::water_mass> {};

/// @brief A kilogram of water (the water substance, not the air that holds it).
struct KilogramsOfWater : au::UnitImpl<au::Dimension<WaterMass>> {
    static constexpr inline const char label[] = "kg_w"; ///< the printed label
};

/// @brief The dimension of the lattice's work: one cell updated for one time step.
struct LatticeWork : au::base_dim::BaseDimension<BaseDimensionIds::lattice_work> {};

/// @brief One cell updated for one time step.
struct LatticeUpdates : au::UnitImpl<au::Dimension<LatticeWork>> {
    static constexpr inline const char label[] = "LU"; ///< the printed label
};

using Length = au::QuantityD<au::Meters>;                                                           ///< a length or a distance, stored in m
using Area = au::QuantityD<au::UnitPowerT<au::Meters, 2>>;                                          ///< an area, stored in m²
using Volume = au::QuantityD<au::UnitPowerT<au::Meters, 3>>;                                        ///< a volume, stored in m³
using Duration = au::QuantityD<au::Seconds>;                                                        ///< a time span, stored in s
using Frequency = au::QuantityD<au::UnitInverseT<au::Seconds>>;                                     ///< a rate or a frequency, stored in 1/s
using Speed = au::QuantityD<au::UnitQuotientT<au::Meters, au::Seconds>>;                            ///< a speed, stored in m/s
using Acceleration = au::QuantityD<au::UnitQuotientT<au::Meters, au::UnitPowerT<au::Seconds, 2>>>;  ///< an acceleration, stored in m/s²
using Mass = au::QuantityD<Kilograms>;                                                              ///< a mass, stored in kg
using Density = au::QuantityD<au::UnitQuotientT<Kilograms, au::UnitPowerT<au::Meters, 3>>>;         ///< a mass density, stored in kg/m³
using Force = au::QuantityD<au::Newtons>;                                                           ///< a force, stored in N
using Pressure = au::QuantityD<au::Pascals>;                                                        ///< a pressure, stored in Pa
using SurfaceTension = au::QuantityD<au::UnitQuotientT<au::Newtons, au::Meters>>;                   ///< a surface tension, stored in N/m
using VolumeFlowRate = au::QuantityD<au::UnitQuotientT<au::UnitPowerT<au::Meters, 3>, au::Seconds>>; ///< a volume per time, stored in m³/s
using KinematicViscosity = au::QuantityD<au::UnitQuotientT<au::UnitPowerT<au::Meters, 2>, au::Seconds>>; ///< a kinematic viscosity or a diffusivity, stored in m²/s
using Temperature = au::QuantityPointD<au::Kelvins>;                                                ///< a temperature reading (a point on the scale), stored in K
using TemperatureDifference = au::QuantityD<au::Kelvins>;                                           ///< a temperature difference, stored in K
using TemperatureVariance = au::QuantityD<au::UnitPowerT<au::Kelvins, 2>>;                          ///< a temperature variance, stored in K²
using LapseRate = au::QuantityD<au::UnitQuotientT<au::Kelvins, au::Meters>>;                        ///< a temperature gradient in height, stored in K/m
using ThermalExpansion = au::QuantityD<au::UnitInverseT<au::Kelvins>>;                              ///< a thermal expansion coefficient, stored in 1/K
using KinematicHeatFlux = au::QuantityD<decltype(au::Kelvins{} * au::Meters{} / au::Seconds{})>;    ///< a heat flux over ρ cp, stored in K m/s
using SpeedVariance = au::QuantityD<au::UnitPowerT<au::UnitQuotientT<au::Meters, au::Seconds>, 2>>; ///< a speed variance, stored in m²/s²
using SpeedCubed = au::QuantityD<au::UnitPowerT<au::UnitQuotientT<au::Meters, au::Seconds>, 3>>;    ///< a third moment of a speed, stored in m³/s³
using WaterContent = au::QuantityD<au::UnitQuotientT<KilogramsOfWater, Kilograms>>;                 ///< kg of water per kg of air
using MoistureFlux = au::QuantityD<decltype(KilogramsOfWater{} / Kilograms{} * au::Meters{} / au::Seconds{})>; ///< a water flux over ρ, stored in kg/kg m/s
using SpecificHeat = au::QuantityD<decltype(au::Joules{} / (Kilograms{} * au::Kelvins{}))>;        ///< a specific heat of the air, stored in J/(kg K)
using LatentHeat = au::QuantityD<au::UnitQuotientT<au::Joules, KilogramsOfWater>>;                  ///< a latent heat, stored in J per kg of water
using Angle = au::QuantityD<au::Degrees>;                                                           ///< an angle or a direction, stored in degrees
using AngularSpeed = au::QuantityD<au::UnitQuotientT<au::Degrees, au::Seconds>>;                    ///< a rotation rate, stored in degrees/s
using MemoryAmount = au::QuantityD<au::Bytes>;                                                      ///< an amount of device memory, stored in bytes
using LatticeWorkAmount = au::QuantityD<LatticeUpdates>;                                            ///< cells times time steps
using LatticeUpdateRate = au::QuantityD<au::UnitQuotientT<au::Mega<LatticeUpdates>, au::Seconds>>;  ///< a lattice speed in million cell updates per second (MLUPs)

constexpr auto kilograms = au::QuantityMaker<Kilograms>{};                    ///< maker of a mass in kg: si::kilograms(1.225)
constexpr auto kilograms_of_water = au::QuantityMaker<KilogramsOfWater>{};    ///< maker of a water mass in kg of water
constexpr auto lattice_updates = au::QuantityMaker<LatticeUpdates>{};         ///< maker of an amount of lattice work in cell updates
constexpr auto mebibytes = au::mebi(au::bytes);                               ///< maker of a memory amount in MiB (2^20 bytes, as the core counts MB)

/// @brief The unit symbols for call sites: 400.0 * m, 0.262 * K * m / s, 8.0 * g_w / kg, 2000.0 * MLUPs.
///
/// Use them in .cpp files and in function bodies (using namespace si::symbols;), not at namespace scope in a header.
/// The memory symbols are binary: MiB is 2^20 bytes and GiB 2^30 bytes, as the core counts its MB and GB. There is no
/// MB symbol: Au's MB is 10^6 bytes, and a budget written in it would be 4.6 % smaller than the core's MB.
namespace symbols {
using au::symbols::deg;
using au::symbols::Hz;
using au::symbols::K;
using au::symbols::m;
using au::symbols::N;
using au::symbols::Pa;
using au::symbols::rad;
using au::symbols::s;
constexpr auto kg = au::SymbolFor<Kilograms>{};                                   ///< kilogram (of air, or of a body)
constexpr auto kg_w = au::SymbolFor<KilogramsOfWater>{};                          ///< kilogram of water
constexpr auto g_w = au::SymbolFor<au::Milli<KilogramsOfWater>>{};                ///< gram of water
constexpr auto J = au::SymbolFor<au::Joules>{};                                   ///< joule
constexpr auto hPa = au::SymbolFor<au::Hecto<au::Pascals>>{};                     ///< hectopascal
constexpr auto km = au::SymbolFor<au::Kilo<au::Meters>>{};                        ///< kilometre
constexpr auto cm = au::SymbolFor<au::Centi<au::Meters>>{};                       ///< centimetre
constexpr auto mm = au::SymbolFor<au::Milli<au::Meters>>{};                       ///< millimetre
constexpr auto ms = au::SymbolFor<au::Milli<au::Seconds>>{};                      ///< millisecond
constexpr auto us = au::SymbolFor<au::Micro<au::Seconds>>{};                      ///< microsecond
constexpr auto kmh = au::SymbolFor<au::UnitQuotientT<au::Kilo<au::Meters>, au::Hours>>{}; ///< kilometre per hour: 300.0 * kmh
constexpr auto MiB = au::SymbolFor<au::Mebi<au::Bytes>>{};                        ///< 2^20 bytes: the core's MB
constexpr auto GiB = au::SymbolFor<au::Gibi<au::Bytes>>{};                        ///< 2^30 bytes: the core's GB
constexpr auto MLUPs = au::SymbolFor<au::UnitQuotientT<au::Mega<LatticeUpdates>, au::Seconds>>{}; ///< million cell updates per second
} // namespace symbols

/// @brief Makes a temperature reading from a number in kelvin.
/// @param kelvin the reading in K
/// @return the reading as a point on the temperature scale
constexpr Temperature kelvin_point(double kelvin) { return au::kelvins_pt(kelvin); }

/// @brief Makes a temperature reading from a number in degrees Celsius (0 °C is exactly 273.15 K).
/// @param celsius the reading in °C
/// @return the reading as a point on the temperature scale, in K
constexpr Temperature celsius_point(double celsius) { return au::celsius_pt(celsius).as(au::kelvins_pt); }

/// @brief Gives a temperature reading as a number in kelvin, for a file or the core.
/// @param t the reading
/// @return the reading in K
constexpr double in_kelvin(Temperature t) { return t.in(au::kelvins_pt); }

/// @brief Gives a temperature reading as a number in degrees Celsius, for a log.
/// @param t the reading
/// @return the reading in °C
constexpr double in_celsius(Temperature t) { return t.in(au::celsius_pt); }

} // namespace si
