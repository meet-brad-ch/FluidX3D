// Moist air on the host: a moist-air model (saturation adjustment by Newton, virtual potential temperature) used by the convective boundary layer
// example and the moisture physics tests. The OpenCL kernel holds the same saturation
// adjustment (kernel.cpp, MOISTURE); the tests compare the two. Units: K, kg/kg, Pa. Bolton 1980's saturation pressure.
#pragma once

#include <algorithm>
#include <cmath>

namespace moist_air {

constexpr double G = 9.81;               ///< m/s^2
constexpr double Rd = 287.04;            ///< J/(kg K), dry air
constexpr double Rv = 461.5;             ///< J/(kg K), water vapour
constexpr double Cp = 1004.0;            ///< J/(kg K)
constexpr double Lv = 2.5e6;             ///< J/kg
constexpr double GroundPressure = 101325.0; ///< Pa
constexpr double Eps = Rd/Rv;
constexpr double PRef = 1.0e5;           ///< Pa, the Exner reference

/// The saturation vapour pressure over water, in hPa (Bolton 1980).
inline double saturation_pressure_hpa(const double celsius) { return 6.112*std::exp(17.67*celsius/(celsius+243.5)); }

/// The saturation specific humidity at a temperature (K) and a pressure (Pa), in kg/kg.
inline double qsat(const double kelvin, const double pressure) {
	const double e = saturation_pressure_hpa(kelvin-273.15)*100.0;
	return Eps*e/(pressure-(1.0-Eps)*e);
}

/// The Exner function (p / 1000 hPa)^(Rd / cp).
inline double exner(const double pressure) { return std::pow(pressure/PRef, Rd/Cp); }

/// The temperature (K) and the liquid water (kg/kg) of air with thl and qt at a pressure, by Newton iteration.
struct Adjusted {
	double kelvin = 0.0;
	double liquid = 0.0;
};
inline Adjusted saturation_adjust(const double thl, const double qt, const double pressure) {
	const double ex = exner(pressure);
	double t = thl*ex;
	if(qt<=qsat(t, pressure)) return { t, 0.0 };
	for(int i=0; i<20; i++) {
		const double qs = qsat(t, pressure);
		const double f = t-thl*ex-Lv/Cp*(qt-qs);
		const double dqs = qs*17.67*243.5/((t-273.15+243.5)*(t-273.15+243.5));
		t -= f/(1.0+Lv/Cp*dqs);
		if(std::fabs(f)<1e-6) break;
	}
	return { t, std::max(0.0, qt-qsat(t, pressure)) };
}

/// The virtual potential temperature of air with thl and qt at a pressure, with its condensate, in K.
inline double virtual_theta(const double thl, const double qt, const double pressure) {
	const Adjusted a = saturation_adjust(thl, qt, pressure);
	const double th = a.kelvin/exner(pressure);
	return th*(1.0+0.61*(qt-a.liquid)-a.liquid);
}

/// The surface moisture flux (kg/kg m/s) of a kinematic heat flux (K m/s) with a Bowen ratio.
inline double moisture_flux(const double heat_flux, const double bowen_ratio) { return heat_flux*Cp/(Lv*bowen_ratio); }

/// The hydrostatic pressure (Pa) at height h (m) above the ground for a potential temperature theta (K) taken constant
/// over the column (the dry mixed layer; the error above zi is a few hPa): p = p0 (1 - g h / (cp theta))^(cp / Rd).
inline double hydrostatic_pressure(const double h, const double theta, const double p0 = GroundPressure) {
	return p0*std::pow(1.0-G*h/(Cp*theta), Cp/Rd);
}

} // namespace moist_air
