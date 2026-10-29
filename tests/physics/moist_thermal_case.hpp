// The moist thermal in saturated, neutral air: the box, its physical scales and its set-up through the atmosphere API
// (AtmosphereSimulation and its builders), used by moist_bubble.cpp (the moist thermal against its dry twin).
#pragma once

#include "setup/setup.hpp"
#include "setup/atmosphere/atmosphere_simulation.hpp"
#include "moist_air.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <vector>

/// @brief The box and its physical scales: cells of 10 m, dt 0.16 s, the CBL example's lattice parameters.
struct ThermalBox {
	uint N = 64u;                ///< cells wide (periodic in x and y)
	uint Nz = 128u;              ///< cells high, the bottom and top layers solid
	double dx = 10.0;            ///< m
	double dt = 0.16;            ///< s
	double env_thl = 300.0;      ///< K
	double moist_qt = 0.026;     ///< kg/kg, the moist case's total water everywhere
	double excess = 2.0;         ///< K, the moist thermal's peak thl excess
	float centre_z = 30.0f;      ///< cells
	float radius = 16.0f;        ///< cells
	ulong sample = 50ull;        ///< steps between read-outs
	ulong steps = 1500ull;       ///< steps at most

	/// @return the cell
	si::Length cell() const { return au::meters(dx); }
	/// @brief Gives the box as an atmosphere grid.
	/// @return the box: N x N cells of dx, Nz levels (the floor and the lid solid)
	AtmosphereGrid grid() const {
		const si::Length across = cell() * (double)N;
		const si::Length ground_to_lid = cell() * (double)(Nz - 2u);
		const AtmosphereGrid g = AtmosphereGrid::of(across, across, ground_to_lid, cell());
		return g;
	}
	/// @brief Gives the numerics of the CBL example.
	/// @return the CBL example's numbers: dt 0.16 s, nu 1e-5 and the scalars' 0.014 (tauT 0.556) in lattice units,
	/// Pr_t 1/3, g 9.81 m/s^2 and the carrier 1e-6
	CblNumerics numerics() const {
		const si::LatticeDiffusivity scalar_diffusivity{ 0.014 }; // the CBL example's scalar diffusivity (tauT 0.556)
		CblNumerics n{};
		n.time_step = TimeStepRule::fixed(au::seconds(dt));
		n.scalar = Diffusivity::lattice(scalar_diffusivity);
		return n;
	}
	/// @return theta_0, the environment's thl
	si::Temperature theta_0() const { return si::kelvin_point(env_thl); }
	/// @brief Gives the environment's moist column at one total water.
	/// @param qt the total water everywhere (kg/kg)
	/// @return the environment's column: the closed-form pressure of a constant thl, theta_v per level
	MoistColumn column(double qt) const {
		const si::Temperature theta = theta_0();
		const si::WaterContent total_water = (si::kilograms_of_water / si::kilograms)(qt);
		const AtmosphereGrid g = grid();
		const MoistColumn c = MoistColumn::isentropic(theta, total_water, g.levels());
		return c;
	}
	/// @brief Gives the thermal's shape at a cell.
	/// @param c a cell's centre
	/// @return the cos^2 shape of the thermal there, 1 at the centre, 0 from one radius out (in cell units: the centre
	/// of cell z is z + 1/2, h / dx + 1 above the floor's top)
	double shape(const CellCentre& c) const {
		const double cx = 0.5 * (double)N;
		const double x = au::as_raw_number(c.east / cell());
		const double y = au::as_raw_number(c.north / cell());
		const double z = au::as_raw_number(c.height / cell()) + 1.0;
		const double x2 = sq(x - cx);
		const double y2 = sq(y - cx);
		const double z2 = sq(z - (double)centre_z);
		const double distance2 = x2 + y2 + z2;
		const double L = std::sqrt(distance2) / (double)radius;
		double value = 0.0;
		if(L < 1.0) {
			const double angle = 0.5 * 3.14159265358979 * L;
			const double cosine = std::cos(angle);
			value = sq(cosine);
		}
		return value;
	}

	/// @brief Sets up the atmosphere: a uniform sounding (thl 300 K, qt everywhere) with a thl excess per cell, the
	/// density in hydrostatic balance with the carrier (a uniform start rang the column's lowest sound mode at fz times
	/// a quarter period, 1.1e-4 lattice, 7 mm/s, which the rest test of Bryan and Fritsch 2002 caught), no level forcing
	/// (T_ref 1, rate 0), the moist air's terms (the level pressure and reference theta_v, theta_0) without a source.
	/// @param atm the atmosphere (made from grid(), theta_0(), numerics())
	/// @param qt the total water of the air everywhere (kg/kg)
	/// @param excess_at the thl excess of a cell at the start (K)
	/// @return the largest start excess (K)
	double set_up(AtmosphereSimulation& atm, const double qt, const std::function<double(const CellCentre&)>& excess_at) const {
		const si::Temperature t = theta_0();
		const si::WaterContent q = (si::kilograms_of_water / si::kilograms)(qt);
		const si::Length bottom = au::meters(0.0); // the sounding's points, around the box
		const si::Length top = au::meters(5000.0);
		const std::vector<TemperaturePoint> thl_points{ { bottom, t }, { top, t } };
		const std::vector<WaterContentPoint> qt_points{ { bottom, q }, { top, q } };
		const TemperatureProfile thl = TemperatureProfile::from_points(thl_points);
		const WaterContentProfile total_water = WaterContentProfile::from_points(qt_points);
		const Sounding uniform = Sounding::tabulated(thl, total_water, t);
		double excess_max = 0.0;
		const CblNumerics n = numerics();
		const BoussinesqBuoyancy buoyancy = BoussinesqBuoyancy(n.gravity, t, n.carrier);
		const std::function<si::TemperatureDifference(const CellCentre&)> excess_of = [&](const CellCentre& c) {
			const double e = excess_at(c);
			excess_max = std::fmax(excess_max, e);
			const si::TemperatureDifference e_kelvin = au::kelvins(e);
			return e_kelvin;
		};
		AtmosphereStartBuilder start = atm.start();
		start.sounding(uniform);
		start.moisture(true);
		start.balanced_density(buoyancy);
		start.add_excess(excess_of);
		start.apply();
		atm.level_forcing().update(nullptr, std::nullopt);
		const MoistColumn environment = column(qt);
		const si::MoistureFlux no_flux = si::MoistureFlux{ au::ZERO };
		atm.moisture(environment, no_flux);
		return excess_max;
	}
};
