// MOISTURE with LEVEL_FORCING: a moist thermal in saturated, neutral air against its dry twin.
//
// The design follows Bryan and Fritsch 2002 (Mon. Wea. Rev. 130, 2917): an environment of constant total water whose
// phase changes are reversible is neutral for one conserved variable, here thl, so no condensation level exists and
// the dilution of the thermal does not decide the result. The environment holds thl 300 K and qt 26 g/kg, saturated
// at every level (0.5 g/kg of liquid at the ground, 3.5 g/kg at 1200 m). The thermal is a cos^2 excess of thl (2 K,
// 160 m radius) with the same qt. In saturated air one kelvin of thl gives about 0.37 K of theta_v (the rest goes
// into evaporating cloud water); the factor grows by 2.6 % over the 1200 m box (host, moist_air.hpp).
//
// The dry twin: qt 0, every cell's thl excess set to the moist thermal's theta_v excess in that cell at the start, so
// both start with the same buoyancy field. With a correct moist buoyancy the twins rise alike, the moist one up to
// about 3 % faster (the factor's growth); a kernel without the latent heating would give the moist thermal 2.7 times
// the buoyancy. Measured: the rise of each thermal's thl-excess centroid; passes when the moist rise is within 3 % of
// the dry one. Also checked: qt stays 26 g/kg in every cell (it is uniform: any change is a numerical error), the
// thl excess stays inside its start range, and |u| stays at most 0.1 (lattice).
//
// Two checks of Bryan and Fritsch's benchmark:
// - The rest test (their p. 4-5): the same saturated box without the thermal must stay at rest. They found motions of
//   order 1e-4 m/s from truncation (double precision). Here one lattice speed unit is dx / dt = 62.5 m/s, and the
//   float32 lattice alone reaches about 1.8e-4 m/s in 1500 steps (the dry rest twin, measured), so their
//   limit holds for what the water adds: the moist rest's largest |u| less the dry rest's must stay below 1e-4 m/s.
// - The conservation (their eqs. 28-29, error about 1e-4 % in their benchmark): in the closed box the totals of qt and
//   of thl over the fluid cells must not change by more than 1e-6 of themselves over the moist thermal's run.
//
// The convective boundary layer's configuration: cells of 10 m, dt 0.16 s, nu 1e-5, SUBGRID with Pr_t 1/3, THERMAL_DIFFUSIVITY_FIX
// with alpha 0.014 (tauT 0.556). The wall cells hold the air's values (the first Esoteric-Pull step streams their
// initial populations into the fluid).

#include "setup/setup.hpp"
#include "physics_check.hpp"
#include "moist_air.hpp"
#include "moist_thermal_case.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

/// What one thermal did.
struct ThermalTrack {
	double start_centroid = 0.0; ///< cells, the thl-excess centroid at the first read-out
	double end_centroid = 0.0;   ///< cells, the same at the last read-out
	ulong steps_run = 0ull;      ///< steps up to the last read-out
	double u_max = 0.0;          ///< lattice, over the run
	double qt_error = 0.0;       ///< kg/kg, the largest |qt - qt_start| over the run
	double excess_over = 0.0;    ///< K, how far the thl excess went above its start maximum
	double excess_under = 0.0;   ///< K, how far it went below 0
	double qt_rho_residual = 0.0; ///< the largest |dQ/Q - drho/rho|: what the density change does not explain of the qt change
	double rho_change = 0.0;     ///< the largest |drho/rho|
	double qt_total_start = 0.0; ///< kg/kg summed over the fluid cells at the first read-out
	double qt_total_end = 0.0;   ///< the same at the last read-out
	double thl_total_start = 0.0; ///< K, thl summed over the fluid cells at the first read-out
	double thl_total_end = 0.0;  ///< the same at the last read-out
};

/// @brief Runs one thermal and tracks its thl-excess centroid.
/// @param box the box and the scales
/// @param qt the total water of the air everywhere (kg/kg)
/// @param excess_at the thl excess of a cell at the start (K)
/// @param name the name in the progress lines
/// @param stop_after the steps to run (0: until the thermal nears the top or |u| passes 0.1)
/// @return the track
template<typename Excess> ThermalTrack run_thermal(const ThermalBox& box, const double qt, const Excess& excess_at, const char* name, const ulong stop_after) {
	LBM lbm(box.N, box.N, box.Nz, box.nu, 0.0f, 0.0f, box.fz, 0.0f, box.alpha, box.beta());
	const double excess_max = box.set_up(lbm, qt, excess_at);
	ThermalTrack track;
	bool first = true;
	const ulong steps = stop_after>0ull ? stop_after : box.steps;
	for(ulong t=0ull; t<steps; t+=box.sample) {
		lbm.run(box.sample);
		lbm.update_fields();
		lbm.T.read_from_device();
		lbm.Q.read_from_device();
		lbm.u.read_from_device();
		lbm.rho.read_from_device();
		double weight = 0.0, moment = 0.0, u_max = 0.0, top = 0.0;
		double dq_min = 0.0, dq_max = 0.0, dq_excess = 0.0; // the qt change, signed, and its part inside the thermal (thl excess over 10 % of the start peak)
		uint dq_min_z = 0u, dq_max_z = 0u;
		double qt_total = 0.0, thl_total = 0.0; // the box totals (the conservation check)
		uint u_max_z = 0u; // the level of the largest |u|
		for(uint z=1u; z+1u<box.Nz; z++) for(uint y=0u; y<box.N; y++) for(uint x=0u; x<box.N; x++) {
			const ulong n = lbm.index(x, y, z);
			const double e = (double)lbm.T[n]-1.0;
			const double dq = ((double)lbm.Q[n]-1.0)*0.01-qt;
			qt_total += ((double)lbm.Q[n]-1.0)*0.01;
			thl_total += box.env_thl+e;
			if(dq<dq_min) { dq_min = dq; dq_min_z = z; }
			if(dq>dq_max) { dq_max = dq; dq_max_z = z; }
			if(e>0.1*excess_max) dq_excess = std::fmax(dq_excess, std::fabs(dq));
			if(e>0.0) { weight += e; moment += e*((double)z+0.5); }
			if(e>0.1*excess_max) top = std::fmax(top, (double)z);
			track.qt_error = std::fmax(track.qt_error, std::fabs(((double)lbm.Q[n]-1.0)*0.01-qt));
			const double Q0 = 1.0+qt/0.01, drho = (double)lbm.rho[n]-1.0;
			track.rho_change = std::fmax(track.rho_change, std::fabs(drho));
			track.qt_rho_residual = std::fmax(track.qt_rho_residual, std::fabs(((double)lbm.Q[n]-Q0)/Q0-drho));
			track.excess_over = std::fmax(track.excess_over, e-excess_max);
			track.excess_under = std::fmax(track.excess_under, -e);
			const double u_cell = std::sqrt(sq((double)lbm.u.x[n])+sq((double)lbm.u.y[n])+sq((double)lbm.u.z[n]));
			if(u_cell>u_max) { u_max = u_cell; u_max_z = z; }
		}
		{ // compressibility check: the qt change against the density's departure from its level mean, cell by cell (least squares slope; 1 = the water follows the density)
			const double Q0 = 1.0+qt/0.01;
			double sxy = 0.0, sxx = 0.0, syy = 0.0, q_sum = 0.0, rho_sum = 0.0;
			for(uint z=1u; z+1u<box.Nz; z++) {
				double rho_mean = 0.0;
				for(uint y=0u; y<box.N; y++) for(uint x=0u; x<box.N; x++) rho_mean += (double)lbm.rho[lbm.index(x, y, z)];
				rho_mean /= (double)(box.N*box.N);
				for(uint y=0u; y<box.N; y++) for(uint x=0u; x<box.N; x++) {
					const ulong n = lbm.index(x, y, z);
					const double xr = ((double)lbm.rho[n]-rho_mean)/rho_mean, yq = ((double)lbm.Q[n]-Q0)/Q0;
					sxy += xr*yq; sxx += xr*xr; syy += yq*yq;
					q_sum += (double)lbm.Q[n]-Q0; rho_sum += (double)lbm.rho[n]-1.0;
				}
			}
			const double cells = (double)(box.N*box.N*(box.Nz-2u));
			std::printf("  %s step %llu: dQ/Q against drho/rho (level mean removed): slope %.3f, correlation %.3f; the box mean dQ/Q %.3e, drho/rho %.3e\n", name, (unsigned long long)(t+box.sample), sxy/sxx, sxy/std::sqrt(sxx*syy), q_sum/cells/Q0, rho_sum/cells);
		}
		const double centroid = weight>0.0 ? moment/weight : 0.0; // the rest test has no excess: no centroid
		if(first) { track.start_centroid = centroid; track.qt_total_start = qt_total; track.thl_total_start = thl_total; first = false; }
		track.end_centroid = centroid;
		track.qt_total_end = qt_total;
		track.thl_total_end = thl_total;
		track.steps_run = t+box.sample;
		track.u_max = std::fmax(track.u_max, u_max);
		std::printf("  %s step %llu: the centroid at %.2f cells, the top (10 %% of the excess) at level %.0f, |u| max %.3e (level %u); qt change %.2e (level %u) to %.2e (level %u), inside the thermal %.2e\n", name, (unsigned long long)track.steps_run, centroid, top, u_max, u_max_z, dq_min, dq_min_z, dq_max, dq_max_z, dq_excess);
		if(stop_after==0ull&&(u_max>0.1||top+4.0>=(double)box.Nz)) break;
	}
	return track;
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID LEVEL_FORCING MOISTURE (the CBL configuration)
	const ThermalBox box;
	const auto moist_excess = [&](const uint x, const uint y, const uint z) { return box.excess*box.shape(x, y, z); };
	const auto dry_excess = [&](const uint x, const uint y, const uint z) { // the moist thermal's theta_v excess in this cell
		const double p = box.pressure(z);
		return moist_air::virtual_theta(box.env_thl+moist_excess(x, y, z), box.moist_qt, p)-moist_air::virtual_theta(box.env_thl, box.moist_qt, p);
	};
	const uint centre = (uint)box.centre_z;
	std::printf("\nmoist thermal: thl %.0f K + %.1f K, qt %.3f kg/kg everywhere (ql %.2f g/kg at the ground); the dry twin's peak excess %.4f K (theta_v of the moist peak)\n",
	            box.env_thl, box.excess, box.moist_qt, 1000.0*moist_air::saturation_adjust(box.env_thl, box.moist_qt, box.pressure(1u)).liquid, dry_excess(box.N/2u, box.N/2u, centre));
	const ThermalTrack moist = run_thermal(box, box.moist_qt, moist_excess, "moist", 0ull);
	const ThermalTrack dry = run_thermal(box, 0.0, dry_excess, "dry", moist.steps_run); // the same number of steps
	const auto no_excess = [](const uint, const uint, const uint) { return 0.0; };
	const ThermalTrack rest = run_thermal(box, box.moist_qt, no_excess, "rest", box.steps); // Bryan and Fritsch's rest test, the longest run
	const ThermalTrack rest_dry = run_thermal(box, 0.0, no_excess, "rest dry", box.steps); // the same without water: what the lattice alone does
	const double moist_rise = moist.end_centroid-moist.start_centroid, dry_rise = dry.end_centroid-dry.start_centroid;
	const double metres_per_second = box.dx/box.dt; // one lattice speed unit in m/s
	const double qt_drift = std::fabs(moist.qt_total_end-moist.qt_total_start)/moist.qt_total_start;
	const double thl_drift = std::fabs(moist.thl_total_end-moist.thl_total_start)/moist.thl_total_start;
	std::printf("\nafter %llu steps: the moist centroid rose %.3f cells, the dry twin's %.3f cells (ratio %.4f)\n", (unsigned long long)moist.steps_run, moist_rise, dry_rise, moist_rise/dry_rise);
	std::printf("moist: qt error %.2e kg/kg, thl excess over its start maximum %.4f K, under 0 %.4f K, |u| max %.4f\n", moist.qt_error, moist.excess_over, moist.excess_under, moist.u_max);
	std::printf("moist: the largest |drho/rho| %.2e, the largest |dQ/Q - drho/rho| %.2e (Q = 1 + qt/0.01)\n", moist.rho_change, moist.qt_rho_residual);
	std::printf("dry:   thl excess over its start maximum %.4f K, under 0 %.4f K, |u| max %.4f\n", dry.excess_over, dry.excess_under, dry.u_max);
	std::printf("rest:  %llu steps without a thermal: |u| max %.3e (lattice) = %.3e m/s, qt error %.2e kg/kg\n", (unsigned long long)rest.steps_run, rest.u_max, rest.u_max*metres_per_second, rest.qt_error);
	std::printf("rest dry: the same box without water: |u| max %.3e (lattice) = %.3e m/s\n", rest_dry.u_max, rest_dry.u_max*metres_per_second);
	std::printf("moist: the box totals over the run: qt %.9e -> %.9e (relative %.2e), thl %.9e -> %.9e K (relative %.2e)\n", moist.qt_total_start, moist.qt_total_end, qt_drift, moist.thl_total_start, moist.thl_total_end, thl_drift);
	PhysicsCheck::report("moist thermal: the rise against the dry twin with the same start buoyancy", std::fabs(moist_rise/dry_rise-1.0), 0.03);
	PhysicsCheck::report("moist thermal: the uniform qt stays uniform (kg/kg over 1e-5)", moist.qt_error/1.0E-5, 1.0);
	PhysicsCheck::report("moist thermal: the thl excess stays in its start range (K over 2 % of the peak)", std::fmax(moist.excess_over, moist.excess_under)/(0.02*box.excess), 1.0);
	PhysicsCheck::report("moist thermal: the lattice speed stays in range (|u| max over 0.1)", std::fmax(moist.u_max, dry.u_max)/0.1, 1.0);
	PhysicsCheck::report("rest test: the water adds no motion to the box at rest (moist less dry |u| max, over 1e-4 m/s)", std::fmax(rest.u_max-rest_dry.u_max, 0.0)*metres_per_second/1.0E-4, 1.0);
	PhysicsCheck::report("moist thermal: the box's total qt is conserved (relative change over 1e-6)", qt_drift/1.0E-6, 1.0);
	PhysicsCheck::report("moist thermal: the box's total thl is conserved (relative change over 1e-6)", thl_drift/1.0E-6, 1.0);
}
