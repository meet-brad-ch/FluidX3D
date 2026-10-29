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
#include <functional>
#include <vector>

namespace {

/// @brief What one thermal did.
struct ThermalTrack {
	double start_centroid{ 0.0 };  ///< cells, the thl-excess centroid at the first read-out
	double end_centroid{ 0.0 };    ///< cells, the same at the last read-out
	ulong steps_run{ 0ull };       ///< steps up to the last read-out
	double u_max{ 0.0 };           ///< lattice, over the run
	double qt_error{ 0.0 };        ///< kg/kg, the largest |qt - qt_start| over the run
	double excess_over{ 0.0 };     ///< K, how far the thl excess went above its start maximum
	double excess_under{ 0.0 };    ///< K, how far it went below 0
	double qt_rho_residual{ 0.0 }; ///< the largest |dQ/Q - drho/rho|: what the density change does not explain of the qt change
	double rho_change{ 0.0 };      ///< the largest |drho/rho|
	double qt_total_start{ 0.0 };  ///< kg/kg summed over the fluid cells at the first read-out
	double qt_total_end{ 0.0 };    ///< the same at the last read-out
	double thl_total_start{ 0.0 }; ///< K, thl summed over the fluid cells at the first read-out
	double thl_total_end{ 0.0 };   ///< the same at the last read-out
};

/// @brief The sums of one read-out over the fluid cells; it also updates the run's extremes in the track.
class ThermalReadOut {
public:
	/// @brief Starts an empty read-out.
	/// @param qt the total water of the air everywhere (kg/kg)
	/// @param excess_max the thermal's start peak thl excess (K)
	/// @param env_thl the environment's thl (K)
	ThermalReadOut(double qt, double excess_max, double env_thl) : qt_(qt), excess_max_(excess_max), env_thl_(env_thl) {}

	/// @brief Adds one fluid cell.
	/// @param lbm the lattice, its fields read
	/// @param n the cell's index
	/// @param z the cell's level
	/// @param track the run's track, whose extremes the cell updates
	void add(LBM& lbm, ulong n, uint z, ThermalTrack& track) {
		const double e{ static_cast<double>(lbm.T[n]) - 1.0 };
		const double q{ static_cast<double>(lbm.Q[n]) };
		const double cell_qt{ (q - 1.0) * q_scale };
		const double dq{ cell_qt - qt_ };
		qt_total += cell_qt;
		thl_total += env_thl_ + e;
		add_water(dq, z);
		const bool inside{ e > inside_fraction * excess_max_ };
		if(inside) {
			dq_excess = std::fmax(dq_excess, std::fabs(dq));
			top = std::fmax(top, static_cast<double>(z));
		}
		if(e > 0.0) {
			weight += e;
			moment += e * (static_cast<double>(z) + 0.5);
		}
		track.qt_error = std::fmax(track.qt_error, std::fabs(cell_qt - qt_));
		const double Q0{ 1.0 + qt_ / q_scale };
		const double drho{ static_cast<double>(lbm.rho[n]) - 1.0 };
		track.rho_change = std::fmax(track.rho_change, std::fabs(drho));
		track.qt_rho_residual = std::fmax(track.qt_rho_residual, std::fabs((q - Q0) / Q0 - drho));
		track.excess_over = std::fmax(track.excess_over, e - excess_max_);
		track.excess_under = std::fmax(track.excess_under, -e);
		const double ux{ static_cast<double>(lbm.u.x[n]) };
		const double uy{ static_cast<double>(lbm.u.y[n]) };
		const double uz{ static_cast<double>(lbm.u.z[n]) };
		const double u_cell{ std::sqrt(sq(ux) + sq(uy) + sq(uz)) };
		if(u_cell > u_max) {
			u_max = u_cell;
			u_max_z = z;
		}
	}

	/// @brief Gives the thl-excess centroid.
	/// @return the centroid in cells, 0 without excess (the rest test)
	double centroid() const {
		const double c{ weight > 0.0 ? moment / weight : 0.0 }; // the rest test has no excess: no centroid
		return c;
	}

	static constexpr double q_scale = 0.01;        ///< the kernel's water scale in kg/kg: Q = 1 + qt / q_scale
	static constexpr double inside_fraction = 0.1; ///< a cell is inside the thermal above this part of the start peak

	double weight{ 0.0 };    ///< the sum of the positive thl excess
	double moment{ 0.0 };    ///< the sum of the positive thl excess times the cell centre's height in cells
	double u_max{ 0.0 };     ///< the largest |u| in lattice units
	uint u_max_z{ 0u };      ///< the level of u_max
	double top{ 0.0 };       ///< the highest level inside the thermal
	double dq_min{ 0.0 };    ///< the most negative qt change in kg/kg
	uint dq_min_z{ 0u };     ///< its level
	double dq_max{ 0.0 };    ///< the most positive qt change in kg/kg
	uint dq_max_z{ 0u };     ///< its level
	double dq_excess{ 0.0 }; ///< the largest |qt change| inside the thermal in kg/kg
	double qt_total{ 0.0 };  ///< the box's total qt in kg/kg
	double thl_total{ 0.0 }; ///< the box's total thl in K

private:
	/// @brief Keeps the extremes of the qt change and their levels.
	/// @param dq the cell's qt change in kg/kg
	/// @param z the cell's level
	void add_water(double dq, uint z) {
		if(dq < dq_min) {
			dq_min = dq;
			dq_min_z = z;
		}
		if(dq > dq_max) {
			dq_max = dq;
			dq_max_z = z;
		}
	}

	double qt_{};         ///< the air's total water everywhere in kg/kg
	double excess_max_{}; ///< the start peak thl excess in K
	double env_thl_{};    ///< the environment's thl in K
};

/// @brief One thermal's run, read out every box.sample steps.
class ThermalRun {
public:
	/// @brief Runs one thermal and tracks its thl-excess centroid.
	/// @param box the box and the scales
	/// @param qt the total water of the air everywhere (kg/kg)
	/// @param excess_at the thl excess of a cell at the start (K)
	/// @param name the name in the progress lines
	/// @param stop_after the steps to run (0: until the thermal nears the top or |u| passes 0.1)
	/// @return the track
	static ThermalTrack run(const ThermalBox& box, double qt, const std::function<double(const CellCentre&)>& excess_at, const char* name, ulong stop_after) {
		AtmosphereSimulation atm(box.grid(), box.theta_0(), box.numerics(), std::nullopt);
		const double excess_max{ box.set_up(atm, qt, excess_at) };
		LBM& lbm = atm.lbm(); // the measurements read the core's fields (the low layer)
		ThermalTrack track{};
		bool first{ true };
		const ulong steps{ stop_after > 0ull ? stop_after : box.steps };
		for(ulong t{ 0ull }; t < steps; t += box.sample) {
			lbm.run(box.sample);
			read_fields(lbm);
			const ThermalReadOut r = read_out(lbm, box, qt, excess_max, track);
			const ulong step{ t + box.sample };
			print_compressibility(lbm, box, qt, name, step);
			const double centroid{ r.centroid() };
			if(first) {
				track.start_centroid = centroid;
				track.qt_total_start = r.qt_total;
				track.thl_total_start = r.thl_total;
				first = false;
			}
			track.end_centroid = centroid;
			track.qt_total_end = r.qt_total;
			track.thl_total_end = r.thl_total;
			track.steps_run = step;
			track.u_max = std::fmax(track.u_max, r.u_max);
			const unsigned long long printed_step{ static_cast<unsigned long long>(track.steps_run) };
			std::printf("  %s step %llu: the centroid at %.2f cells, the top (10 %% of the excess) at level %.0f, |u| max %.3e (level %u); qt change %.2e (level %u) to %.2e (level %u), inside the thermal %.2e\n",
			            name, printed_step, centroid, r.top, r.u_max, r.u_max_z, r.dq_min, r.dq_min_z, r.dq_max, r.dq_max_z, r.dq_excess);
			const bool too_fast{ r.u_max > largest_speed };
			const bool near_top{ r.top + top_margin >= static_cast<double>(box.Nz) };
			if(stop_after == 0ull && (too_fast || near_top)) {
				break;
			}
		}
		return track;
	}

	static constexpr double largest_speed = 0.1; ///< the lattice speed at which a free run stops

private:
	/// @brief Reads T, Q, u and rho back from the device.
	/// @param lbm the lattice
	static void read_fields(LBM& lbm) {
		lbm.update_fields();
		lbm.T.read_from_device();
		lbm.Q.read_from_device();
		lbm.u.read_from_device();
		lbm.rho.read_from_device();
	}

	/// @brief Sums one read-out over the fluid levels (1 to Nz - 2).
	/// @param lbm the lattice, its fields read
	/// @param box the box's cells and environment thl
	/// @param qt the air's total water everywhere (kg/kg)
	/// @param excess_max the start peak thl excess (K)
	/// @param track the run's track, whose extremes the cells update
	/// @return the read-out
	static ThermalReadOut read_out(LBM& lbm, const ThermalBox& box, double qt, double excess_max, ThermalTrack& track) {
		ThermalReadOut r(qt, excess_max, box.env_thl);
		const ulong layer{ static_cast<ulong>(box.N) * static_cast<ulong>(box.N) };
		for(uint z{ 1u }; z + 1u < box.Nz; z++) {
			const ulong layer_start{ static_cast<ulong>(lbm.index(0u, 0u, z)) };
			for(ulong i{ 0ull }; i < layer; i++) {
				r.add(lbm, layer_start + i, z, track);
			}
		}
		return r;
	}

	/// @brief Prints the qt change against the density's departure from its level mean, cell by cell (least squares
	/// slope; 1 = the water follows the density).
	/// @param lbm the lattice, its fields read
	/// @param box the box's cells along each axis
	/// @param qt the air's total water everywhere (kg/kg)
	/// @param name the run's name
	/// @param step the step of the read-out
	static void print_compressibility(LBM& lbm, const ThermalBox& box, double qt, const char* name, ulong step) {
		const double Q0{ 1.0 + qt / ThermalReadOut::q_scale };
		double sxy{ 0.0 };
		double sxx{ 0.0 };
		double syy{ 0.0 };
		double q_sum{ 0.0 };
		double rho_sum{ 0.0 };
		const ulong layer{ static_cast<ulong>(box.N) * static_cast<ulong>(box.N) };
		for(uint z{ 1u }; z + 1u < box.Nz; z++) {
			const ulong layer_start{ static_cast<ulong>(lbm.index(0u, 0u, z)) };
			double rho_mean{ 0.0 };
			for(ulong i{ 0ull }; i < layer; i++) {
				rho_mean += static_cast<double>(lbm.rho[layer_start + i]);
			}
			rho_mean /= static_cast<double>(box.N * box.N);
			for(ulong i{ 0ull }; i < layer; i++) {
				const ulong n{ layer_start + i };
				const double rho{ static_cast<double>(lbm.rho[n]) };
				const double q{ static_cast<double>(lbm.Q[n]) };
				const double xr{ (rho - rho_mean) / rho_mean };
				const double yq{ (q - Q0) / Q0 };
				sxy += xr * yq;
				sxx += xr * xr;
				syy += yq * yq;
				q_sum += q - Q0;
				rho_sum += rho - 1.0;
			}
		}
		const double cells{ static_cast<double>(box.N * box.N * (box.Nz - 2u)) };
		const double slope{ sxy / sxx };
		const double correlation{ sxy / std::sqrt(sxx * syy) };
		const double mean_dq{ q_sum / cells / Q0 };
		const double mean_drho{ rho_sum / cells };
		const unsigned long long printed_step{ static_cast<unsigned long long>(step) };
		std::printf("  %s step %llu: dQ/Q against drho/rho (level mean removed): slope %.3f, correlation %.3f; the box mean dQ/Q %.3e, drho/rho %.3e\n", name, printed_step, slope,
		            correlation, mean_dq, mean_drho);
	}

	static constexpr double top_margin = 4.0; ///< the levels under the lid at which a free run stops
};

/// @brief The four runs (moist, its dry twin, the rest test with and without water) and their checks.
class MoistBubbleTest {
public:
	/// @brief The tracks of the four runs.
	struct Runs {
		ThermalTrack moist{};    ///< the moist thermal
		ThermalTrack dry{};      ///< the dry twin
		ThermalTrack rest{};     ///< the moist box at rest
		ThermalTrack rest_dry{}; ///< the dry box at rest
	};

	/// @brief Runs the four thermals.
	/// @param box the box and the scales
	/// @return their tracks
	static Runs run_all(const ThermalBox& box) {
		const AtmosphereGrid grid = box.grid();
		const MoistColumn column = box.column(box.moist_qt);
		const std::function<double(const CellCentre&)> moist_excess = [&](const CellCentre& c) { return box.excess * box.shape(c); };
		const std::function<double(const CellCentre&)> dry_excess = [&](const CellCentre& c) { // the moist thermal's theta_v excess in this cell
			const double p = column.pressure(grid.levels().nearest_level(c.height)).in(au::pascals); // the raw model below (moist_air.hpp)
			return moist_air::virtual_theta(box.env_thl + moist_excess(c), box.moist_qt, p) - moist_air::virtual_theta(box.env_thl, box.moist_qt, p);
		};
		const uint centre{ static_cast<uint>(box.centre_z) };
		const si::WaterContent moist_qt = (si::kilograms_of_water / si::kilograms)(box.moist_qt);
		const double ground_liquid{ column.liquid(box.theta_0(), moist_qt, 1u).in(au::milli(si::kilograms_of_water) / si::kilograms) };
		const double dry_peak{ dry_excess(grid.centre(box.N / 2u, box.N / 2u, centre)) };
		std::printf("\nmoist thermal: thl %.0f K + %.1f K, qt %.3f kg/kg everywhere (ql %.2f g/kg at the ground); the dry twin's peak excess %.4f K (theta_v of the moist peak)\n",
		            box.env_thl, box.excess, box.moist_qt, ground_liquid, dry_peak);
		Runs runs{};
		runs.moist = ThermalRun::run(box, box.moist_qt, moist_excess, "moist", 0ull);
		runs.dry = ThermalRun::run(box, 0.0, dry_excess, "dry", runs.moist.steps_run); // the same number of steps
		const std::function<double(const CellCentre&)> no_excess = [](const CellCentre&) { return 0.0; };
		runs.rest = ThermalRun::run(box, box.moist_qt, no_excess, "rest", box.steps); // Bryan and Fritsch's rest test, the longest run
		runs.rest_dry = ThermalRun::run(box, 0.0, no_excess, "rest dry", box.steps); // the same without water: what the lattice alone does
		return runs;
	}

	/// @brief Prints the results and reports the seven checks.
	/// @param box the box and the scales
	/// @param runs the four tracks
	static void report(const ThermalBox& box, const Runs& runs) {
		const ThermalTrack& moist = runs.moist;
		const ThermalTrack& dry = runs.dry;
		const double moist_rise{ moist.end_centroid - moist.start_centroid };
		const double dry_rise{ dry.end_centroid - dry.start_centroid };
		const double metres_per_second{ box.dx / box.dt }; // one lattice speed unit in m/s
		const double qt_drift{ std::fabs(moist.qt_total_end - moist.qt_total_start) / moist.qt_total_start };
		const double thl_drift{ std::fabs(moist.thl_total_end - moist.thl_total_start) / moist.thl_total_start };
		print_results(runs, moist_rise, dry_rise, metres_per_second, qt_drift, thl_drift);
		const double rise_error{ std::fabs(moist_rise / dry_rise - 1.0) };
		PhysicsCheck::report("moist thermal: the rise against the dry twin with the same start buoyancy", rise_error, rise_limit);
		PhysicsCheck::report("moist thermal: the uniform qt stays uniform (kg/kg over 1e-5)", moist.qt_error / qt_unit, 1.0);
		const double excess_part{ std::fmax(moist.excess_over, moist.excess_under) / (excess_fraction * box.excess) };
		PhysicsCheck::report("moist thermal: the thl excess stays in its start range (K over 2 % of the peak)", excess_part, 1.0);
		const double speed_part{ std::fmax(moist.u_max, dry.u_max) / ThermalRun::largest_speed };
		PhysicsCheck::report("moist thermal: the lattice speed stays in range (|u| max over 0.1)", speed_part, 1.0);
		const double rest_motion{ std::fmax(runs.rest.u_max - runs.rest_dry.u_max, 0.0) * metres_per_second / rest_speed_unit };
		PhysicsCheck::report("rest test: the water adds no motion to the box at rest (moist less dry |u| max, over 1e-4 m/s)", rest_motion, 1.0);
		PhysicsCheck::report("moist thermal: the box's total qt is conserved (relative change over 1e-6)", qt_drift / drift_unit, 1.0);
		PhysicsCheck::report("moist thermal: the box's total thl is conserved (relative change over 1e-6)", thl_drift / drift_unit, 1.0);
	}

private:
	/// @brief Prints the results of the four runs.
	/// @param runs the four tracks
	/// @param moist_rise the moist centroid's rise in cells
	/// @param dry_rise the dry twin's rise in cells
	/// @param metres_per_second one lattice speed unit in m/s
	/// @param qt_drift the relative change of the moist box's total qt
	/// @param thl_drift the relative change of the moist box's total thl
	static void print_results(const Runs& runs, double moist_rise, double dry_rise, double metres_per_second, double qt_drift, double thl_drift) {
		const ThermalTrack& moist = runs.moist;
		const ThermalTrack& dry = runs.dry;
		const unsigned long long moist_steps{ static_cast<unsigned long long>(moist.steps_run) };
		const unsigned long long rest_steps{ static_cast<unsigned long long>(runs.rest.steps_run) };
		const double ratio{ moist_rise / dry_rise };
		const double rest_speed{ runs.rest.u_max * metres_per_second };
		const double rest_dry_speed{ runs.rest_dry.u_max * metres_per_second };
		std::printf("\nafter %llu steps: the moist centroid rose %.3f cells, the dry twin's %.3f cells (ratio %.4f)\n", moist_steps, moist_rise, dry_rise, ratio);
		std::printf("moist: qt error %.2e kg/kg, thl excess over its start maximum %.4f K, under 0 %.4f K, |u| max %.4f\n", moist.qt_error, moist.excess_over, moist.excess_under, moist.u_max);
		std::printf("moist: the largest |drho/rho| %.2e, the largest |dQ/Q - drho/rho| %.2e (Q = 1 + qt/0.01)\n", moist.rho_change, moist.qt_rho_residual);
		std::printf("dry:   thl excess over its start maximum %.4f K, under 0 %.4f K, |u| max %.4f\n", dry.excess_over, dry.excess_under, dry.u_max);
		std::printf("rest:  %llu steps without a thermal: |u| max %.3e (lattice) = %.3e m/s, qt error %.2e kg/kg\n", rest_steps, runs.rest.u_max, rest_speed, runs.rest.qt_error);
		std::printf("rest dry: the same box without water: |u| max %.3e (lattice) = %.3e m/s\n", runs.rest_dry.u_max, rest_dry_speed);
		std::printf("moist: the box totals over the run: qt %.9e -> %.9e (relative %.2e), thl %.9e -> %.9e K (relative %.2e)\n", moist.qt_total_start, moist.qt_total_end, qt_drift,
		            moist.thl_total_start, moist.thl_total_end, thl_drift);
	}

	static constexpr double rise_limit = 0.03;       ///< the largest relative difference of the rises that passes (3 %)
	static constexpr double qt_unit = 1.0E-5;        ///< the qt error in kg/kg that fails the uniformity check
	static constexpr double excess_fraction = 0.02;  ///< the part of the peak excess that fails the range check
	static constexpr double rest_speed_unit = 1.0E-4; ///< the extra speed in m/s that fails the rest test
	static constexpr double drift_unit = 1.0E-6;     ///< the relative change that fails a conservation check
};

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID LEVEL_FORCING MOISTURE (the CBL configuration)
	const ThermalBox box;
	const MoistBubbleTest::Runs runs = MoistBubbleTest::run_all(box);
	MoistBubbleTest::report(box, runs);
}
