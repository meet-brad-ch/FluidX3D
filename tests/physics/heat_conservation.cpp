// The heat budget with buoyancy (SCALAR_FV): a heated patch on the floor of a box with TYPE_S floor and ceiling (nu
// 5e-4, alpha nu/0.71 with THERMAL_DIFFUSIVITY_FIX, SUBGRID with Pr_t 1/3, the Boussinesq force f = fz (1 - beta (T - 1))
// with fz = -1e-6). Every 1000 steps of 20000: the total heat sum(T - 1) over the fluid cells against the heat added
// (heat_rate x patch cells x steps), and the lowest T. The cases: without buoyancy (beta 0), the plume (beta 2000), the
// plume without the eddy heat diffusivity, and the plume in a box periodic in z (no walls).
// Passes when in every case the total heat stays within 2 % of the heat added and T never falls below 1 - 0.01
// (nothing cools below the start). The D3Q7 thermal lattice does not pass at this alpha (tauT 0.503): next to walls it
// creates heat, so SCALAR_FV is the transport for such cases (research/convection/README.md).

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <cstdio>

namespace {

/// @brief The heat budget of one sample.
struct HeatSample {
	double total{ 0.0 };      ///< sum(T - 1) over the fluid cells
	double minimum{ 0.0 };    ///< the smallest T of a fluid cell
	double maximum{ 0.0 };    ///< the largest T of a fluid cell
	double floor_mean{ 0.0 }; ///< the mean T of z = 1 outside the patch
};

/// @brief One case of the matrix.
struct CaseSettings {
	float beta{ 0.0f };          ///< the thermal expansion (0: no buoyancy)
	float inv_prandtl_t{ 0.0f }; ///< the eddy heat diffusivity's 1/Pr_t (0: SUBGRID adds nothing to the thermal relaxation)
	float alpha{ 0.0f };         ///< the thermal diffusivity
	bool walls{ true };          ///< TYPE_S floor and ceiling (false: periodic in z, the patch at z = 0 as the heat_source test)
	const char* name{ "" };      ///< the case's name, for the messages
};

/// @brief What one case measured at the end.
struct CaseResult {
	double worst_budget{ 0.0 }; ///< the largest |total - added| / added over the samples
	double lowest_T{ 0.0 };     ///< the smallest T of a fluid cell over the samples
	double final_error{ 0.0 };  ///< (total - added) / added at the end
	double floor_T{ 0.0 };      ///< the floor layer's mean T outside the patch at the end
};

/// @brief One case: a heated patch in a box, its total heat sampled against the heat added.
class HeatBudget {
public:
	/// @brief Runs a case and prints its result.
	/// @param c the case
	/// @return what it measured
	static CaseResult run(const CaseSettings& c) {
		LBM lbm(box_cells, box_cells, box_levels, viscosity, 0.0f, 0.0f, fz, 0.0f, c.alpha, c.beta);
		const uint patch_z{ c.walls ? 1u : 0u };
		const ulong patch{ flag_cells(lbm, c.walls, patch_z) };
		lbm.set_heat_rate(heat_rate);
		lbm.set_inv_prandtl_t(c.inv_prandtl_t);
		const int printed_walls{ c.walls ? 1 : 0 };
		const unsigned long long printed_patch{ static_cast<unsigned long long>(patch) };
		std::printf("\nheat conservation, %s: beta %.0f, 1/Pr_t %.1f, alpha %.2e, walls %d; box %u x %u x %u, patch %llu cells, heat_rate %.1e\n",
		            c.name, c.beta, c.inv_prandtl_t, c.alpha, printed_walls, box_cells, box_cells, box_levels, printed_patch, heat_rate);
		CaseResult r{};
		r.lowest_T = start_extreme;
		for(ulong t{ 0ull }; t < steps; t += sample) {
			lbm.run(sample);
			const HeatSample s = sample_of(lbm, patch_z);
			const double added{ static_cast<double>(heat_rate) * static_cast<double>(patch) * static_cast<double>(t + sample) };
			r.final_error = (s.total - added) / added;
			r.worst_budget = std::fmax(r.worst_budget, std::fabs(r.final_error));
			r.lowest_T = std::fmin(r.lowest_T, s.minimum);
			r.floor_T = s.floor_mean;
		}
		const double final_percent{ percent * r.final_error };
		const double worst_percent{ percent * r.worst_budget };
		std::printf("  %s: budget error at the end %+.1f %% (worst %.1f %%), min T %.3f, floor T at the end %.3f\n", c.name, final_percent, worst_percent, r.lowest_T, r.floor_T);
		return r;
	}

private:
	/// @brief Sets T to 1 and flags the walls (when the case has them) and the heated disc in the patch layer.
	/// @param lbm the box's lattice
	/// @param walls whether the floor and the ceiling are TYPE_S
	/// @param patch_z the patch's layer
	/// @return the patch's cells
	static ulong flag_cells(LBM& lbm, bool walls, uint patch_z) {
		const float cx{ 0.5f * static_cast<float>(box_cells - 1u) };
		const float cy{ cx };
		const float r0{ static_cast<float>(patch_radius) };
		ulong patch{ 0ull };
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			lbm.T[n] = 1.0f;
			const bool wall{ walls && (z == 0u || z == box_levels - 1u) };
			if(wall) {
				lbm.flags[n] = TYPE_S;
				continue;
			}
			const float dx{ static_cast<float>(x) - cx };
			const float dy{ static_cast<float>(y) - cy };
			const bool in_disc{ sq(dx) + sq(dy) <= sq(r0) };
			if(z == patch_z && in_disc) {
				lbm.flags[n] = TYPE_Y;
				patch++;
			}
		}
		return patch;
	}

	/// @brief Gives the heat budget of the fluid cells.
	/// @param lbm the box's lattice
	/// @param patch_z the patch's layer
	/// @return the sample
	static HeatSample sample_of(LBM& lbm, uint patch_z) {
		lbm.update_fields();
		lbm.T.read_from_device();
		HeatSample s{};
		s.minimum = start_extreme;
		s.maximum = -start_extreme;
		ulong floor_cells{ 0ull };
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			const uchar solid_bit{ static_cast<uchar>(lbm.flags[n] & TYPE_S) };
			const bool solid{ solid_bit != 0u };
			if(solid) {
				continue;
			}
			const double T{ static_cast<double>(lbm.T[n]) };
			s.total += T - 1.0;
			s.minimum = std::fmin(s.minimum, T);
			s.maximum = std::fmax(s.maximum, T);
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			const uchar source_bit{ static_cast<uchar>(lbm.flags[n] & TYPE_Y) };
			const bool source{ source_bit != 0u };
			if(z == patch_z && !source) {
				s.floor_mean += T;
				floor_cells++;
			}
		}
		s.floor_mean /= static_cast<double>(floor_cells);
		return s;
	}

	static constexpr uint box_cells = 64u;         ///< the box's cells along x and y
	static constexpr uint box_levels = 128u;       ///< the box's cells along z
	static constexpr uint patch_radius = 6u;       ///< the heated disc's radius in cells
	static constexpr float viscosity = 5.0E-4f;    ///< the kinematic viscosity in lattice units
	static constexpr float fz = -1.0E-6f;          ///< the volume force along z in lattice units
	static constexpr float heat_rate = 1.0E-3f;    ///< the temperature added per step to each source cell
	static constexpr ulong steps = 20000ull;       ///< the steps of a case
	static constexpr ulong sample = 1000ull;       ///< the steps between samples
	static constexpr double start_extreme = 1.0E9; ///< the start of a minimum (and, negated, of a maximum) search
	static constexpr double percent = 100.0;       ///< the percent of a fraction of 1, for the printed errors
};

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID SCALAR_FV
	const double plume_viscosity{ 5.0E-4 };
	const double prandtl{ 0.71 };
	const float alpha{ static_cast<float>(plume_viscosity / prandtl) }; // the plume's diffusivity (tauT 0.503)
	const float buoyant{ 2000.0f };
	const float eddy{ 3.0f };
	const CaseSettings matrix[] = {
		{ 0.0f, eddy, alpha, true, "without buoyancy" },
		{ buoyant, eddy, alpha, true, "plume" },
		{ buoyant, 0.0f, alpha, true, "plume without the eddy heat diffusivity" },
		{ buoyant, eddy, alpha, false, "plume, periodic in z" },
	};
	constexpr std::size_t name_capacity{ 160u };
	const double budget_limit{ 0.02 }; // the largest budget error that passes (2 %)
	const double cooling_limit{ 0.01 }; // the largest cooling below the start that passes
	for(const CaseSettings& c : matrix) {
		const CaseResult r = HeatBudget::run(c);
		char name[name_capacity]{};
		std::snprintf(name, sizeof(name), "heat conservation, %s: total heat against the heat added", c.name);
		PhysicsCheck::report(name, r.worst_budget, budget_limit);
		std::snprintf(name, sizeof(name), "heat conservation, %s: no cooling below the start (1 - min T)", c.name);
		const double cooling{ std::fmax(0.0, 1.0 - r.lowest_T) };
		PhysicsCheck::report(name, cooling, cooling_limit);
	}
}
