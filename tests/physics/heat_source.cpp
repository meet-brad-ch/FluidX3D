// Heat source: a periodic box of fluid at rest whose layer z = 0 is flagged TYPE_Y, heated by LBM::set_heat_rate().
// Without walls every bit of added heat stays in the fluid cells: the mean temperature rises by
// heat_rate * (source cells) / (cells) per time step, while diffusion carries the heat away from the layer.
// (Next to TYPE_S walls a sum over the fluid cells misses heat: esoteric pull returns a population sent into a wall
// two steps later, so one population per wall link sits in the wall's own slots at any time.)
// The heated patch: cells flagged TYPE_X|TYPE_Y get four times heat_rate (the kernel's fixed factor, the convective
// boundary layer example's surface patch); a second box has half of its source layer so flagged, and its mean rises by
// heat_rate * (plain source cells + 4 patch cells) / (cells) per step.
// Passes when each box's mean temperature rise from step 1000 to 2000 is within 0.1 % of that, and the layer farthest
// from the source has warmed.

// On the core directly, not the atmosphere API: the test measures the core's own heat source rates in lattice units, in a
// periodic box without walls (the atmosphere API always has a solid floor and lid, where a sum over the fluid misses heat).

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <string>

namespace {

/// @brief The source cells of a box: those with the plain heat rate and those of the heated patch (4x).
struct SourceCells {
	ulong plain{ 0ull };   ///< the cells flagged TYPE_Y
	ulong patched{ 0ull }; ///< the cells flagged TYPE_X|TYPE_Y
};

/// @brief One periodic box at rest whose layer z = 0 is a heat source: its source layer plain, or half of it the heated
/// patch. Checks the added heat against the rate and that the heat reached the farthest layer.
class HeatBox {
public:
	/// @brief Runs the box and reports its two checks.
	/// @param name the check's name
	/// @param patch whether the cells x < Nx/2 of the source layer are TYPE_X|TYPE_Y
	static void run(const std::string& name, bool patch) {
		LBM lbm(cells_x, cells_y, cells_z, viscosity, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f); // no gravity: the fluid stays at rest
		const SourceCells sources = flag_sources(lbm, patch);
		lbm.set_heat_rate(heat_rate);
		lbm.run(steps);
		const double rise1{ mean_rise(lbm) };
		lbm.run(steps);
		const double rise2{ mean_rise(lbm) };
		const double middle{ middle_layer_rise(lbm) };
		const double plain{ static_cast<double>(sources.plain) };
		const double patched{ static_cast<double>(sources.patched) };
		const double cells{ static_cast<double>(lbm.get_N()) };
		const double expected{ static_cast<double>(heat_rate) * (plain + patch_factor * patched) * static_cast<double>(steps) / cells }; // mean rise over one run of steps
		const unsigned long long printed_plain{ static_cast<unsigned long long>(sources.plain) };
		const unsigned long long printed_patched{ static_cast<unsigned long long>(sources.patched) };
		const unsigned long long printed_steps{ static_cast<unsigned long long>(steps) };
		const unsigned long long printed_both{ 2ull * printed_steps };
		std::printf("\n%s: %llu plain source cells, %llu patch cells; mean temperature rise %.6e after %llu steps, %.6e after %llu (expected per %llu steps %.6e), farthest layer %.6e\n",
		            name.c_str(), printed_plain, printed_patched, rise1, printed_steps, rise2, printed_both, printed_steps, expected, middle);
		const double heat_error{ std::fabs((rise2 - rise1) / expected - 1.0) };
		const std::string heat_check = name + ": the added heat";
		PhysicsCheck::report(heat_check.c_str(), heat_error, heat_limit);
		const double reach_error{ middle > 0.0 ? 0.0 : 1.0 };
		const std::string reach_check = name + ": the heat reached the farthest layer";
		PhysicsCheck::report(reach_check.c_str(), reach_error, reach_limit);
	}

private:
	/// @brief Flags the source layer z = 0: TYPE_Y, or TYPE_X|TYPE_Y for x < Nx/2 when the box has the patch.
	/// @param lbm the box's lattice
	/// @param patch whether half of the layer is the heated patch
	/// @return the counts of the flagged cells
	static SourceCells flag_sources(LBM& lbm, bool patch) {
		SourceCells sources{};
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			if(z != 0u) { // not the source layer
				continue;
			}
			const bool in_patch{ patch && x < cells_x / 2u };
			if(in_patch) {
				lbm.flags[n] = TYPE_X | TYPE_Y;
				sources.patched++;
			} else {
				lbm.flags[n] = TYPE_Y;
				sources.plain++;
			}
		}
		return sources;
	}

	/// @brief Gives the mean temperature rise of all cells.
	/// @param lbm the box's lattice
	/// @return the mean of T - 1 in lattice temperature
	static double mean_rise(LBM& lbm) {
		lbm.update_fields();
		lbm.T.read_from_device();
		double sum{ 0.0 };
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			sum += static_cast<double>(lbm.T[n]) - 1.0;
		}
		const double mean{ sum / static_cast<double>(cells) };
		return mean;
	}

	/// @brief Gives the mean temperature rise of the layer farthest from the source (z = Nz/2), from the last read.
	/// @param lbm the box's lattice
	/// @return the mean of T - 1 in lattice temperature
	static double middle_layer_rise(LBM& lbm) {
		double middle{ 0.0 };
		ulong middle_cells{ 0ull };
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			if(z == cells_z / 2u) {
				middle += static_cast<double>(lbm.T[n]) - 1.0;
				middle_cells++;
			}
		}
		const double mean{ middle / static_cast<double>(middle_cells) };
		return mean;
	}

	static constexpr uint cells_x = 16u;      ///< the box's cells along x
	static constexpr uint cells_y = 16u;      ///< the box's cells along y
	static constexpr uint cells_z = 16u;      ///< the box's cells along z
	static constexpr float viscosity = 0.1f;  ///< the kinematic viscosity in lattice units
	static constexpr float alpha = 0.05f;     ///< the thermal diffusion coefficient in lattice units
	/// the temperature added per time step to each source cell; large against the rounding of T = 1+sum(g) in float
	/// (~1E-7 per step, the same in every cell of a layer here, so it does not average out)
	static constexpr float heat_rate = 1.0E-3f;
	static constexpr ulong steps = 1000ull;        ///< the steps of each of the two runs
	static constexpr double patch_factor = 4.0;    ///< the kernel's fixed factor of the heated patch
	static constexpr double heat_limit = 0.001;    ///< the largest relative error of the added heat that passes (0.1 %)
	static constexpr double reach_limit = 0.5;     ///< a cold farthest layer fails (error 1), a warmed one passes (error 0)
};

} // namespace

void main_setup() { // extensions: TEMPERATURE
	HeatBox::run("heat source", false);
	HeatBox::run("heat source with the heated patch (TYPE_X|TYPE_Y, 4x)", true);
}
