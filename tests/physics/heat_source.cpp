// Heat source: a periodic box of fluid at rest whose layer z = 0 is flagged TYPE_Y, heated by LBM::set_heat_rate().
// Without walls every bit of added heat stays in the fluid cells: the mean temperature rises by
// heat_rate * (source cells) / (cells) per time step, while diffusion carries the heat away from the layer.
// (Next to TYPE_S walls a sum over the fluid cells misses heat: esoteric pull returns a population sent into a wall
// two steps later, so one population per wall link sits in the wall's own slots at any time.)
// Passes when the mean temperature rise from step 1000 to 2000 is within 0.1 % of that, and the layer farthest from the
// source has warmed.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>

void main_setup() { // extensions: TEMPERATURE
	const uint Nx = 16u, Ny = 16u, Nz = 16u;
	const float alpha = 0.05f;       // thermal diffusion coefficient
	const float heat_rate = 1.0E-3f; // temperature added per time step to each source cell; large against the rounding of T = 1+sum(g) in float (~1E-7 per step, the same in every cell of a layer here, so it does not average out)
	const ulong steps = 1000ull;
	LBM lbm(Nx, Ny, Nz, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f); // no gravity: the fluid stays at rest

	ulong sources = 0ull;
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		if(z==0u) { lbm.flags[n] = TYPE_Y; sources++; } // the source layer
	}
	lbm.set_heat_rate(heat_rate);
	const auto mean_rise = [&]() { // the mean temperature rise of all cells
		lbm.update_fields();
		lbm.T.read_from_device();
		double sum = 0.0;
		for(ulong n=0ull; n<lbm.get_N(); n++) sum += (double)lbm.T[n]-1.0;
		return sum/(double)lbm.get_N();
	};
	lbm.run(steps);
	const double rise1 = mean_rise();
	lbm.run(steps);
	const double rise2 = mean_rise();
	double middle = 0.0;
	ulong middle_cells = 0ull;
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		if(z==Nz/2u) { middle += (double)lbm.T[n]-1.0; middle_cells++; }
	}
	const double expected = (double)heat_rate*(double)sources*(double)steps/(double)lbm.get_N(); // mean rise over one run of steps
	std::printf("\nmean temperature rise %.6e after %llu steps, %.6e after %llu (expected per %llu steps %.6e), farthest layer %.6e\n",
	            rise1, steps, rise2, 2ull*steps, steps, expected, middle/(double)middle_cells);
	PhysicsCheck::report("heat source: the added heat", std::fabs((rise2-rise1)/expected-1.0), 0.001);
	PhysicsCheck::report("heat source: the heat reached the farthest layer", middle>0.0 ? 0.0 : 1.0, 0.5);
}
