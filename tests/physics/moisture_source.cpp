// MOISTURE: the second scalar lattice. Total water qt rides on a D3Q7
// lattice hi, the twin of the thermal gi, as Q = 1 + qt / def_q_scale, with the thermal relaxation (the same molecular and
// eddy diffusivity) and a surface source in TYPE_Y cells (LBM::set_moisture_rate(), per step and cell, as the heat rate).
// 1. The budget: a periodic box of fluid at rest whose layer z = 0 is TYPE_Y, moistened by moisture_rate; without walls
//    every bit of added water stays in the fluid cells: the mean Q rises by moisture_rate x (source cells) / (cells) per
//    step while diffusion carries it away from the layer. Passes when the mean rise from step 1000 to 2000 is within
//    0.1 % of that, and the layer farthest from the source has moistened.
// 2. The domains: the same box with a moist blob and a flow, one domain against two along x, after 500 steps the Q
//    fields equal to the last bit (the halo exchange of hi; with SCALAR_FV the finite volumes' 2-layer exchange).

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

/// The Q field of a box with a moist sphere in a uniform flow after some steps.
std::vector<float> blob_after(const uint Dx, const ulong steps) {
	const uint N = 32u;
	LBM lbm(N, N, N, Dx, 1u, 1u, 0.02f, 0.0f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f);
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		lbm.T[n] = 1.0f;
		lbm.Q[n] = 1.0f+(sq((float)x-10.0f)+sq((float)y-16.0f)+sq((float)z-16.0f)<36.0f ? 0.1f : 0.0f); // a sphere of radius 6 near the x halo
		lbm.u.x[n] = 0.05f;
	}
	lbm.run(steps);
	lbm.update_fields();
	lbm.Q.read_from_device();
	std::vector<float> q(lbm.get_N());
	for(ulong n=0ull; n<lbm.get_N(); n++) q[n] = lbm.Q[n];
	return q;
}

} // namespace

void main_setup() { // extensions: TEMPERATURE MOISTURE
	{ // 1. the budget
		const uint Nx = 16u, Ny = 16u, Nz = 16u;
		const float alpha = 0.05f;
		const float moisture_rate = 1.0E-3f; // Q added per step to each source cell
		const ulong steps = 1000ull;
		LBM lbm(Nx, Ny, Nz, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f);
		ulong sources = 0ull;
		for(ulong n=0ull; n<lbm.get_N(); n++) {
			uint x=0u, y=0u, z=0u;
			lbm.coordinates(n, x, y, z);
			lbm.T[n] = 1.0f; lbm.Q[n] = 1.0f;
			if(z==0u) { lbm.flags[n] = TYPE_Y; sources++; }
		}
		lbm.set_moisture_rate(moisture_rate);
		const auto mean_rise = [&]() {
			lbm.update_fields();
			lbm.Q.read_from_device();
			double sum = 0.0;
			for(ulong n=0ull; n<lbm.get_N(); n++) sum += (double)lbm.Q[n]-1.0;
			return sum/(double)lbm.get_N();
		};
		lbm.run(steps);
		const double rise1 = mean_rise();
		lbm.run(steps);
		const double rise2 = mean_rise();
		double middle = 0.0;
		for(ulong n=0ull; n<lbm.get_N(); n++) {
			uint x=0u, y=0u, z=0u;
			lbm.coordinates(n, x, y, z);
			if(z==Nz/2u) middle += (double)lbm.Q[n]-1.0;
		}
		const double expected = (double)moisture_rate*(double)sources*(double)steps/(double)lbm.get_N();
		std::printf("\nmean Q rise %.6e after %llu steps, %.6e after %llu (expected per %llu steps %.6e), the farthest layer %.6e\n",
		            rise1, steps, rise2, 2ull*steps, steps, expected, middle);
		PhysicsCheck::report("moisture source: the added water", std::fabs((rise2-rise1)/expected-1.0), 0.001);
		PhysicsCheck::report("moisture source: the water reached the farthest layer", middle>0.0 ? 0.0 : 1.0, 0.5);
	}
	{ // 2. one domain against two
		const std::vector<float> one = blob_after(1u, 500ull), two = blob_after(2u, 500ull);
		float largest = 0.0f, peak = 0.0f;
		for(size_t n=0; n<one.size(); n++) { largest = std::fmax(largest, std::fabs(one[n]-two[n])); peak = std::fmax(peak, one[n]-1.0f); }
		std::printf("\nmoist blob after 500 steps: the largest difference between one and two domains %.3e (the blob's peak Q - 1 %.4f)\n", largest, peak);
#ifdef SCALAR_FV
		PhysicsCheck::report("moisture, finite volumes: two domains against one (bit-identical: absolute, any difference fails)", largest>0.0f ? 1.0 : 0.0, 0.5);
#else // SCALAR_FV
		PhysicsCheck::report("moisture lattice: two domains against one (absolute, limit 1e-6)", (double)largest, 1.0E-6);
#endif // SCALAR_FV
	}
}
