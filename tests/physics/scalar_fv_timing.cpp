// The cost of each kernel of one time step with the link-flux finite volumes (SCALAR_FV, with MOISTURE): a box of the
// size of a dry CBL at 10 m cells (256 x 256 x 207, TYPE_S floor and ceiling, the first fluid layer heated as TYPE_Y, Smagorinsky,
// a small start noise in u so the flow is not at rest), warmed up for 300 steps, then 500 steps timed by
// LBM::time_fv_step(), which finishes the queue around each kernel. Prints the mean ms per step of stream_collide, the
// half stage and the full stage, and their shares. A second block of 500 steps runs as usual (LBM::run()) for the
// cost of a normal step. Passes when the sum of the timed kernels is within 10 % of the normal step (the timing's own
// overhead small enough for the shares to hold).

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <cstdio>
#include <random>

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE SUBGRID THERMAL_DIFFUSIVITY_FIX LEVEL_FORCING MOISTURE SCALAR_FV
	const uint Nx{ 256u }, Ny{ 256u }, Nz{ 207u };
	const ulong warm{ 300ull }, timed{ 500ull };
	LBM lbm(Nx, Ny, Nz, 1.0E-4f, 0.0f, 0.0f, -1.0E-6f, 0.0f, 0.014f, 1.0f);
	std::mt19937 random(2u);
	std::uniform_real_distribution<float> noise(-0.005f, 0.005f);
	const ulong N{ lbm.get_N() };
	for(ulong n{}; n<N; n++) {
		uint x{}, y{}, z{};
		lbm.coordinates(n, x, y, z);
		if(z==0u||z==Nz-1u) { lbm.flags[n] = TYPE_S; continue; }
		if(z==1u) lbm.flags[n] = TYPE_Y;
		lbm.u.x[n] = noise(random); lbm.u.y[n] = noise(random); lbm.u.z[n] = noise(random);
		lbm.Q[n] = 1.5f; // some water: the moisture finite volume does its full work
	}
	lbm.set_heat_rate(1.0E-5f);
	lbm.run(0ull);
	lbm.run(warm);
	FvKernelTimes sum{};
	for(ulong step{}; step<timed; step++) {
		const FvKernelTimes t{ lbm.time_fv_step() };
		sum.stream_collide += t.stream_collide; sum.fv_half += t.fv_half; sum.fv_full += t.fv_full;
	}
	const double sc{ sum.stream_collide/(double)timed }, half{ sum.fv_half/(double)timed }, full{ sum.fv_full/(double)timed };
	const double kernels{ sc+half+full };
	const Clock clock;
	lbm.run(timed);
	const double normal{ 1.0E3*clock.stop()/(double)timed };
	std::printf("\nSCALAR_FV kernel times, %u x %u x %u = %.2f M cells, the mean of %llu steps:\n", Nx, Ny, Nz, (double)N*1.0E-6, (unsigned long long)timed);
	std::printf("  stream_collide %.3f ms (%.1f %%), half stage %.3f ms (%.1f %%), full stage %.3f ms (%.1f %%), sum %.3f ms\n",
	            sc, 100.0*sc/kernels, half, 100.0*half/kernels, full, 100.0*full/kernels, kernels);
	std::printf("  a normal step (LBM::run, %llu steps): %.3f ms, %.0f MLUPs\n", (unsigned long long)timed, normal, (double)N*1.0E-3/normal);
	PhysicsCheck::report("SCALAR_FV timing: the timed kernels against a normal step (relative)", std::fabs(kernels/normal-1.0), 0.10);
}
