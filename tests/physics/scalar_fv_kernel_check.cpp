// SCALAR_FV: the kernel's conservative finite-volume T and Q (fv_scalars() in kernel.cpp, the LBM's link mass fluxes)
// against the host twin LinkFluxFiniteVolumeScalars (scalar_fv_reference.hpp), every cell, every step. The host gets
// the GPU's own inputs: the DDFs a step stored (LBM::debug_populations(), the 18 link fluxes) and the relaxation rate wT
// from the cell probe (DEBUG_PROBE), from which the kernel's diffusivity D = (1/wT - 1/2)/4 follows. Here the FV T
// drives the flow (the buoyancy reads the field T).
//
// - Code check: the GPU's T and Q (the fields at the start of each step) against the host's after the same number of
//   updates; a difference beyond float rounding (the host computes in double) is a code defect, and its first step
//   and cell are printed.
// - The uniform qt stays uniform on the GPU (the gate moist_bubble failed with the D3Q7 lattice).
//
// The case: the moist thermal of moist_bubble.cpp (moist_thermal_case.hpp), the CBL configuration.

#include "setup/setup.hpp"
#include "physics_check.hpp"
#include "moist_thermal_case.hpp"
#include "debug_probe.hpp"
#include "scalar_reference_lattice.hpp"
#include "scalar_fv_reference.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

/// The largest GPU-host difference of one field, relative to the size of the stored float value, and where it first
/// passed the tolerance. The float rounding of the GPU's fields scales with their size: Q - 1 = 2.6 differed by
/// 2.4e-6 (10 float steps), Q - 1 = 0.36 by 2.4e-7 (8 float steps), the same 1400 steps.
struct FieldDifference {
	double largest{};     ///< over the run, |GPU - host| / max(1, |stored value|)
	ulong first_step{};   ///< the first step beyond the tolerance (0: none)
	ulong first_cell{};
	double first_gpu{}, first_host{};
	/// @brief Takes one cell's GPU and host values.
	/// @param stored the float value the GPU stores (T or Q, not shifted); it sets the scale of the rounding
	void take(const double gpu, const double host, const double stored, const double tolerance, const ulong step, const ulong cell) {
		const double d{ std::fabs(gpu-host)/std::fmax(1.0, std::fabs(stored)) };
		largest = std::fmax(largest, d);
		if(d>tolerance&&first_step==0ull) { first_step = step; first_cell = cell; first_gpu = gpu; first_host = host; }
	}
	/// @brief Prints the result for a field name.
	void print(const char* what, const uint Nx, const uint Ny) const {
		if(first_step==0ull) { std::printf("%s: within the tolerance on every step (largest %.2e)\n", what, largest); return; }
		const uint x{ (uint)(first_cell%Nx) }, y{ (uint)((first_cell/Nx)%Ny) }, z{ (uint)(first_cell/((ulong)Nx*Ny)) };
		std::printf("%s: first beyond the tolerance at step %llu, cell (%u, %u, %u): GPU %.7f, host %.7f; largest %.2e\n", what, (unsigned long long)first_step, x, y, z, first_gpu, first_host, largest);
	}
};

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID LEVEL_FORCING MOISTURE DEBUG_PROBE SCALAR_FV
	const ThermalBox box{};
	const ulong steps{ 1400ull };
	const double tolerance{ 1.0E-6 }; // relative to max(1, |T|) and max(1, |Q|): the shifted float fields against the double host (T read as a float: 1.2e-7 steps); unshifted float T drifted 1.7e-5
	const auto excess = [&](const uint x, const uint y, const uint z) { return box.excess*box.shape(x, y, z); };
	LBM lbm(box.N, box.N, box.Nz, box.nu, 0.0f, 0.0f, box.fz, 0.0f, box.alpha, box.beta());
	box.set_up(lbm, box.moist_qt, excess);
	const ulong N{ lbm.get_N() };
	std::vector<float> rho0(N, 0.0f);
	std::vector<double> theta0(N, 0.0), q0(N, 0.0); // the shifted fields, as fv_init() sets them
	for(ulong n{}; n<N; n++) { rho0[n] = lbm.rho[n]; theta0[n] = (double)lbm.T[n]-1.0; q0[n] = (double)lbm.Q[n]-1.0; }
	const std::vector<bool> solid{ ScalarReference::solid_of(lbm) };
	LinkFluxFiniteVolumeScalars host(lbm.get_Nx(), lbm.get_Ny(), lbm.get_Nz(), solid, rho0, std::vector<std::vector<double>>{ theta0, q0 });
	Memory<float>& probe{ lbm.debug_cells() };
	Memory<fpxx>& populations{ lbm.debug_populations() };
	std::vector<double> psi;
	const uint first{ DebugProbe::index(DebugProbe::Cell::ux_eq) }, last{ DebugProbe::index(DebugProbe::Cell::Q) };
	const float* w_T{ &probe[(ulong)DebugProbe::index(DebugProbe::Cell::w_T)*N] };
	const float* gpu_T{ &probe[(ulong)DebugProbe::index(DebugProbe::Cell::T)*N] };
	const float* gpu_Q{ &probe[(ulong)DebugProbe::index(DebugProbe::Cell::Q)*N] };
	std::vector<float> D(N, 0.0f);
	const double Q_start{ 1.0+box.moist_qt/0.01 };
	FieldDifference dT{}, dQ{};
	double qt_drift{};
	std::printf("\nSCALAR_FV kernel check: %llu steps, tolerance %.0e (relative to max(1, |value|))\n", (unsigned long long)steps, tolerance);
	for(ulong step{ 1ull }; step<=steps; step++) {
		lbm.run(1ull);
		probe.read_from_device((ulong)first*N, (ulong)(last-first+1u)*N);
		for(ulong n{}; n<N; n++) { // the GPU's fields at the start of this step against the host's after step-1 updates
			if(solid[n]) continue;
			dT.take((double)gpu_T[n]-1.0, host.at(0ull, n), (double)gpu_T[n], tolerance, step, n);
			dQ.take((double)gpu_Q[n]-1.0, host.at(1ull, n), (double)gpu_Q[n], tolerance, step, n);
			qt_drift = std::fmax(qt_drift, 0.01*std::fabs((double)gpu_Q[n]-Q_start));
		}
		for(ulong n{}; n<N; n++) D[n] = solid[n] ? 0.0f : 0.25f*(1.0f/w_T[n]-0.5f);
		populations.read_from_device();
		host.link_fluxes(populations.data(), lbm.get_t(), psi);
		host.step(psi, D.data());
		if(step%200ull==0ull) std::printf("\n  step %llu: the largest relative |GPU - host| T %.2e, Q %.2e; the GPU's qt drift %.2e kg/kg\n", (unsigned long long)step, dT.largest, dQ.largest, qt_drift);
	}
	std::printf("\n");
	dT.print("T", box.N, box.N);
	dQ.print("Q", box.N, box.N);
	PhysicsCheck::report("SCALAR_FV kernel check: the kernel's T against the host twin (relative, over 1e-6)", dT.largest/tolerance, 1.0);
	PhysicsCheck::report("SCALAR_FV kernel check: the kernel's Q against the host twin (relative, over 1e-6)", dQ.largest/tolerance, 1.0);
	PhysicsCheck::report("SCALAR_FV kernel check: the uniform qt stays uniform on the GPU (kg/kg over 1e-6)", qt_drift/1.0E-6, 1.0);
}
