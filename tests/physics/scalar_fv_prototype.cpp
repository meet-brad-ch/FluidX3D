// ProLB's finite-volume scheme (Feng et al. 2021,
// JAMES, sect. 3.2; scalar_fv_reference.hpp) carries T and Q on the host, fed on every step with the GPU's equilibrium
// velocity and diffusivity D = (1/wT - 1/2)/4 from the cell probe (DEBUG_PROBE). The case is the moist thermal of
// moist_bubble.cpp (moist_thermal_case.hpp). The FV T is passive: the flow follows the GPU's D3Q7 T, so the two
// schemes are compared in the same flow.
//
// Gates: the uniform qt stays uniform (1e-6 kg/kg; the D3Q7 lattice drifts 1.6e-4 in this flow), the FV T excess
// stays inside its start range (1 % of the peak), the FV T centroid rises within 5 % of the GPU's.

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

/// What the finite-volume prototype did on the GPU's flow.
struct FiniteVolumeRun {
	double qt_drift = 0.0;        ///< kg/kg, the largest |qt - qt_start| of the FV moisture at the end
	double gpu_qt_drift = 0.0;    ///< kg/kg, the same of the GPU's D3Q7 moisture
	double gpu_rise = 0.0;        ///< cells, the rise of the GPU's T-excess centroid
	double fv_rise = 0.0;         ///< cells, the same of the FV T (passive: the flow follows the GPU's T)
	double fv_over = 0.0;         ///< K, how far the FV T excess went above its start maximum
	double fv_under = 0.0;        ///< K, how far it went below 0
};

/// @brief The centroid height (cells) of the positive excess T - 1 over the fluid cells.
template<typename Field> double excess_centroid(const Field& value, const std::vector<bool>& solid, const uint Nx, const uint Ny, const ulong N) {
	double weight = 0.0, moment = 0.0;
	for(ulong n=0ull; n<N; n++) {
		if(solid[n]) continue;
		const double e = value(n)-1.0;
		if(e>0.0) { weight += e; moment += e*((double)(n/((ulong)Nx*Ny))+0.5); }
	}
	return moment/weight;
}

/// @brief POC step 1: ProLB's finite-volume scheme for T and Q on the host, fed with the GPU's u_eq and its
/// diffusivity D = (1/wT - 1/2)/4 (the D3Q7 relation with THERMAL_DIFFUSIVITY_FIX, the eddy part included).
FiniteVolumeRun compare_fv(const ThermalBox& box, const ulong steps) {
	const auto excess = [&](const uint x, const uint y, const uint z) { return box.excess*box.shape(x, y, z); };
	LBM lbm(box.N, box.N, box.Nz, box.nu, 0.0f, 0.0f, box.fz, 0.0f, box.alpha, box.beta());
	const double excess_max = box.set_up(lbm, box.moist_qt, excess);
	const ulong N = lbm.get_N();
	std::vector<float> T0(N, 0.0f), Q0(N, 0.0f);
	for(ulong n=0ull; n<N; n++) { T0[n] = lbm.T[n]; Q0[n] = lbm.Q[n]; }
	const std::vector<bool> solid = ScalarReference::solid_of(lbm);
	FiniteVolumeScalar fv_T(lbm.get_Nx(), lbm.get_Ny(), lbm.get_Nz(), solid, T0), fv_Q(lbm.get_Nx(), lbm.get_Ny(), lbm.get_Nz(), solid, Q0);
	Memory<float>& probe = lbm.debug_cells();
	const uint first = DebugProbe::index(DebugProbe::Cell::ux_eq), last = DebugProbe::index(DebugProbe::Cell::Q);
	const float* u_eq = &probe[(ulong)DebugProbe::index(DebugProbe::Cell::ux_eq)*N];
	const float* w_T = &probe[(ulong)DebugProbe::index(DebugProbe::Cell::w_T)*N];
	const float* gpu_T = &probe[(ulong)DebugProbe::index(DebugProbe::Cell::T)*N];
	const float* gpu_Q = &probe[(ulong)DebugProbe::index(DebugProbe::Cell::Q)*N];
	std::vector<float> D(N, 0.0f);
	const double Q_start = 1.0+box.moist_qt/0.01;
	FiniteVolumeRun r;
	double gpu_start = 0.0, fv_start = 0.0;
	for(ulong step=1ull; step<=steps; step++) {
		lbm.run(1ull);
		probe.read_from_device((ulong)first*N, (ulong)(last-first+1u)*N);
		if(step==1ull) {
			gpu_start = excess_centroid([&](const ulong n) { return (double)gpu_T[n]; }, solid, box.N, box.N, N);
			fv_start = excess_centroid([&](const ulong n) { return fv_T.at(n); }, solid, box.N, box.N, N);
		}
		for(ulong n=0ull; n<N; n++) D[n] = solid[n] ? 0.0f : 0.25f*(1.0f/w_T[n]-0.5f);
		fv_T.step(u_eq, D.data()); fv_Q.step(u_eq, D.data());
		if(step%200ull==0ull||step==steps) {
			double qd = 0.0, gqd = 0.0;
			for(ulong n=0ull; n<N; n++) {
				if(solid[n]) continue;
				qd = std::fmax(qd, std::fabs(fv_Q.at(n)-Q_start)); gqd = std::fmax(gqd, std::fabs((double)gpu_Q[n]-Q_start));
				const double e = fv_T.at(n)-1.0;
				r.fv_over = std::fmax(r.fv_over, e-excess_max); r.fv_under = std::fmax(r.fv_under, -e);
			}
			r.qt_drift = 0.01*qd; r.gpu_qt_drift = 0.01*gqd;
			r.gpu_rise = excess_centroid([&](const ulong n) { return (double)gpu_T[n]; }, solid, box.N, box.N, N)-gpu_start;
			r.fv_rise = excess_centroid([&](const ulong n) { return fv_T.at(n); }, solid, box.N, box.N, N)-fv_start;
			std::printf("\n  step %llu: the qt drift FV %.2e, GPU %.2e kg/kg; the T-excess centroid rose FV %.3f, GPU %.3f cells; FV T excess over %.4f K, under 0 %.4f K\n",
			            (unsigned long long)step, r.qt_drift, r.gpu_qt_drift, r.fv_rise, r.gpu_rise, r.fv_over, r.fv_under);
		}
	}
	return r;
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID LEVEL_FORCING MOISTURE DEBUG_PROBE (the CBL configuration and the probe)
	const ThermalBox box;
	const ulong steps = 1400ull;
	std::printf("\nfinite-volume prototype: ProLB's scheme for T and Q on the GPU's flow, %llu steps\n", (unsigned long long)steps);
	const FiniteVolumeRun fv = compare_fv(box, steps);
	PhysicsCheck::report("finite-volume prototype: the uniform qt stays uniform (kg/kg over 1e-6)", fv.qt_drift/1.0E-6, 1.0);
	PhysicsCheck::report("finite-volume prototype: the T excess stays in its start range (K over 1 % of the peak)", std::fmax(fv.fv_over, fv.fv_under)/(0.01*box.excess), 1.0);
	PhysicsCheck::report("finite-volume prototype: the T centroid rises as the GPU's (relative)", std::fabs(fv.fv_rise-fv.gpu_rise)/fv.gpu_rise, 0.05);
}
