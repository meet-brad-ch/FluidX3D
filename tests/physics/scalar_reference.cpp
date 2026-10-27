// DEBUG_PROBE: the kernel's thermal and moisture lattices against a host reference, step by step. It separates code defects from properties of the scheme.
//
// The host reference is a plain D3Q7 BGK scalar lattice with push streaming and bounce-back at TYPE_S cells, written
// from the equations the kernel claims to solve: the equilibrium of calculate_g_eq() (DDF-shifted, linear in u, c^2 =
// 1/4), the collision g = (1 - wT) g + wT g_eq. Its inputs on every step are the GPU's own: the velocity of the scalar
// equilibria (j/rho + F/(2 rho)) and the relaxation rate wT of every cell, read from the cell probe. So the host does
// exactly what the scheme prescribes with the GPU's flow, and nothing else.
//
// - Code check: the GPU's T and Q (Sum g + 1 before the collision) and the scalar fluxes (Sum c g) against the host's,
//   every cell, every step. A difference beyond float rounding is a code defect; the first step and cell are printed.
// - Scheme check: the host's Q starts uniform (26 g/kg); its drift is the scheme's own (the GPU's flow, the textbook
//   operator). If the host drifts as the GPU does, the drift is a property of the scheme, not of the code.
//
// The case: the moist thermal of moist_bubble.cpp (moist_thermal_case.hpp), the CBL configuration.

#include "setup/setup.hpp"
#include "physics_check.hpp"
#include "moist_air.hpp"
#include "moist_thermal_case.hpp"
#include "debug_probe.hpp"
#include "scalar_reference_lattice.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

namespace {



/// The largest difference of one quantity over the fluid cells, and where it was first beyond the tolerance.
struct Difference {
	double largest = 0.0;    ///< over the run
	ulong largest_cell = 0ull;
	ulong first_step = 0ull; ///< the first step beyond the tolerance (0: none)
	ulong first_cell = 0ull;
	double first_gpu = 0.0, first_host = 0.0;
	/// @brief Takes one cell's GPU and host values.
	void take(const double gpu, const double host, const double tolerance, const ulong step, const ulong cell) {
		const double d = std::fabs(gpu-host);
		if(d>largest) { largest = d; largest_cell = cell; }
		if(d>tolerance&&first_step==0ull) { first_step = step; first_cell = cell; first_gpu = gpu; first_host = host; }
	}
};

/// The fit of the uniform scalar's relative change against a time-integrated divergence.
struct DivergenceFit {
	double slope = 0.0;       ///< dQ/Q against -Sum_t div (1: the divergence explains the change)
	double correlation = 0.0; ///< over the cells with a change above 10 % of the largest
};

/// @brief Accumulates, per fluid cell, the two discrete divergences a uniform field sees: the D3Q7 central difference of
/// the scalar equilibrium velocity (what the linear D3Q7 equilibrium does to a uniform Q on every step:
/// dQ = -Q div_7(u_eq)) and the D3Q19-weighted divergence of rho u_eq (the flow's mass continuity, drho = -div_19(rho u)).
class DivergenceBudget {
public:
	explicit DivergenceBudget(LBM& lbm) : Nx(lbm.get_Nx()), Ny(lbm.get_Ny()), Nz(lbm.get_Nz()), N(lbm.get_N()), d7(lbm.get_N(), 0.0), d19(lbm.get_N(), 0.0) {}
	/// @brief Adds this step's divergences. @param rho the probe's rho channel @param u the probe's u_eq channels (3 N)
	void take(const float* rho, const float* u) {
		for(uint z=1u; z+1u<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
			const ulong n = index(x, y, z);
			const ulong xp = index((x+1u)%Nx, y, z), xm = index((x+Nx-1u)%Nx, y, z), yp = index(x, (y+1u)%Ny, z), ym = index(x, (y+Ny-1u)%Ny, z), zp = index(x, y, z+1u), zm = index(x, y, z-1u);
			d7[n] += 0.5*(((double)u[xp]-(double)u[xm])+((double)u[N+yp]-(double)u[N+ym])+((double)u[2ull*N+zp]-(double)u[2ull*N+zm]));
			double div = 0.0; // 3 Sum_i w_i c_i . (rho u)(n + c_i), D3Q19: axes 1/18, diagonals 1/36
			for(int dz=-1; dz<=1; dz++) for(int dy=-1; dy<=1; dy++) for(int dx=-1; dx<=1; dx++) {
				const int a = std::abs(dx)+std::abs(dy)+std::abs(dz);
				if(a==0||a==3) continue;
				const double w = a==1 ? 1.0/18.0 : 1.0/36.0;
				const ulong m = index((x+Nx+(uint)(dx+(int)Nx))%Nx, (y+Ny+(uint)(dy+(int)Ny))%Ny, (uint)((int)z+dz));
				const double r = (double)rho[m];
				div += 3.0*w*r*((double)dx*(double)u[m]+(double)dy*(double)u[N+m]+(double)dz*(double)u[2ull*N+m]);
			}
			d19[n] += div;
		}
	}
	/// @brief Fits the relative change of a field against -Sum div_7 (stencil 7) or -Sum div_19 (stencil 19).
	DivergenceFit fit(const ScalarReference& field, const double start, const uint stencil) const {
		const std::vector<double>& d = stencil==7u ? d7 : d19;
		double largest = 0.0;
		for(ulong n=0ull; n<N; n++) if(!field.is_solid(n)) largest = std::fmax(largest, std::fabs(((double)field.at(n)-start)/start));
		double sxy = 0.0, sxx = 0.0, syy = 0.0;
		for(ulong n=0ull; n<N; n++) {
			if(field.is_solid(n)) continue;
			const double y = ((double)field.at(n)-start)/start, x = -d[n];
			if(std::fabs(y)<0.1*largest) continue;
			sxy += x*y; sxx += x*x; syy += y*y;
		}
		return { sxy/sxx, sxy/std::sqrt(sxx*syy) };
	}
	/// @brief The validation of the budget: the GPU's density change (rho - 1) against -Sum div (stencil 7 or 19).
	DivergenceFit fit_density(const float* rho, const uint stencil) const {
		const std::vector<double>& d = stencil==7u ? d7 : d19;
		double sxy = 0.0, sxx = 0.0, syy = 0.0;
		for(uint z=1u; z+1u<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
			const ulong n = index(x, y, z);
			const double yv = (double)rho[n]-1.0, xv = -d[n];
			sxy += xv*yv; sxx += xv*xv; syy += yv*yv;
		}
		return { sxy/sxx, sxy/std::sqrt(sxx*syy) };
	}
private:
	uint Nx = 0u, Ny = 0u, Nz = 0u;
	ulong N = 0ull;
	std::vector<double> d7, d19;
	ulong index(const uint x, const uint y, const uint z) const { return (ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx; }
};

/// What one comparison run found.
struct Comparison {
	Difference T, Q, flux;
	DivergenceFit fit7, fit19; ///< the host's uniform-Q change against -Sum div_7(u_eq) and -Sum div_19(rho u_eq)
	double gpu_drift = 0.0;  ///< kg/kg, the largest |qt - qt_start| of the GPU at the end
	double host_drift = 0.0; ///< kg/kg, the same of the host reference
	std::array<float, 7> gpu_h_pre{}, host_h_pre{}; ///< the traced cell's moisture populations before the collision at the last step
	std::array<float, 7> gpu_g_pre{}, host_g_pre{}; ///< the same of the thermal lattice
	float traced_step = 0.0f; ///< the step the trace record holds
};

/// @brief Runs the moist thermal on the GPU and the host reference side by side.
/// @param box the case
/// @param steps the steps to run
/// @param trace the traced cell (x, y, z)
/// @param tolerance the GPU-host difference counted as a mismatch (lattice units)
/// @param report print a progress line every 100 steps
/// @param bounce_back_delay the host reference's bounce-back delay in steps (1 standard, 2 the Esoteric-Pull hypothesis)
/// @param variant the host's scheme: 0 the kernel's, 1 the D3Q19-consistent equilibrium velocity, 2 the advective form, 3 wT forced to 1, 4 the two-step velocity mean, 5 the Chopard et al. 2009 correction
/// @return the differences, the drifts and the traced cell's populations at the last step
Comparison compare(const ThermalBox& box, const ulong steps, const uint3 trace, const float tolerance, const bool report, const uint bounce_back_delay, const uint variant) {
	const auto excess = [&](const uint x, const uint y, const uint z) { return box.excess*box.shape(x, y, z); };
	LBM lbm(box.N, box.N, box.Nz, box.nu, 0.0f, 0.0f, box.fz, 0.0f, box.alpha, box.beta());
	box.set_up(lbm, box.moist_qt, excess);
	const ulong N = lbm.get_N();
	std::vector<float> T0(N, 0.0f), Q0(N, 0.0f);
	for(ulong n=0ull; n<N; n++) { T0[n] = lbm.T[n]; Q0[n] = lbm.Q[n]; }
	const std::vector<bool> solid = ScalarReference::solid_of(lbm);
	ScalarReference ref_T(lbm.get_Nx(), lbm.get_Ny(), lbm.get_Nz(), solid, T0, bounce_back_delay), ref_Q(lbm.get_Nx(), lbm.get_Ny(), lbm.get_Nz(), solid, Q0, bounce_back_delay);
	ref_T.use_d3q19_consistent_velocity(variant==1u); ref_Q.use_d3q19_consistent_velocity(variant==1u);
	ref_T.use_advective_form(variant==2u); ref_Q.use_advective_form(variant==2u);
	ref_T.override_rate(variant==3u ? 1.0f : 0.0f); ref_Q.override_rate(variant==3u ? 1.0f : 0.0f);
	ref_T.average_velocity_over_two_steps(variant==4u); ref_Q.average_velocity_over_two_steps(variant==4u);
	ref_T.use_chopard_correction(variant==5u); ref_Q.use_chopard_correction(variant==5u);
	lbm.set_trace_cell(trace.x, trace.y, trace.z);
	Memory<float>& probe = lbm.debug_cells();
	const uint first = DebugProbe::index(DebugProbe::Cell::rho), last = DebugProbe::index(DebugProbe::Cell::hflux_z); // one contiguous read
	const float* rho = &probe[(ulong)DebugProbe::index(DebugProbe::Cell::rho)*N];
	DivergenceBudget budget(lbm);
	const float* u_eq = &probe[(ulong)DebugProbe::index(DebugProbe::Cell::ux_eq)*N];
	const float* w_T = &probe[(ulong)DebugProbe::index(DebugProbe::Cell::w_T)*N];
	const float* gpu_T = &probe[(ulong)DebugProbe::index(DebugProbe::Cell::T)*N];
	const float* gpu_Q = &probe[(ulong)DebugProbe::index(DebugProbe::Cell::Q)*N];
	const float* gpu_hflux = &probe[(ulong)DebugProbe::index(DebugProbe::Cell::hflux_x)*N];
	const double Q_start = 1.0+box.moist_qt/0.01;
	const ulong traced = lbm.index(trace.x, trace.y, trace.z);
	Comparison result;
	for(ulong step=1ull; step<=steps; step++) {
		lbm.run(1ull);
		probe.read_from_device((ulong)first*N, (ulong)(last-first+1u)*N);
		ref_T.stream(); ref_Q.stream();
		double host_drift = 0.0, gpu_drift = 0.0;
		for(ulong n=0ull; n<N; n++) {
			if(ref_Q.is_solid(n)) continue;
			result.T.take(gpu_T[n], ref_T.at(n), tolerance, step, n);
			result.Q.take(gpu_Q[n], ref_Q.at(n), tolerance, step, n);
			for(uint d=0u; d<3u; d++) result.flux.take(gpu_hflux[(ulong)d*N+n], ref_Q.flux(n, d), tolerance, step, n);
			host_drift = std::fmax(host_drift, std::fabs((double)ref_Q.at(n)-Q_start));
			gpu_drift = std::fmax(gpu_drift, std::fabs((double)gpu_Q[n]-Q_start));
		}
		result.gpu_drift = 0.01*gpu_drift; result.host_drift = 0.01*host_drift;
		if(step==steps) for(uint i=0u; i<7u; i++) { result.host_h_pre[i] = ref_Q.population(traced, i); result.host_g_pre[i] = ref_T.population(traced, i); }
		budget.take(rho, u_eq);
		ref_T.collide(u_eq, w_T); ref_Q.collide(u_eq, w_T);
		if(report&&step%100ull==0ull) std::printf("\n  step %llu: the largest |GPU - host| T %.2e, Q %.2e, flux %.2e; the qt drift GPU %.2e, host %.2e kg/kg\n",
		                                          (unsigned long long)step, result.T.largest, result.Q.largest, result.flux.largest, result.gpu_drift, result.host_drift);
	}
	result.fit7 = budget.fit(ref_Q, Q_start, 7u);
	result.fit19 = budget.fit(ref_Q, Q_start, 19u);
	const DivergenceFit rho19 = budget.fit_density(rho, 19u), rho7 = budget.fit_density(rho, 7u);
	if(report) std::printf("\nthe budget's validation: the GPU's rho - 1 against -Sum div_19(rho u_eq): slope %.3f, correlation %.3f; against -Sum div_7(u_eq): slope %.3f, correlation %.3f\n", rho19.slope, rho19.correlation, rho7.slope, rho7.correlation);
	Memory<float>& ring = lbm.debug_trace();
	ring.read_from_device();
	uint newest = 0u; // the record of the last step: the largest step number in the ring
	for(uint r=1u; r<DebugProbe::trace_steps; r++) if(ring[(ulong)r*DebugProbe::trace_channels+DebugProbe::trace_step]>ring[(ulong)newest*DebugProbe::trace_channels+DebugProbe::trace_step]) newest = r;
	const ulong base = (ulong)newest*DebugProbe::trace_channels;
	result.traced_step = ring[base+DebugProbe::trace_step];
	for(uint i=0u; i<7u; i++) { result.gpu_h_pre[i] = ring[base+DebugProbe::trace_h_pre+i]; result.gpu_g_pre[i] = ring[base+DebugProbe::trace_g_pre+i]; }
	return result;
}

/// @brief Prints where a difference was first beyond the tolerance and where it was largest.
void print_difference(const char* what, const Difference& d, const uint Nx, const uint Ny) {
	const auto xyz = [&](const ulong n) { return uint3((uint)(n%Nx), (uint)((n/Nx)%Ny), (uint)(n/((ulong)Nx*Ny))); };
	const uint3 l = xyz(d.largest_cell);
	if(d.first_step==0ull) { std::printf("%s: within the tolerance on every step (largest %.2e at (%u, %u, %u))\n", what, d.largest, l.x, l.y, l.z); return; }
	const uint3 f = xyz(d.first_cell);
	std::printf("%s: first beyond the tolerance at step %llu, cell (%u, %u, %u): GPU %.7f, host %.7f; largest %.2e at (%u, %u, %u)\n",
	            what, (unsigned long long)d.first_step, f.x, f.y, f.z, d.first_gpu, d.first_host, d.largest, l.x, l.y, l.z);
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID LEVEL_FORCING MOISTURE DEBUG_PROBE (the CBL configuration and the probe)
	// The code check only: the host reference with the Esoteric-Pull walls' 2-step bounce-back. The
	// scheme variants (the 1-step bounce-back, the D3Q19-consistent velocity, the advective-form source, wT 1, the
	// two-step velocity mean, the Chopard correction) were measured once; their code stays
	// in scalar_reference_lattice.hpp (compare()'s variant argument) and is not rerun here.
	const ThermalBox box;
	const float tolerance = 1.0E-5f; // lattice units; float rounding of values about 3.6 stays far below; the qt drift is 1.6e-2
	const ulong steps = 1400ull;
	const uint delay = 2u;
	const uint3 top = uint3(box.N/2u, box.N/2u, (uint)(box.centre_z+0.7f*box.radius)); // on the axis near the thermal's top, where the qt drift grows
	std::printf("\nscalar reference: %llu steps, tolerance %.0e (lattice units), the host's bounce-back after %u steps\n", (unsigned long long)steps, tolerance, delay);
	const Comparison run = compare(box, steps, top, tolerance, true, delay, 0u);
	std::printf("\n");
	print_difference("T", run.T, box.N, box.N);
	print_difference("Q", run.Q, box.N, box.N);
	print_difference("the moisture flux", run.flux, box.N, box.N);
	std::printf("the qt drift at step %llu: GPU %.3e kg/kg, host reference %.3e kg/kg\n", (unsigned long long)steps, run.gpu_drift, run.host_drift);
	if(run.Q.first_step>0ull) { // a mismatch: trace the first mismatching cell up to the first mismatching step
		const ulong n = run.Q.first_cell;
		const uint3 cell = uint3((uint)(n%box.N), (uint)((n/box.N)%box.N), (uint)(n/((ulong)box.N*box.N)));
		std::printf("\nscalar reference: the cell (%u, %u, %u) traced to step %llu\n", cell.x, cell.y, cell.z, (unsigned long long)run.Q.first_step);
		const Comparison at = compare(box, run.Q.first_step, cell, tolerance, false, delay, 0u);
		std::printf("the trace record holds step %.0f; the populations before the collision (D3Q7: 0, +x, -x, +y, -y, +z, -z):\n", at.traced_step);
		for(uint i=0u; i<7u; i++) std::printf("  h_%u: GPU %+.9f host %+.9f difference %+.3e   g_%u: GPU %+.9f host %+.9f difference %+.3e\n",
		                                      i, at.gpu_h_pre[i], at.host_h_pre[i], at.gpu_h_pre[i]-at.host_h_pre[i], i, at.gpu_g_pre[i], at.host_g_pre[i], at.gpu_g_pre[i]-at.host_g_pre[i]);
	}
	PhysicsCheck::report("scalar reference: the host reproduces the GPU's qt drift (relative)", std::fabs(run.host_drift-run.gpu_drift)/run.gpu_drift, 0.01);
	PhysicsCheck::report("scalar reference: the kernel's T against the host's (lattice units over 1e-5)", run.T.largest/(double)tolerance, 1.0);
	PhysicsCheck::report("scalar reference: the kernel's Q against the host's (lattice units over 1e-5)", run.Q.largest/(double)tolerance, 1.0);
	PhysicsCheck::report("scalar reference: the kernel's moisture flux against the host's (lattice units over 1e-5)", run.flux.largest/(double)tolerance, 1.0);
}
