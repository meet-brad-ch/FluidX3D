// The link-flux finite volumes (LinkFluxFiniteVolumeScalars, scalar_fv_reference.hpp) on the host, in a decaying
// temperature wave T = 1 + 0.01 sin(2 pi s / 64) in a uniform flow u = 0.1 along x, s = x (along the flow) or y
// (across it). The SCALAR_FV build of thermal_diffusivity measured at alpha 2e-3 a diffusivity 7.4 % too large across
// the flow (limit 1 %); the advective finite volumes passed that check. Hypothesis: in a uniform flow the 8 diagonal
// links with a y or z part carry mass both ways (psi = 6 w_i c_ix u = u/6 each, no net cross flow), and the upwind
// MUSCL face values on them diffuse across the flow where the limiter falls back to first order. The lab measures D for
// the exact D3Q19 link fluxes of the uniform flow (the LBM's post-collision populations are the equilibrium there) and
// for the same flow on the x axis links alone (psi_+x = u). The diagonals are the cause if the excess across the flow
// is there with the D3Q19 fluxes and not with the axis fluxes.
// Passes when the D3Q19 fluxes give D within 1 % of alpha across the flow and 5 % along it for alpha >= 0.014 (the tauT
// floor of the thermal tests, the limits of thermal_diffusivity); alpha 2e-3 is printed.

#include "setup/setup.hpp"
#include "physics_check.hpp"
#include "scalar_fv_reference.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

/// How the uniform flow is put on the links.
enum class Links { d3q19, axis };

/// The measured diffusivity of one case.
struct CrossResult {
	double D{};      ///< the measured diffusivity (lattice units)
	double ratio{};  ///< D / alpha
};

/// @brief The wave case: a box of 64 cells along the wave and 4 across, run for the steps of thermal_diffusivity.
class CrossWave {
public:
	/// @param alpha the diffusivity
	/// @param u the flow along x
	/// @param along_x true: the wave varies along x (along the flow), false: along y (across it)
	/// @param links how the flow is put on the links
	CrossWave(const double alpha, const double u, const bool along_x, const Links links)
		: alpha_(alpha), u_(u), along_x_(along_x), links_(links) {}

	/// @brief Runs the wave and measures D from the amplitude of mode k at t1 and t2.
	/// @return the measured diffusivity
	CrossResult run() const {
		const uint Nx{ along_x_ ? L : W }, Ny{ along_x_ ? W : L }, Nz{ W };
		const ulong N{ (ulong)Nx*Ny*Nz };
		std::vector<double> start(N, 0.0);
		for(ulong n{}; n<N; n++) start[n] = A*std::sin(k()*position(n, Nx, Ny));
		LinkFluxFiniteVolumeScalars field(Nx, Ny, Nz, std::vector<bool>(N, false), std::vector<float>(N, 1.0f), std::vector<std::vector<double>>{ start });
		const std::vector<double> psi{ fluxes(N) };
		const std::vector<float> D(N, (float)alpha_);
		const ulong t2{ steps() }, t1{ t2/10ull };
		double A1{};
		for(ulong t{ 1ull }; t<=t2; t++) {
			field.step(psi, D.data());
			if(t==t1) A1 = amplitude(field, N, Nx, Ny);
		}
		const double A2{ amplitude(field, N, Nx, Ny) };
		CrossResult r{};
		r.D = std::log(A1/A2)/(k()*k()*(double)(t2-t1));
		r.ratio = r.D/alpha_;
		return r;
	}

private:
	static constexpr uint L{ 64u }, W{ 4u };
	static constexpr double A{ 0.01 };
	double alpha_{}, u_{};
	bool along_x_{};
	Links links_{};

	static double k() { return 2.0*3.14159265358979/(double)L; }
	/// @brief The steps of thermal_diffusivity: the wave decays by about e between t2/10 and t2, at least 1000.
	ulong steps() const {
		const double s{ 1.0/(alpha_*k()*k()) };
		return s<1000.0 ? 1000ull : (ulong)s;
	}
	/// @brief The coordinate the wave varies along.
	double position(const ulong n, const uint Nx, const uint Ny) const { return along_x_ ? (double)(n%Nx) : (double)((n/Nx)%Ny); }
	/// @brief The link fluxes of the uniform flow, psi_i(n) at (i - 1) N + n.
	std::vector<double> fluxes(const ulong N) const {
		std::vector<double> psi(18ull*N, 0.0);
		for(uint i=1u; i<19u; i++) {
			const int cx{ LinkFluxFiniteVolumeScalars::velocity[i][0] };
			const bool axis{ i<7u };
			double value{};
			if(links_==Links::d3q19) value = 6.0*(axis ? 1.0/18.0 : 1.0/36.0)*(double)cx*u_; // f_i - f_ibar of the equilibrium
			else if(i<3u) value = (double)cx*u_; // the x axis links alone
			for(ulong n{}; n<N; n++) psi[(ulong)(i-1u)*N+n] = value;
		}
		return psi;
	}
	/// @brief The magnitude of the wave's Fourier component.
	double amplitude(const LinkFluxFiniteVolumeScalars& field, const ulong N, const uint Nx, const uint Ny) const {
		double s{}, c{};
		for(ulong n{}; n<N; n++) {
			const double phase{ k()*position(n, Nx, Ny) };
			s += field.at(0ull, n)*std::sin(phase);
			c += field.at(0ull, n)*std::cos(phase);
		}
		return 2.0*std::sqrt(s*s+c*c)/(double)N;
	}
};

} // namespace

void main_setup() { // extensions: TEMPERATURE (the host scheme only; no GPU lattice is built)
	const double alphas[]{ 2.0E-3, 0.014, 0.125 };
	const double u{ 0.1 };
	double worst_across{}, worst_along{};
	std::printf("\nlink-flux lab: a wave T = 1 + 0.01 sin(2 pi s / 64) in a flow u 0.1 along x (D / alpha: 1 is right)\n");
	for(const double alpha : alphas) {
		const CrossResult across19{ CrossWave(alpha, u, false, Links::d3q19).run() }, across_axis{ CrossWave(alpha, u, false, Links::axis).run() };
		const CrossResult along19{ CrossWave(alpha, u, true, Links::d3q19).run() }, along_axis{ CrossWave(alpha, u, true, Links::axis).run() };
		std::printf("  alpha %.3e: across D3Q19 %.4f (excess %.2e), axis %.4f (excess %.2e); along D3Q19 %.4f (excess %.2e), axis %.4f (excess %.2e)\n",
		            alpha, across19.ratio, across19.D-alpha, across_axis.ratio, across_axis.D-alpha, along19.ratio, along19.D-alpha, along_axis.ratio, along_axis.D-alpha);
		if(alpha>=0.014) {
			worst_across = std::fmax(worst_across, std::fabs(across19.ratio-1.0));
			worst_along = std::fmax(worst_along, std::fabs(along19.ratio-1.0));
		}
	}
	PhysicsCheck::report("link-flux lab: D across the flow with the D3Q19 link fluxes, alpha >= 0.014 (relative)", worst_across, 0.01);
	PhysicsCheck::report("link-flux lab: D along the flow with the D3Q19 link fluxes, alpha >= 0.014 (relative)", worst_along, 0.05);
}
