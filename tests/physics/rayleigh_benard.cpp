// Rayleigh-Benard convection of air (Pr = 0.71) between a hot floor and a cold ceiling, periodic sideways: the onset of
// convection and the Nusselt number against the known answers.
// - Onset: for rigid plates the critical Rayleigh number is 1708 (Chandrasekhar 1961) at the wavelength 2.016 H; the box
//   is 2.016 H wide so that mode fits. At Ra = 1600 a small perturbation must decay (Nu stays 1), at Ra = 1900 it must
//   grow (Nu > 1): the bracket of 10 % around 1708.
// - Nusselt number at Ra = 1e4, 1e5, 1e6: Nu = 1 + <w (T - T_mean)> H / (D dT) over the fluid, averaged over the second half
//   of the run, against Kerr 1996 (J. Fluid Mech. 310, Pr 0.7, 5e4 <= Ra <= 2e7): Nu = 0.186 Ra^0.276; the limit 5 % at
//   1e5 and 1e6, the value at 1e4 (below the fit's range, steady rolls) reported only.
// The case setup and the Nu measurement: rayleigh_benard_case.hpp. Writes bin/rayleigh_benard_<Ra>_nu.csv and
// _section.csv for charts.

#include "rayleigh_benard_case.hpp"
#include "physics_check.hpp"

#include <cstdio>
#include <string>

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX
	using namespace rayleigh_benard;
	const auto prefix = [](const double Ra) { return "rayleigh_benard_"+std::to_string((long long)Ra); };
	// the onset bracket: H = 32, a 1e-3 perturbation, 1.5 million steps (about 10 diffusive times H^2/alpha)
	const RayleighResult below = run_case({ 1600.0, 32u, 2.016, 1500000ull, 1.0E-3 }, prefix(1600.0));
	const RayleighResult above = run_case({ 1900.0, 32u, 2.016, 1500000ull, 1.0E-3 }, prefix(1900.0));
	std::printf("\nonset: Nu(1600) %.4f, Nu(1900) %.4f (critical 1708: the first must stay at 1, the second rise above it)\n", below.nusselt, above.nusselt);
	const bool onset_ok = below.nusselt<1.01&&above.nusselt>1.05;
	PhysicsCheck::report("Rayleigh-Benard onset within 10 % of Ra_c = 1708", onset_ok ? 0.0 : 1.0, 0.5);

	// the Nusselt numbers: H = 64 (1e5, 1e6) or 32 (1e4), the box 2.016 H
	const RayleighResult r4 = run_case({ 1.0E4, 32u, 2.016, 400000ull, 0.0 }, prefix(1.0E4));
	std::printf("\nRa 1e4: Nu %.3f +- %.3f (Kerr 1996 fit, below its range: %.3f; reported only)\n", r4.nusselt, r4.spread, kerr_1996(1.0E4));
	const RayleighResult r5 = run_case({ 1.0E5, 64u, 2.016, 600000ull, 0.0 }, prefix(1.0E5));
	std::printf("\nRa 1e5: Nu %.3f +- %.3f, Kerr 1996 %.3f\n", r5.nusselt, r5.spread, kerr_1996(1.0E5));
	PhysicsCheck::report("Rayleigh-Benard Nu at Ra 1e5 against Kerr 1996", std::fabs(r5.nusselt/kerr_1996(1.0E5)-1.0), 0.05);
	const RayleighResult r6 = run_case({ 1.0E6, 64u, 2.016, 300000ull, 0.0 }, prefix(1.0E6));
	std::printf("\nRa 1e6: Nu %.3f +- %.3f, Kerr 1996 %.3f\n", r6.nusselt, r6.spread, kerr_1996(1.0E6));
	PhysicsCheck::report("Rayleigh-Benard Nu at Ra 1e6 against Kerr 1996", std::fabs(r6.nusselt/kerr_1996(1.0E6)-1.0), 0.05);
}
