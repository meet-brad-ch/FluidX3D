// Rayleigh-Benard convection: the Nusselt number against the cell count (the grid
// dependence). The test rayleigh_benard measured Nu 25 % (Ra 1e5) and 38 % (Ra 1e6) above Kerr 1996 at H = 64, where the
// thermal boundary layer is H / (2 Nu) = 3 to 6 cells thick and the fixed-temperature plane (the TYPE_T cell centre at
// z = 1) lies half a cell inside the no-slip wall (z = 0.5). Both errors shrink with the cell count; a model defect does
// not. So: Ra 1e5 at H = 32, 64, 128 and Ra 1e6 at H = 64, 128 (nu fixed at 0.005, the run length about 100 free-fall
// times or more), the observed order of convergence p from the three Ra 1e5 grids (log2 of the ratio of successive
// differences; the first run measured 1.19: first order, the wall treatment) and the Richardson extrapolation
// Nu(h -> 0) = Nu_fine + (Nu_fine - Nu_coarse) / (2^p - 1) at that order, also for Ra 1e6 (two grids: the order is
// taken from 1e5 and said so). Gates: the extrapolated Nu within 5 % of Kerr 1996 at both Ra (the known answer; the
// MicroHH references in the same box: 4.78 +- 0.37 at 1e5, 8.67 +- 0.76 at 1e6); the finest grid's own error is reported.
// Writes bin/rayleigh_benard_grid_<Ra>_h<H>_nu.csv and _section.csv for charts.

#include "rayleigh_benard_case.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

/// One grid of one Rayleigh number and what it measured.
struct GridPoint {
	uint H = 0u;
	ulong steps = 0ull;
	rayleigh_benard::RayleighResult result;
};

/// Runs the grids of one Ra in order of H and returns them with their results.
std::vector<GridPoint> run_grids(const double Ra, std::vector<GridPoint> grids) {
	for(GridPoint& g : grids) {
		const std::string prefix = "rayleigh_benard_grid_"+std::to_string((long long)Ra)+"_h"+std::to_string(g.H);
		g.result = rayleigh_benard::run_case({ Ra, g.H, 2.016, g.steps, 0.0 }, prefix);
	}
	return grids;
}

/// The observed order of convergence from three grids (0 when the differences do not shrink monotonically).
double observed_order(const std::vector<GridPoint>& grids) {
	const double e1 = grids[1].result.nusselt-grids[0].result.nusselt, e2 = grids[2].result.nusselt-grids[1].result.nusselt;
	return e1*e2>0.0 ? std::log2(e1/e2) : 0.0;
}

/// The Richardson extrapolation of the two finest grids at the order p.
double extrapolated(const std::vector<GridPoint>& grids, const double p) {
	const double fine = grids.back().result.nusselt, coarse = grids[grids.size()-2u].result.nusselt;
	return fine+(fine-coarse)/(std::pow(2.0, p)-1.0);
}

void report(const double Ra, const std::vector<GridPoint>& grids, const double p, const char* order_source) {
	const double reference = rayleigh_benard::kerr_1996(Ra);
	std::printf("\nRa %.0e grids:", Ra);
	for(const GridPoint& g : grids) std::printf(" H %u: Nu %.3f +- %.3f (%+.1f %%);", g.H, g.result.nusselt, g.result.spread, 100.0*(g.result.nusselt/reference-1.0));
	const double Nu0 = extrapolated(grids, p);
	std::printf("\n  Kerr 1996 %.3f; extrapolated Nu(h -> 0) at order %.2f (%s): %.3f (%+.1f %%)\n", reference, p, order_source, Nu0, 100.0*(Nu0/reference-1.0));
	const std::string label = Ra<5.0E5 ? "1e5" : "1e6";
	PhysicsCheck::report(("Rayleigh-Benard grid: extrapolated Nu at Ra "+label+" against Kerr 1996").c_str(), std::fabs(Nu0/reference-1.0), 0.05);
	std::printf("  (finest grid H %u on its own: %.1f %% from Kerr 1996)\n", grids.back().H, 100.0*std::fabs(grids.back().result.nusselt/reference-1.0));
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX
	// the steps per grid give at least 100 free-fall times H / sqrt(g beta dT H) (nu fixed, so t_ff grows as H^2)
	const std::vector<GridPoint> grids5 = run_grids(1.0E5, { { 32u, 150000ull }, { 64u, 600000ull }, { 128u, 900000ull } });
	const double p = observed_order(grids5);
	report(1.0E5, grids5, p, "observed at Ra 1e5");
	report(1.0E6, run_grids(1.0E6, { { 64u, 300000ull }, { 128u, 300000ull } }), p, "the order observed at Ra 1e5");
}
