// A flow in a lattice split into domains: wind over a block on the floor, the same run as one domain and as two along X,
// every cell's velocity read back through FieldReader (the path a recording takes) and compared. The domains exchange
// the DDFs a single lattice streams within itself, so the two runs agree to float precision.
// Passes when the largest velocity difference is under 1E-5 of the inflow speed, with the block's wake in both halves.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

/// Every cell's velocity in m/s after the same run, in index order, and the grid's cells along X.
struct Velocities {
	std::vector<float> u;
	std::uint32_t nx = 0u;
};

/// Wind from the west over a block on the floor, run for a fixed time in a lattice of `domains` along X.
Velocities run(std::uint32_t domains) {
	const Speed inflow = Speed::from_si(5.0f);
	Simulation sim(Domain::box(4.0_m, 1.0_m, 1.0_m).cell_size(0.05_m).gpus(domains, 1u, 1u), Fluid::AIR, inflow);
	sim.boundaries()
		.set_solid_floor()
		.set_open_boundaries()
		.set_wind_profile_power_law(inflow, 0.5_m, 0.12f)
		.set_wind_direction(Face::X_MIN)
		.add_solid(Shape::box({ 1.0_m, 0.3_m, 0.0_m }, { 1.4_m, 0.7_m, 0.4_m })) // a block on the floor, west of the split
		.apply();
	sim.run_for(Duration::from_si(1.0f));
	const FieldReader fields = sim.fields();
	Velocities out;
	out.nx = sim.lbm().get_Nx();
	fields.for_each_cell([&](const FieldReader::Cell& c) {
		out.u.push_back(c.velocity.x.si());
		out.u.push_back(c.velocity.y.si());
		out.u.push_back(c.velocity.z.si());
	});
	return out;
}

} // namespace

void main_setup() { // extensions: FP16S EQUILIBRIUM_BOUNDARIES SUBGRID, as the wind solver (tools/cfd in para-sim)
	const Velocities whole = run(1u);
	const Velocities split = run(2u);
	double largest = 0.0, wake_east = 0.0;
	const bool same_grid = whole.u.size() == split.u.size() && whole.nx == split.nx;
	if(same_grid) {
		for(std::size_t i = 0u; i < whole.u.size(); i++) largest = std::max(largest, (double)std::fabs(whole.u[i] - split.u[i]));
		// the flow reaches the east half: its vertical velocities are not all zero
		for(std::size_t i = 2u; i < split.u.size(); i += 3u) {
			const std::size_t cell = i / 3u;
			if(cell % split.nx >= split.nx / 2u) wake_east = std::max(wake_east, (double)std::fabs(split.u[i]));
		}
	}
	std::printf("\ncells %zu, largest velocity difference %.3g m/s, largest vertical velocity in the east half %.3g m/s\n",
	            whole.u.size() / 3u, largest, wake_east);
	const double error = (!same_grid || wake_east <= 0.0) ? 1.0 : largest / 5.0;
	PhysicsCheck::report("a flow in two domains against one", error, 1E-5);
}
