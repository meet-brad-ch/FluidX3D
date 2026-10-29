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

/// @brief Every cell's velocity in m/s after the same run, in index order, and the grid's cells along X.
struct Velocities {
	std::vector<float> u{}; ///< u, v, w of each cell in m/s, cell after cell
	std::uint32_t nx{ 0u }; ///< the grid's cells along X
};

/// @brief Runs the same wind over a block in one lattice and in a lattice of two domains, and compares the velocities.
class FlowComparison {
public:
	/// @brief Runs wind from the west over a block on the floor for a fixed time in a lattice of domains along X.
	/// @param domains the domains along X
	/// @return every cell's velocity
	static Velocities run(std::uint32_t domains) {
		using namespace si::symbols;
		const si::Length length = 4.0 * m;
		const si::Length width = 1.0 * m;
		const si::Length height = 1.0 * m;
		Domain domain = Domain::box(length, width, height);
		const si::Length cell = 0.05 * m;
		domain.cell_size(cell).gpus(domains, 1u, 1u);
		const si::Speed inflow = inflow_speed();
		Simulation sim(domain, Fluid::AIR, inflow);
		const si::Position block_from{ 1.0 * m, 0.3 * m, 0.0 * m };
		const si::Position block_to{ 1.4 * m, 0.7 * m, 0.4 * m };
		const si::Length reference_height = 0.5 * m;
		const double wind_exponent{ 0.12 };
		sim.boundaries()
			.set_solid_floor()
			.set_open_boundaries()
			.set_wind_profile_power_law(inflow, reference_height, wind_exponent)
			.set_wind_direction(Face::X_MIN)
			.add_solid(Shape::box(block_from, block_to)) // a block on the floor, west of the split
			.apply();
		const si::Duration run_time = 1.0 * s;
		sim.run_for(run_time);
		const Velocities out = velocities_of(sim);
		return out;
	}

	/// @brief Gives the largest difference of a velocity component between two runs of the same grid.
	/// @param a one run
	/// @param b the other run, with as many values as a
	/// @return the difference in m/s
	static double largest_difference(const Velocities& a, const Velocities& b) {
		double largest{ 0.0 };
		const std::size_t values{ a.u.size() };
		for(std::size_t i{ 0u }; i < values; i++) {
			const float difference{ std::fabs(a.u[i] - b.u[i]) };
			largest = std::max(largest, static_cast<double>(difference));
		}
		return largest;
	}

	/// @brief Gives the largest vertical velocity in the east half: the flow reaches it when it is not zero.
	/// @param run the velocities of a run in two domains
	/// @return the speed in m/s
	static double east_wake(const Velocities& run) {
		double wake{ 0.0 };
		const std::size_t values{ run.u.size() };
		const std::uint32_t east_from{ run.nx / 2u };
		for(std::size_t i{ 2u }; i < values; i += 3u) {
			const std::size_t cell{ i / 3u };
			const std::size_t x{ cell % run.nx };
			if(x >= east_from) {
				const float w{ std::fabs(run.u[i]) };
				wake = std::max(wake, static_cast<double>(w));
			}
		}
		return wake;
	}

	/// @brief Gives the wind's reference speed.
	/// @return 5 m/s
	static si::Speed inflow_speed() {
		using namespace si::symbols;
		const si::Speed inflow = 5.0 * m / s;
		return inflow;
	}

private:
	/// @brief Reads every cell's velocity.
	/// @param sim a simulation after its run
	/// @return the velocities in index order
	static Velocities velocities_of(Simulation& sim) {
		using namespace si::symbols;
		const FieldReader fields = sim.fields();
		Velocities out{};
		out.nx = sim.lbm().get_Nx();
		const std::uint64_t count{ fields.cell_count() };
		for(std::uint64_t n{ 0u }; n < count; ++n) {
			const FieldReader::Cell c{ fields.cell(n) };
			const float u = static_cast<float>(c.velocity.x.in(m / s));
			const float v = static_cast<float>(c.velocity.y.in(m / s));
			const float w = static_cast<float>(c.velocity.z.in(m / s));
			out.u.push_back(u);
			out.u.push_back(v);
			out.u.push_back(w);
		}
		return out;
	}
};

} // namespace

void main_setup() { // extensions: FP16S EQUILIBRIUM_BOUNDARIES SUBGRID, as a terrain wind solver
	using namespace si::symbols;
	const Velocities whole = FlowComparison::run(1u);
	const Velocities split = FlowComparison::run(2u);
	double largest{ 0.0 };
	double wake_east{ 0.0 };
	const bool same_grid{ whole.u.size() == split.u.size() && whole.nx == split.nx };
	if(same_grid) {
		largest = FlowComparison::largest_difference(whole, split);
		wake_east = FlowComparison::east_wake(split);
	}
	const std::size_t cells{ whole.u.size() / 3u };
	std::printf("\ncells %zu, largest velocity difference %.3g m/s, largest vertical velocity in the east half %.3g m/s\n",
	            cells, largest, wake_east);
	const double inflow_mps{ FlowComparison::inflow_speed().in(m / s) };
	double error{ 1.0 }; // a different grid or no flow in the east half fails
	if(same_grid && wake_east > 0.0) {
		error = largest / inflow_mps;
	}
	const double error_limit{ 1E-5 }; // float precision of the inflow speed
	PhysicsCheck::report("a flow in two domains against one", error, error_limit);
}
