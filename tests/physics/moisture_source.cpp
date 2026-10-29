// MOISTURE: the second scalar lattice. Total water qt rides on a D3Q7
// lattice hi, the twin of the thermal gi, as Q = 1 + qt / def_q_scale, with the thermal relaxation (the same molecular and
// eddy diffusivity) and a surface source in TYPE_Y cells (LBM::set_moisture_rate(), per step and cell, as the heat rate).
// 1. The budget: a periodic box of fluid at rest whose layer z = 0 is TYPE_Y, moistened by moisture_rate; without walls
//    every bit of added water stays in the fluid cells: the mean Q rises by moisture_rate x (source cells) / (cells) per
//    step while diffusion carries it away from the layer. Passes when the mean rise from step 1000 to 2000 is within
//    0.1 % of that, and the layer farthest from the source has moistened.
// 2. The domains: the same box with a moist blob and a flow, one domain against two along x, after 500 steps the Q
//    fields equal to the last bit (the halo exchange of hi; with SCALAR_FV the finite volumes' 2-layer exchange).

// On the core directly, not the atmosphere API: the test measures the core's own moisture source and its halo exchange in
// lattice units, in a periodic box without walls (the atmosphere API always has a solid floor and lid).

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

/// @brief The moisture lattice's two checks: the water budget of a source layer, and one domain against two.
class MoistureChecks {
public:
	/// @brief A periodic box at rest whose layer z = 0 is TYPE_Y, moistened by the moisture rate: checks the added
	/// water against the rate and that the water reached the farthest layer.
	static void budget() {
		LBM lbm(box_cells, box_cells, box_cells, viscosity, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f);
		const ulong sources{ flag_sources(lbm) };
		lbm.set_moisture_rate(moisture_rate);
		lbm.run(budget_steps);
		const double rise1{ mean_rise(lbm) };
		lbm.run(budget_steps);
		const double rise2{ mean_rise(lbm) };
		const double middle{ middle_layer_rise(lbm) };
		const double cells{ static_cast<double>(lbm.get_N()) };
		const double expected{ static_cast<double>(moisture_rate) * static_cast<double>(sources) * static_cast<double>(budget_steps) / cells };
		const unsigned long long printed_steps{ static_cast<unsigned long long>(budget_steps) };
		const unsigned long long printed_both{ 2ull * printed_steps };
		std::printf("\nmean Q rise %.6e after %llu steps, %.6e after %llu (expected per %llu steps %.6e), the farthest layer %.6e\n",
		            rise1, printed_steps, rise2, printed_both, printed_steps, expected, middle);
		const double water_error{ std::fabs((rise2 - rise1) / expected - 1.0) };
		PhysicsCheck::report("moisture source: the added water", water_error, water_limit);
		const double reach_error{ middle > 0.0 ? 0.0 : 1.0 };
		PhysicsCheck::report("moisture source: the water reached the farthest layer", reach_error, reach_limit);
	}

	/// @brief The same moist blob in a flow in one domain and in two along x: checks that the Q fields are equal.
	static void domains() {
		const std::vector<float> one = blob_after(1u);
		const std::vector<float> two = blob_after(2u);
		float largest{ 0.0f };
		float peak{ 0.0f };
		const std::size_t cells{ one.size() };
		for(std::size_t n{ 0u }; n < cells; n++) {
			largest = std::fmax(largest, std::fabs(one[n] - two[n]));
			peak = std::fmax(peak, one[n] - 1.0f);
		}
		std::printf("\nmoist blob after 500 steps: the largest difference between one and two domains %.3e (the blob's peak Q - 1 %.4f)\n", largest, peak);
#ifdef SCALAR_FV
		const double any_difference{ largest > 0.0f ? 1.0 : 0.0 };
		PhysicsCheck::report("moisture, finite volumes: two domains against one (bit-identical: absolute, any difference fails)", any_difference, reach_limit);
#else // SCALAR_FV
		const double difference{ static_cast<double>(largest) };
		PhysicsCheck::report("moisture lattice: two domains against one (absolute, limit 1e-6)", difference, domains_limit);
#endif // SCALAR_FV
	}

private:
	/// @brief Sets T and Q to 1 everywhere and flags the layer z = 0 TYPE_Y.
	/// @param lbm the box's lattice
	/// @return the number of source cells
	static ulong flag_sources(LBM& lbm) {
		ulong sources{ 0ull };
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			lbm.T[n] = 1.0f;
			lbm.Q[n] = 1.0f;
			if(z == 0u) {
				lbm.flags[n] = TYPE_Y;
				sources++;
			}
		}
		return sources;
	}

	/// @brief Gives the mean Q rise of all cells.
	/// @param lbm the box's lattice
	/// @return the mean of Q - 1
	static double mean_rise(LBM& lbm) {
		lbm.update_fields();
		lbm.Q.read_from_device();
		double sum{ 0.0 };
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			sum += static_cast<double>(lbm.Q[n]) - 1.0;
		}
		const double mean{ sum / static_cast<double>(cells) };
		return mean;
	}

	/// @brief Gives the summed Q rise of the layer farthest from the source (z = Nz/2), from the last read.
	/// @param lbm the box's lattice
	/// @return the sum of Q - 1 over the layer
	static double middle_layer_rise(LBM& lbm) {
		double middle{ 0.0 };
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			if(z == box_cells / 2u) {
				middle += static_cast<double>(lbm.Q[n]) - 1.0;
			}
		}
		return middle;
	}

	/// @brief Gives the Q field of a box with a moist sphere in a uniform flow after the blob's steps.
	/// @param domains the domains along x
	/// @return Q of every cell in index order
	static std::vector<float> blob_after(uint domains) {
		LBM lbm(blob_cells, blob_cells, blob_cells, domains, 1u, 1u, blob_viscosity, 0.0f, 0.0f, 0.0f, 0.0f, blob_alpha, 0.0f);
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			lbm.T[n] = 1.0f;
			const float dx{ static_cast<float>(x) - blob_x };
			const float dy{ static_cast<float>(y) - blob_yz };
			const float dz{ static_cast<float>(z) - blob_yz };
			const bool in_blob{ sq(dx) + sq(dy) + sq(dz) < blob_radius_squared };
			const float moisture{ in_blob ? blob_excess : 0.0f };
			lbm.Q[n] = 1.0f + moisture;
			lbm.u.x[n] = flow_speed;
		}
		lbm.run(blob_steps);
		lbm.update_fields();
		lbm.Q.read_from_device();
		std::vector<float> q(cells);
		for(ulong n{ 0ull }; n < cells; n++) {
			q[n] = lbm.Q[n];
		}
		return q;
	}

	static constexpr uint box_cells = 16u;            ///< the budget box's cells along each axis
	static constexpr float viscosity = 0.1f;          ///< the budget box's kinematic viscosity in lattice units
	static constexpr float alpha = 0.05f;             ///< the budget box's thermal diffusion coefficient in lattice units
	static constexpr float moisture_rate = 1.0E-3f;   ///< the Q added per step to each source cell
	static constexpr ulong budget_steps = 1000ull;    ///< the steps of each of the two budget runs
	static constexpr double water_limit = 0.001;      ///< the largest relative error of the added water that passes (0.1 %)
	static constexpr double reach_limit = 0.5;        ///< a 0/1 verdict: 1 fails, 0 passes
	static constexpr uint blob_cells = 32u;           ///< the blob box's cells along each axis
	static constexpr float blob_viscosity = 0.02f;    ///< the blob box's kinematic viscosity in lattice units
	static constexpr float blob_alpha = 0.01f;        ///< the blob box's thermal diffusion coefficient in lattice units
	static constexpr float blob_x = 10.0f;            ///< the blob's centre along x in cells, near the x halo
	static constexpr float blob_yz = 16.0f;           ///< the blob's centre along y and z in cells
	static constexpr float blob_radius_squared = 36.0f; ///< the blob's squared radius in cells^2 (radius 6)
	static constexpr float blob_excess = 0.1f;        ///< the blob's Q - 1
	static constexpr float flow_speed = 0.05f;        ///< the uniform flow along x in lattice units
	static constexpr ulong blob_steps = 500ull;       ///< the blob's steps
	static constexpr double domains_limit = 1.0E-6;   ///< the largest absolute Q difference that passes (D3Q7 lattice)
};

} // namespace

void main_setup() { // extensions: TEMPERATURE MOISTURE
	MoistureChecks::budget();
	MoistureChecks::domains();
}
