// SCALAR_FV across domains (this fork): a run split into domains is the run in one domain, to the last bit.
//
// The finite volumes read every cell's neighbours up to 2 cells along each D3Q19 link (diagonals too: the face value
// takes the cell behind and the cell 2 ahead), and both cells of a face compute it from the same 4 values. A domain
// cut therefore needs 2 halo layers of theta and q (and their half-step values), the diffusivity and the flags
// (2 layers: the wall test reads 2 cells along a link), and the link fluxes from the exchanged DDFs. With every halo
// value equal to the cell it mirrors, each cell computes the same float operations from the same inputs as in one
// domain, so the fields must be bit-identical.
//
// The case: a periodic box (32 x 32 cells, 24 high) between a solid floor and lid, a warm moist blob that sits across
// the x and the y cut (centre at x 16, y 16), a diagonal start flow (0.04, 0.03, 0), buoyancy through VOLUME_FORCE and
// the thermal expansion, a heat and moisture source in a patch of the floor, SUBGRID (the diffusivity varies, so its
// exchange is tested), 400 steps. Splits: 2 along x, 2 along y, 2 x 2 (the corner and edge cells of the diagonal links).
// Compared in every cell: T, Q, u. Gate: the largest absolute difference is 0.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

/// @brief The fields of one run, read back through the host's stitched view of the domains.
struct DomainFields {
	std::vector<float> T{}; ///< 1 + theta
	std::vector<float> Q{}; ///< 1 + q / 0.01
	std::vector<float> u{}; ///< the velocity, x then y then z components
};

/// @brief The case, run in one domain and split into domains, and the comparison of its fields.
class SplitRun {
public:
	/// @brief Runs the case split into Dx x Dy domains and reads the fields back.
	/// @param Dx domains along x
	/// @param Dy domains along y
	/// @return the fields after the steps
	static DomainFields run(uint Dx, uint Dy) {
		LBM lbm(box_cells, box_cells, box_levels, Dx, Dy, 1u, viscosity, 0.0f, 0.0f, fz, 0.0f, alpha, beta);
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			start_cell(lbm, n);
		}
		lbm.set_heat_rate(source_rate);
		lbm.set_moisture_rate(source_rate);
		lbm.set_inv_prandtl_t(inverse_prandtl);
		lbm.run(steps);
		lbm.update_fields();
		lbm.T.read_from_device();
		lbm.Q.read_from_device();
		lbm.u.read_from_device();
		DomainFields f{};
		f.T.resize(cells);
		f.Q.resize(cells);
		f.u.resize(3ull * cells);
		for(ulong n{ 0ull }; n < cells; n++) {
			f.T[n] = lbm.T[n];
			f.Q[n] = lbm.Q[n];
			f.u[n] = lbm.u.x[n];
			f.u[cells + n] = lbm.u.y[n];
			f.u[2ull * cells + n] = lbm.u.z[n];
		}
		return f;
	}

	/// @brief Gives the largest absolute difference between two fields.
	/// @param a the first field
	/// @param b the second field, of the same length
	/// @return max |a - b|
	static float largest_difference(const std::vector<float>& a, const std::vector<float>& b) {
		float largest{ 0.0f };
		const std::size_t values{ a.size() };
		for(std::size_t n{ 0u }; n < values; n++) {
			largest = std::fmax(largest, std::fabs(a[n] - b[n]));
		}
		return largest;
	}

	static constexpr ulong steps = 400ull; ///< the steps of each run

private:
	/// @brief Writes the start of one cell: the blob in T and Q, the walls, the start flow and the floor's source.
	/// @param lbm the lattice
	/// @param n the cell's index
	static void start_cell(LBM& lbm, ulong n) {
		uint x{ 0u };
		uint y{ 0u };
		uint z{ 0u };
		lbm.coordinates(n, x, y, z);
		const float dx{ static_cast<float>(x) - blob_xy };
		const float dy{ static_cast<float>(y) - blob_xy };
		const float dz{ static_cast<float>(z) - blob_z };
		const float r2{ sq(dx) + sq(dy) + sq(dz) };
		const float blob{ r2 < blob_radius_squared ? 1.0f - r2 / blob_radius_squared : 0.0f }; // a smooth blob of radius 5 across the x and the y cut
		lbm.T[n] = 1.0f + blob_theta * blob;
		lbm.Q[n] = 1.0f + background_q + blob_q * blob;
		if(z == 0u || z == box_levels - 1u) {
			lbm.flags[n] = TYPE_S;
			return;
		}
		lbm.u.x[n] = flow_x;
		lbm.u.y[n] = flow_y;
		const bool in_patch_x{ x >= patch_from && x < patch_to };
		const bool in_patch_y{ y >= patch_from && y < patch_to };
		if(z == 1u && in_patch_x && in_patch_y) { // the floor's source under the blob, across both cuts
			lbm.flags[n] = TYPE_Y;
		}
	}

	static constexpr uint box_cells = 32u;            ///< the box's cells along x and y
	static constexpr uint box_levels = 24u;           ///< the box's cells along z, the floor and the lid solid
	static constexpr float viscosity = 1.0E-3f;       ///< the kinematic viscosity in lattice units
	static constexpr float fz = -1.0E-6f;             ///< the volume force along z in lattice units
	static constexpr float alpha = 0.014f;            ///< the thermal diffusion coefficient in lattice units
	static constexpr float beta = 50.0f;              ///< the thermal expansion in lattice units
	static constexpr float blob_xy = 16.0f;           ///< the blob's centre along x and y in cells, on both cuts
	static constexpr float blob_z = 8.0f;             ///< the blob's centre along z in cells
	static constexpr float blob_radius_squared = 25.0f; ///< the blob's squared radius in cells^2 (radius 5)
	static constexpr float blob_theta = 0.5f;         ///< the blob's peak theta in lattice temperature
	static constexpr float background_q = 0.8f;       ///< the water everywhere, Q - 1
	static constexpr float blob_q = 0.5f;             ///< the blob's extra peak Q
	static constexpr float flow_x = 0.04f;            ///< the start flow along x in lattice units
	static constexpr float flow_y = 0.03f;            ///< the start flow along y in lattice units
	static constexpr uint patch_from = 12u;           ///< the source patch's first cell along x and y
	static constexpr uint patch_to = 20u;             ///< one past the source patch's last cell along x and y
	static constexpr float source_rate = 1.0E-3f;     ///< the heat and the moisture added per step to each source cell
	static constexpr float inverse_prandtl = 3.0f;    ///< 1 / Pr_t
};

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID MOISTURE SCALAR_FV
	const DomainFields one = SplitRun::run(1u, 1u);
	float t_peak{ 0.0f };
	for(const float t : one.T) {
		t_peak = std::fmax(t_peak, t - 1.0f);
	}
	const unsigned long long printed_steps{ static_cast<unsigned long long>(SplitRun::steps) };
	std::printf("\none domain after %llu steps: the largest theta %.4f (the source keeps heating; the blob moved across the cuts)\n", printed_steps, t_peak);
	const uint splits[3][2] = { { 2u, 1u }, { 1u, 2u }, { 2u, 2u } };
	for(const auto& s : splits) {
		const DomainFields d = SplitRun::run(s[0], s[1]);
		const float dT{ SplitRun::largest_difference(one.T, d.T) };
		const float dQ{ SplitRun::largest_difference(one.Q, d.Q) };
		const float du{ SplitRun::largest_difference(one.u, d.u) };
		std::printf("%u x %u domains: the largest difference to one domain T %.3e, Q %.3e, u %.3e\n", s[0], s[1], dT, dQ, du);
		const string name = "finite volumes: " + to_string(s[0]) + " x " + to_string(s[1]) + " domains against one (bit-identical: T, Q, u; any difference fails)";
		const bool any_difference{ dT > 0.0f || dQ > 0.0f || du > 0.0f };
		const double error{ any_difference ? 1.0 : 0.0 };
		const double limit{ 0.5 }; // a 0/1 verdict: 1 fails, 0 passes
		PhysicsCheck::report(name.c_str(), error, limit);
	}
}
