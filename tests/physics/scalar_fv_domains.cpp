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

/// The fields of one run, read back through the host's stitched view of the domains.
struct DomainFields {
	std::vector<float> T;  ///< 1 + theta
	std::vector<float> Q;  ///< 1 + q / 0.01
	std::vector<float> u;  ///< the velocity, x then y then z components
};

/// @brief Runs the case split into Dx x Dy domains and reads the fields back.
/// @param Dx domains along x
/// @param Dy domains along y
/// @param steps the steps to run
/// @return the fields after the steps
DomainFields run_split(const uint Dx, const uint Dy, const ulong steps) {
	const uint N = 32u, Nz = 24u;
	LBM lbm(N, N, Nz, Dx, Dy, 1u, 1.0E-3f, 0.0f, 0.0f, -1.0E-6f, 0.0f, 0.014f, 50.0f);
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		const float r2 = sq((float)x-16.0f)+sq((float)y-16.0f)+sq((float)z-8.0f);
		const float blob = r2<25.0f ? 1.0f-r2/25.0f : 0.0f; // a smooth blob of radius 5 across the x and the y cut
		lbm.T[n] = 1.0f+0.5f*blob;
		lbm.Q[n] = 1.0f+0.8f+0.5f*blob;
		if(z==0u||z==Nz-1u) { lbm.flags[n] = TYPE_S; continue; }
		lbm.u.x[n] = 0.04f; lbm.u.y[n] = 0.03f;
		if(z==1u&&x>=12u&&x<20u&&y>=12u&&y<20u) lbm.flags[n] = TYPE_Y; // the floor's source under the blob, across both cuts
	}
	lbm.set_heat_rate(1.0E-3f);
	lbm.set_moisture_rate(1.0E-3f);
	lbm.set_inv_prandtl_t(3.0f);
	lbm.run(steps);
	lbm.update_fields();
	lbm.T.read_from_device();
	lbm.Q.read_from_device();
	lbm.u.read_from_device();
	DomainFields f;
	f.T.resize(lbm.get_N()); f.Q.resize(lbm.get_N()); f.u.resize(3ull*lbm.get_N());
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		f.T[n] = lbm.T[n]; f.Q[n] = lbm.Q[n];
		f.u[n] = lbm.u.x[n]; f.u[lbm.get_N()+n] = lbm.u.y[n]; f.u[2ull*lbm.get_N()+n] = lbm.u.z[n];
	}
	return f;
}

/// @brief The largest absolute difference between two fields.
/// @param a the first field
/// @param b the second field, of the same length
/// @return max |a - b|
float largest_difference(const std::vector<float>& a, const std::vector<float>& b) {
	float largest = 0.0f;
	for(size_t n=0; n<a.size(); n++) largest = std::fmax(largest, std::fabs(a[n]-b[n]));
	return largest;
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID MOISTURE SCALAR_FV
	const ulong steps = 400ull;
	const DomainFields one = run_split(1u, 1u, steps);
	float t_peak = 0.0f;
	for(const float t : one.T) t_peak = std::fmax(t_peak, t-1.0f);
	std::printf("\none domain after %llu steps: the largest theta %.4f (the source keeps heating; the blob moved across the cuts)\n", (unsigned long long)steps, t_peak);
	const uint splits[3][2] = { { 2u, 1u }, { 1u, 2u }, { 2u, 2u } };
	for(const auto& s : splits) {
		const DomainFields d = run_split(s[0], s[1], steps);
		const float dT = largest_difference(one.T, d.T), dQ = largest_difference(one.Q, d.Q), du = largest_difference(one.u, d.u);
		std::printf("%u x %u domains: the largest difference to one domain T %.3e, Q %.3e, u %.3e\n", s[0], s[1], dT, dQ, du);
		const string name = "finite volumes: "+to_string(s[0])+" x "+to_string(s[1])+" domains against one (bit-identical: T, Q, u; any difference fails)";
		PhysicsCheck::report(name.c_str(), dT>0.0f||dQ>0.0f||du>0.0f ? 1.0 : 0.0, 0.5);
	}
}
