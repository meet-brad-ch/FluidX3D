// A finite-volume scalar on the host: the scheme of ProLB (Feng et al. 2021, JAMES, sect. 3.2, eqs 42-46).
// Advective form d_t phi + u . grad phi = div(D grad phi),
// per direction u_i (phi_i+1/2 - phi_i-1/2) / dx with upwind face values from MUSCL (kappa 1/3) and the van Albada
// limiter, centred diffusion with face diffusivities (D_i + D_i+1)/2, explicit Euler on the LB step or, with use_rk2(), the midpoint step of the SCALAR_FV kernel (Feng et al.
// 2019, eq 31). Lattice units (dx = dt = 1). Solid cells are zero-gradient ghosts: no advective or diffusive flux
// through a wall. Fed with a velocity and a diffusivity per cell and step (from the GPU's cell probe).
#pragma once

#include "setup/setup.hpp"

#include <cmath>
#include <vector>

/// @brief A finite-volume scalar on the host in advective form, MUSCL with the van Albada limiter (ProLB's scheme).
class FiniteVolumeScalar {
public:
	/// @param nx, ny, nz the box (periodic where no solid cell is)
	/// @param solid_cells true for the TYPE_S cells
	/// @param start the start value of every cell
	FiniteVolumeScalar(const uint nx, const uint ny, const uint nz, const std::vector<bool>& solid_cells, const std::vector<float>& start)
		: Nx(nx), Ny(ny), Nz(nz), N((ulong)nx*(ulong)ny*(ulong)nz), solid(solid_cells), phi(start.begin(), start.end()), next(N, 0.0) {
		if(solid.size()!=N||start.size()!=N) print_error("FiniteVolumeScalar: the solid and start vectors need Nx Ny Nz values.");
	}
	/// @brief One step: advection by u, diffusion by D; explicit Euler, or the midpoint step with use_rk2().
	/// @param u ux, uy, uz of every cell (3 N floats, structure of arrays)
	/// @param D the diffusivity of every cell (N floats, lattice units)
	void step(const float* u, const float* D) {
		if(rk2) { // the midpoint rule of Feng et al. 2021 (anelastic LB, eq 36): phi_1/2 = phi + F(phi)/2, phi_1 = phi + F(phi_1/2)
			const std::vector<double> start{ phi };
			euler(u, D);
			for(ulong n{}; n<N; n++) phi[n] = start[n]+0.5*(phi[n]-start[n]); // phi_1/2
			std::vector<double> half{ phi };
			euler(u, D); // phi_1/2 + F(phi_1/2)
			for(ulong n{}; n<N; n++) phi[n] = start[n]+(phi[n]-half[n]); // phi + F(phi_1/2)
			if(float_storage) for(ulong n{}; n<N; n++) phi[n] = (double)(float)phi[n];
			return;
		}
		euler(u, D);
	}
	/// @brief A scheme variant: the second-order midpoint time step instead of explicit Euler.
	void use_rk2(const bool on) { rk2 = on; }
private:
	bool rk2{};
	/// @brief One explicit Euler step of advection and diffusion (eqs 42-46), the z layers on all host threads.
	void euler(const float* u, const float* D) {
		parallel_for(Nz, [&](const uint z) { for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
			const ulong n = index(x, y, z);
			if(solid[n]) { next[n] = phi[n]; continue; }
			double change = 0.0;
			for(uint d=0u; d<3u; d++) {
				const ulong m1 = neighbour(x, y, z, d, -1), m2 = neighbour(x, y, z, d, -2), p1 = neighbour(x, y, z, d, 1), p2 = neighbour(x, y, z, d, 2);
				const double c = phi[n], cm1 = value_at(m1, n), cp1 = value_at(p1, n);
				const double cm2 = solid[m1] ? cm1 : value_at(m2, m1), cp2 = solid[p1] ? cp1 : value_at(p2, p1); // behind a wall the ghost continues
				const double ud = (double)u[(ulong)d*N+n];
				const double face_plus = face(cm1, c, cp1, cp2, ud);   // phi at i+1/2
				const double face_minus = face(cm2, cm1, c, cp1, ud);  // phi at i-1/2
				change -= ud*(face_plus-face_minus); // eq 42
				const double Dn = (double)D[n];
				const double Dp = solid[p1] ? 0.0 : 0.5*(Dn+(double)D[p1]), Dm = solid[m1] ? 0.0 : 0.5*(Dn+(double)D[m1]); // eq 46, no flux through a wall
				change += Dp*(cp1-c)-Dm*(c-cm1);
			}
			next[n] = phi[n]+change;
			if(float_storage) next[n] = (double)(float)next[n]; // the field stored as a float after every step, as a GPU buffer of T
		} });
		phi.swap(next);
	}
public:
	/// @brief A diagnostic: store the field as a float after every step.
	void use_float_storage(const bool on) { float_storage = on; }
	/// @brief Adds a source to one cell after a step (the kernel adds the floor's heat in the full stage, the same).
	/// @param n the cell
	/// @param value the amount added
	void add(const ulong n, const double value) { phi[n] += value; }
	/// @brief The value of a cell.
	double at(const ulong n) const { return phi[n]; }
	/// @brief Whether a cell is solid.
	bool is_solid(const ulong n) const { return solid[n]; }
private:
	uint Nx = 0u, Ny = 0u, Nz = 0u;
	ulong N = 0ull;
	std::vector<bool> solid;
	std::vector<double> phi, next;
	bool float_storage{};
	ulong index(const uint x, const uint y, const uint z) const { return (ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx; }
	/// @brief The cell s steps along direction d (periodic), the index of a solid cell included.
	ulong neighbour(const uint x, const uint y, const uint z, const uint d, const int s) const {
		const auto wrap = [](const uint v, const int s, const uint L) { return (uint)(((int)v+s%(int)L+(int)L)%(int)L); };
		if(d==0u) return index(wrap(x, s, Nx), y, z);
		if(d==1u) return index(x, wrap(y, s, Ny), z);
		return index(x, y, wrap(z, s, Nz));
	}
	/// @brief The value of cell m, or of the fluid cell `inner` next to it when m is solid (a zero-gradient ghost).
	double value_at(const ulong m, const ulong inner) const { return solid[m] ? phi[inner] : phi[m]; }
public: // the reconstruction, shared with ConservativeFiniteVolumeScalars
	/// @brief van Albada: phi(r) = 2r / (1 + r^2), 0 for r <= 0.
	static double limiter(const double r) { return r<=0.0 ? 0.0 : 2.0*r/(1.0+r*r); }
	/// @brief The upwind MUSCL value at the face between cells b and c of the run a, b, c, e (eqs 43-45, kappa 1/3).
	static double face(const double a, const double b, const double c, const double e, const double u) {
		const double kappa = 1.0/3.0;
		if(u>0.0) { // theta^L: from b, with the differences a-b and b-c
			const double dm = b-a, dp = c-b;
			const double r = dp!=0.0 ? dm/dp : 0.0;
			return b+limiter(r)/4.0*((1.0-kappa)*dm+(1.0+kappa)*dp);
		}
		const double dp2 = e-c, dp = c-b; // theta^R: from c, with the differences c-e and b-c
		const double r = dp2!=0.0 ? dp/dp2 : 0.0;
		return c-limiter(r)/4.0*((1.0-kappa)*dp2+(1.0+kappa)*dp);
	}
};

/// @brief Finite-volume scalars in conservative form with their own density and 7-point face mass fluxes
/// (Guo, Feng, Sagaut 2021, Computers & Fluids 219, eq 17): rho_FV and every rho phi change by the same face mass
/// fluxes, phi = (rho phi) / rho_FV. A uniform phi stays uniform (numerator and denominator change by the same flux
/// times phi) and the sum of rho phi changes only by the sources, whatever face mass flux is used. The face mass flux
/// is the mean of the two cells' rho u (a 7-point flux, which the LBM's own mass transport does not satisfy); the face value of phi is
/// the upwind MUSCL value of FiniteVolumeScalar by the flux's sign. No flux through a solid face. The midpoint step
/// keeps the step's mass fluxes in both stages, so both properties hold in each stage.
class ConservativeFiniteVolumeScalars {
public:
	/// @param nx, ny, nz the box (periodic where no solid cell is)
	/// @param solid_cells true for the TYPE_S cells
	/// @param rho_start the start density of every cell (the LBM's)
	/// @param starts the start value of every cell, one vector per scalar
	ConservativeFiniteVolumeScalars(const uint nx, const uint ny, const uint nz, const std::vector<bool>& solid_cells, const std::vector<float>& rho_start, const std::vector<std::vector<double>>& starts)
		: Nx(nx), Ny(ny), Nz(nz), N((ulong)nx*(ulong)ny*(ulong)nz), solid(solid_cells), rho(rho_start.begin(), rho_start.end()) {
		if(solid.size()!=N||rho.size()!=N) print_error("ConservativeFiniteVolumeScalars: the solid and density vectors need Nx Ny Nz values.");
		for(const std::vector<double>& start : starts) {
			if(start.size()!=N) print_error("ConservativeFiniteVolumeScalars: every start vector needs Nx Ny Nz values.");
			std::vector<double> mass(N, 0.0);
			for(ulong n{}; n<N; n++) mass[n] = rho[n]*start[n];
			rho_phi.push_back(mass);
		}
	}
	/// @brief One midpoint step: advection by the face mass fluxes of rho u, diffusion by D.
	/// @param rho_lbm the LBM's density of every cell (N floats)
	/// @param u ux, uy, uz of every cell (3 N floats, structure of arrays)
	/// @param D the diffusivity of every cell (N floats, lattice units)
	void step(const float* rho_lbm, const float* u, const float* D) {
		const ulong S = rho_phi.size();
		std::vector<double> rho_change(N, 0.0);
		std::vector<std::vector<double>> change(S, std::vector<double>(N, 0.0)), phi(S, std::vector<double>(N, 0.0));
		for(ulong s{}; s<S; s++) for(ulong n{}; n<N; n++) phi[s][n] = rho_phi[s][n]/rho[n];
		for(ulong s{}; s<S; s++) changes(rho_lbm, u, D, phi[s], change[s], s==0ull ? &rho_change : nullptr); // the first stage at phi^n
		for(ulong s{}; s<S; s++) for(ulong n{}; n<N; n++) phi[s][n] = (rho_phi[s][n]+0.5*change[s][n])/(rho[n]+0.5*rho_change[n]); // phi at the half step
		for(ulong s{}; s<S; s++) {
			changes(rho_lbm, u, D, phi[s], change[s], nullptr); // the second stage at phi_1/2, the same mass fluxes
			for(ulong n{}; n<N; n++) rho_phi[s][n] += change[s][n];
		}
		for(ulong n{}; n<N; n++) rho[n] += rho_change[n];
	}
	/// @brief Adds a source to one cell's rho phi after a step (as the kernel adds the floor's heat).
	/// @param s the scalar
	/// @param n the cell
	/// @param value the amount of rho phi added
	void add(const ulong s, const ulong n, const double value) { rho_phi[s][n] += value; }
	/// @brief phi of one cell.
	/// @param s the scalar
	/// @param n the cell
	/// @return (rho phi) / rho_FV
	double at(const ulong s, const ulong n) const { return rho_phi[s][n]/rho[n]; }
	/// @brief rho phi of one cell.
	/// @param s the scalar
	/// @param n the cell
	/// @return the conserved quantity
	double mass_at(const ulong s, const ulong n) const { return rho_phi[s][n]; }
	/// @brief The scheme's own density of one cell.
	/// @param n the cell
	/// @return rho_FV
	double density_at(const ulong n) const { return rho[n]; }
private:
	uint Nx{}, Ny{}, Nz{};
	ulong N{};
	std::vector<bool> solid;
	std::vector<double> rho;
	std::vector<std::vector<double>> rho_phi;
	ulong index(const uint x, const uint y, const uint z) const { return (ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx; }
	/// @brief The cell s steps along direction d (periodic).
	ulong neighbour(const uint x, const uint y, const uint z, const uint d, const int s) const {
		const auto wrap = [](const uint v, const int s, const uint L) { return (uint)(((int)v+s%(int)L+(int)L)%(int)L); };
		if(d==0u) return index(wrap(x, s, Nx), y, z);
		if(d==1u) return index(x, wrap(y, s, Ny), z);
		return index(x, y, wrap(z, s, Nz));
	}
	/// @brief The value of cell m, or of the fluid cell inner next to it when m is solid (a zero-gradient ghost).
	double value_at(const std::vector<double>& phi, const ulong m, const ulong inner) const { return solid[m] ? phi[inner] : phi[m]; }
	/// @brief The face mass flux between two cells along d: the mean of their rho u, 0 through a solid face.
	double mass_flux(const float* rho_lbm, const float* u, const uint d, const ulong a, const ulong b) const {
		if(solid[a]||solid[b]) return 0.0;
		return 0.5*((double)rho_lbm[a]*(double)u[(ulong)d*N+a]+(double)rho_lbm[b]*(double)u[(ulong)d*N+b]);
	}
	/// @brief The change of rho phi (and of rho_FV, when rho_change is given) of every cell for one full step.
	/// @param rho_lbm, u, D the step's flow, as in step()
	/// @param phi the scalar the face values come from
	/// @param change the change of rho phi (written)
	/// @param rho_change the change of rho_FV (written), or nullptr
	void changes(const float* rho_lbm, const float* u, const float* D, const std::vector<double>& phi, std::vector<double>& change, std::vector<double>* rho_change) const {
		parallel_for(Nz, [&](const uint z) { for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
			const ulong n = index(x, y, z);
			double dm{}, drho{};
			if(!solid[n]) for(uint d=0u; d<3u; d++) {
				const ulong m1 = neighbour(x, y, z, d, -1), m2 = neighbour(x, y, z, d, -2), p1 = neighbour(x, y, z, d, 1), p2 = neighbour(x, y, z, d, 2);
				const double c = phi[n], cm1 = value_at(phi, m1, n), cp1 = value_at(phi, p1, n);
				const double cm2 = solid[m1] ? cm1 : value_at(phi, m2, m1), cp2 = solid[p1] ? cp1 : value_at(phi, p2, p1); // behind a wall the ghost continues
				const double m_plus = mass_flux(rho_lbm, u, d, n, p1), m_minus = mass_flux(rho_lbm, u, d, m1, n);
				// each face's mass flux and face value are the same seen from both of its cells: the sum is conserved
				dm -= m_plus*FiniteVolumeScalar::face(cm1, c, cp1, cp2, m_plus)-m_minus*FiniteVolumeScalar::face(cm2, cm1, c, cp1, m_minus);
				drho -= m_plus-m_minus;
				const double Dn = (double)D[n];
				const double Dp = solid[p1] ? 0.0 : 0.5*(Dn+(double)D[p1]), Dm = solid[m1] ? 0.0 : 0.5*(Dn+(double)D[m1]); // eq 46, no flux through a wall
				dm += Dp*(cp1-c)-Dm*(c-cm1);
			}
			change[n] = dm;
			if(rho_change!=nullptr) (*rho_change)[n] = drho;
		} });
	}
};

/// @brief Finite-volume scalars in conservative form, transported by the LBM's own link mass fluxes
/// (Zhao et al. 2020, Phys. Fluids 32, eqs 18 and C4, with the density of Guo et al. 2021, eq 17). On the
/// link from cell n to n + c_i the net mass is psi_i(n) = f*_i(n) - f*_ibar(n + c_i) (post-collision populations), the
/// LBM's own transport: rho_FV changes by -sum_i psi_i like the LBM density, and rho phi by -sum_i psi_i phi_face with
/// the upwind MUSCL value along the link. psi_ibar(n + c_i) = -psi_i(n) exactly, so the sum is conserved; a uniform phi
/// stays uniform. psi is 0 on a link with a solid cell. Diffusion as FiniteVolumeScalar (7 points). Midpoint step with
/// the step's link fluxes in both stages.
class LinkFluxFiniteVolumeScalars {
public:
	/// @brief The D3Q19 velocity set in FluidX3D's order (kernel.cpp c_data): direction i and i + 1 are opposite (i odd).
	static constexpr int velocity[19][3]{ { 0, 0, 0 }, { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 },
		{ 1, 1, 0 }, { -1, -1, 0 }, { 1, 0, 1 }, { -1, 0, -1 }, { 0, 1, 1 }, { 0, -1, -1 }, { 1, -1, 0 }, { -1, 1, 0 },
		{ 1, 0, -1 }, { -1, 0, 1 }, { 0, 1, -1 }, { 0, -1, 1 } };
	/// @brief The direction opposite to i.
	/// @param i a direction 1 to 18
	/// @return ibar
	static uint opposite(const uint i) { return i%2u==1u ? i+1u : i-1u; }
	/// @param nx, ny, nz the box (periodic where no solid cell is)
	/// @param solid_cells true for the TYPE_S cells
	/// @param rho_start the start density of every cell (the LBM's)
	/// @param starts the start value of every cell, one vector per scalar
	LinkFluxFiniteVolumeScalars(const uint nx, const uint ny, const uint nz, const std::vector<bool>& solid_cells, const std::vector<float>& rho_start, const std::vector<std::vector<double>>& starts)
		: Nx(nx), Ny(ny), Nz(nz), N((ulong)nx*(ulong)ny*(ulong)nz), solid(solid_cells), rho(rho_start.begin(), rho_start.end()) {
		if(solid.size()!=N||rho.size()!=N) print_error("LinkFluxFiniteVolumeScalars: the solid and density vectors need Nx Ny Nz values.");
		for(const std::vector<double>& start : starts) {
			if(start.size()!=N) print_error("LinkFluxFiniteVolumeScalars: every start vector needs Nx Ny Nz values.");
			std::vector<double> mass(N, 0.0);
			for(ulong n{}; n<N; n++) mass[n] = rho[n]*start[n];
			rho_phi.push_back(mass);
		}
		next.assign(19ull*N, 0u); // the neighbour table: every step reads it about 200 times per cell
		parallel_for(Nz, [&](const uint z) { for(ulong n{ (ulong)z*Nx*Ny }; n<(ulong)(z+1u)*Nx*Ny; n++) for(uint i=0u; i<19u; i++) next[(ulong)i*N+n] = (uint)along(n, i, 1); });
	}
	/// @brief The neighbour of n along direction i (the table of along(n, i, 1)).
	/// @param n the cell
	/// @param i the direction 0 to 18
	/// @return the cell index
	ulong step_to(const ulong n, const uint i) const { return (ulong)next[(ulong)i*N+n]; }
	/// @brief The cell reached from n by s steps along direction i (periodic).
	/// @param n the cell
	/// @param i the direction 0 to 18
	/// @param s the number of steps (negative: backwards)
	/// @return the cell index
	ulong along(const ulong n, const uint i, const int s) const {
		const auto wrap = [](const ulong v, const int d, const uint L) { return (ulong)(((long long)v+(long long)d%(long long)L+(long long)L)%(long long)L); };
		const ulong x{ n%Nx }, y{ (n/Nx)%Ny }, z{ n/((ulong)Nx*Ny) };
		return wrap(x, s*velocity[i][0], Nx)+(wrap(y, s*velocity[i][1], Ny)+wrap(z, s*velocity[i][2], Nz)*(ulong)Ny)*(ulong)Nx;
	}
	/// @brief The link mass fluxes of one step from the DDFs the step stored (Esoteric-Pull): the populations the next
	/// step loads are f_i(n) = f*_i(n - c_i) (kernel.cpp load_f with the next step's t), so psi_i(n) = f_i(n + c_i) - f_ibar(n).
	/// @param fi the DDF buffer after the step (index i N + n, any constant shift per direction pair cancels)
	/// @param t_next the time step the next stream_collide runs with (LBM::get_t() after the step)
	/// @param psi the fluxes (written, 18 N values, psi_i(n) at (i - 1) N + n; 0 on a link with a solid cell)
	void link_fluxes(const float* fi, const ulong t_next, std::vector<double>& psi) const {
		psi.assign(18ull*N, 0.0);
		parallel_for(Nz, [&](const uint z) { for(ulong n{ (ulong)z*Nx*Ny }; n<(ulong)(z+1u)*Nx*Ny; n++) {
			if(solid[n]) continue;
			for(uint i=1u; i<19u; i++) {
				const ulong m{ step_to(n, i) };
				if(solid[m]) continue;
				psi[(ulong)(i-1u)*N+n] = incoming(fi, t_next, m, i)-incoming(fi, t_next, n, opposite(i));
			}
		} });
	}
	/// @brief The population of direction i that cell n loads in the next step (kernel.cpp load_f): f*_i(n - c_i) of
	/// the step that stored fi. For a solid cell n it is the slot that the fluid cell n - c_i wrote its outgoing f*_i
	/// into, the population in transit to the wall (Esoteric-Pull returns it two steps later).
	/// @param fi the DDF buffer after the step
	/// @param t_next the time step the next stream_collide runs with
	/// @param n the cell
	/// @param i the direction 1 to 18
	/// @return the population (shifted by -w_i, as stored)
	double incoming(const float* fi, const ulong t_next, const ulong n, const uint i) const {
		if(i%2u==1u) return (double)fi[(ulong)(t_next%2ull ? i : i+1u)*N+n]; // odd i from the cell itself
		return (double)fi[(ulong)(t_next%2ull ? i : i-1u)*N+step_to(n, i-1u)]; // even i from its neighbour along i - 1
	}
	/// @brief One midpoint step with the step's link fluxes in both stages, diffusion by D.
	/// @param psi the link fluxes of the step (link_fluxes())
	/// @param D the diffusivity of every cell (N floats, lattice units)
	void step(const std::vector<double>& psi, const float* D) {
		const ulong S{ rho_phi.size() };
		std::vector<double> rho_change(N, 0.0);
		for(ulong n{}; n<N; n++) for(uint i=0u; i<18u; i++) rho_change[n] -= psi[(ulong)i*N+n];
		std::vector<std::vector<double>> change(S, std::vector<double>(N, 0.0)), phi(S, std::vector<double>(N, 0.0));
		for(ulong s{}; s<S; s++) for(ulong n{}; n<N; n++) phi[s][n] = rho_phi[s][n]/rho[n];
		for(ulong s{}; s<S; s++) changes(psi, D, phi[s], change[s]); // the first stage at phi^n
		for(ulong s{}; s<S; s++) for(ulong n{}; n<N; n++) phi[s][n] = (rho_phi[s][n]+0.5*change[s][n])/(rho[n]+0.5*rho_change[n]); // phi at the half step
		for(ulong s{}; s<S; s++) {
			changes(psi, D, phi[s], change[s]); // the second stage at phi_1/2, the same link fluxes
			for(ulong n{}; n<N; n++) rho_phi[s][n] += change[s][n];
		}
		for(ulong n{}; n<N; n++) rho[n] += rho_change[n];
	}
	/// @brief Adds a source to one cell's rho phi after a step.
	/// @param s the scalar
	/// @param n the cell
	/// @param value the amount of rho phi added
	void add(const ulong s, const ulong n, const double value) { rho_phi[s][n] += value; }
	/// @brief phi of one cell.
	/// @param s the scalar
	/// @param n the cell
	/// @return (rho phi) / rho_FV
	double at(const ulong s, const ulong n) const { return rho_phi[s][n]/rho[n]; }
	/// @brief rho phi of one cell.
	/// @param s the scalar
	/// @param n the cell
	/// @return the conserved quantity
	double mass_at(const ulong s, const ulong n) const { return rho_phi[s][n]; }
	/// @brief The scheme's own density of one cell.
	/// @param n the cell
	/// @return rho_FV
	double density_at(const ulong n) const { return rho[n]; }
private:
	uint Nx{}, Ny{}, Nz{};
	ulong N{};
	std::vector<bool> solid;
	std::vector<double> rho;
	std::vector<std::vector<double>> rho_phi;
	std::vector<uint> next; // next[i N + n]: the neighbour of n along direction i
	/// @brief The value of cell m, or of the fluid cell inner next to it when m is solid (a zero-gradient ghost).
	double value_at(const std::vector<double>& phi, const ulong m, const ulong inner) const { return solid[m] ? phi[inner] : phi[m]; }
	/// @brief The change of rho phi of every cell for one full step: the link fluxes with the upwind MUSCL value along
	/// each link, and the 7-point diffusion.
	/// @param psi the link fluxes
	/// @param D the diffusivity of every cell
	/// @param phi the scalar the face values come from
	/// @param change the change of rho phi (written)
	void changes(const std::vector<double>& psi, const float* D, const std::vector<double>& phi, std::vector<double>& change) const {
		parallel_for(Nz, [&](const uint z) { for(ulong n{ (ulong)z*Nx*Ny }; n<(ulong)(z+1u)*Nx*Ny; n++) {
			double dm{};
			if(!solid[n]) {
				for(uint i=1u; i<19u; i++) {
					const double flux{ psi[(ulong)(i-1u)*N+n] };
					if(flux==0.0) continue; // no mass on the link (or a solid cell on it)
					const ulong m1{ step_to(n, opposite(i)) }, p1{ step_to(n, i) }, p2{ step_to(p1, i) };
					const double c{ phi[n] }, cm1{ value_at(phi, m1, n) }, cp1{ value_at(phi, p1, n) };
					const double cp2{ solid[p1] ? cp1 : value_at(phi, p2, p1) }; // behind a wall the ghost continues
					dm -= flux*FiniteVolumeScalar::face(cm1, c, cp1, cp2, flux); // the face between n and n + c_i, upwind by the net mass's sign
				}
				for(uint i=1u; i<7u; i++) { // the 7-point diffusion, the face diffusivity (D_n + D_m) / 2, no flux through a wall
					const ulong m{ step_to(n, i) };
					if(solid[m]) continue;
					dm +=0.5*((double)D[n]+(double)D[m])*(phi[m]-phi[n]);
				}
			}
			change[n] = dm;
		} });
	}
};
