#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // hydraulic jump; required extensions in defines.hpp: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SURFACE, SUBGRID, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint memory = 208u; // GPU VRAM in MB
	const float si_T = 100.0f; // simulated time in [s]

	const float3 si_N = float3(0.96f, 3.52f, 0.96f); // box size in [m]
	const float si_p1 = si_N.y*3.0f/20.0f; // socket length in [m]
	const float si_h1 = si_N.z*2.0f/5.0f; // socket height in [m]
	const float si_h2 = si_N.z*3.0f/5.0f; // water height in [m]

	const float si_Q = 0.25f; // inlet volumetric flow rate in [m^3/s]
	const float si_A_inlet = si_N.x*(si_h2-si_h1); // inlet cross-section area in [m^2]
	const float si_A_outlet = si_N.x*si_h1; // outlet cross-section area in [m^2]
	const float si_u_inlet = si_Q/si_A_inlet; // inlet average flow velocity in [m/s]
	const float si_u_outlet = si_Q/si_A_outlet; // outlet average flow velocity in [m/s]

	float const si_nu = 1.0E-6f; // kinematic shear viscosity [m^2/s]
	const float si_rho = 1000.0f; // water density [kg/m^3]
	const float si_g = 9.81f; // gravitational acceleration [m/s^2]
	//const float si_sigma = 73.81E-3f; // water surface tension [kg/s^2] (no need to use surface tension here)

	const uint3 lbm_N = resolution(si_N, memory); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_u_inlet = 0.075f; // velocity in LBM units for pairing lbm_u with si_u --> lbm_u in LBM units will be equivalent si_u in SI units
	units.set_m_kg_s((float)lbm_N.y, lbm_u_inlet, 1.0f, si_N.y, si_u_inlet, si_rho); // calculate 3 independent conversion factors (m, kg, s)

	const float lbm_nu = units.nu(si_nu); // kinematic shear viscosity
	const ulong lbm_T = units.t(si_T); // how many time steps to compute to cover exactly si_T seconds in real time
	const float lbm_f = units.f(si_rho, si_g); // force per volume
	//const float lbm_sigma = units.sigma(si_sigma); // surface tension (not required here)

	const uint lbm_p1 = to_uint(units.x(si_p1));
	const uint lbm_h1 = to_uint(units.x(si_h1));
	const uint lbm_h2 = to_uint(units.x(si_h2));
	const float lbm_u_outlet = units.u(si_u_outlet);

	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu, 0.0f, 0.0f, -lbm_f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(z<lbm_h2) {
			lbm.flags[n] = TYPE_F;
			lbm.rho[n] = units.rho_hydrostatic(0.0005f, z, lbm_h2);
		}
		if(y<lbm_p1&&z<lbm_h1) lbm.flags[n] = TYPE_S;
		if(y<=1u&&x>0u&&x<Nx-1u&&z>=lbm_h1&&z<lbm_h2) {
			lbm.flags[n] = y==0u ? TYPE_S : TYPE_F;
			lbm.u.y[n] = lbm_u_inlet;
		}
		if(y==Ny-1u&&x>0u&&x<Nx-1u&&z>0u) {
			lbm.flags[n] = TYPE_E;
			lbm.u.y[n] = lbm_u_outlet;
		}
		if(x==0u||x==Nx-1u||y==0u||z==0u) lbm.flags[n] = TYPE_S; // sides and bottom non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
	lbm.run();
	//lbm.run(1000u); lbm.u.read_from_device(); println(lbm.u.x[lbm.index(Nx/2u, Ny/4u, Nz/4u)]); wait(); // test for binary identity
} /**/
