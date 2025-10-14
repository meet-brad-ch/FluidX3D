#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // city_rt_real: Real-time interactive wind flow with an atmospheric boundary layer profile.
	// Required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS

	// ################################################################## define simulation box size and fluid properties ##################################################################
	const uint L = 512u; // Base resolution
	const float si_u_ref = 1.0f / 3.6f; // Reference wind speed in m/s (e.g., 10 km/h)
	const float si_h_ref = 100.0f;       // Reference height in meters for the wind profile
	const float si_building_size = 1000.0f; // Characteristic size of the building in meters
	const float si_rho = 1.225f;       // Air density in kg/m^3
	const float si_nu = 1.48E-5f;      // Kinematic viscosity of air in m^2/s

	const float lbm_u_ref = 0.07f;     // Reference velocity in LBM units
	const float lbm_building_size = 1.7f * (float)L;

	// Set up the unit conversion system
	units.set_m_kg_s(lbm_building_size, lbm_u_ref, 1.0f, si_building_size, si_u_ref, si_rho);
	const float lbm_nu = units.nu(si_nu);
	print_info("Reynolds number Re = " + to_string(units.si_Re(si_building_size, si_u_ref, si_nu)));

	LBM lbm(L, L * 2u, L / 2u, lbm_nu);

	// ###################################################################################### define geometry ######################################################################################
	const float3 center = lbm.center() - float3(0.0f, 0.05f * lbm_building_size, 0.025f * lbm_building_size);
	const float3x3 rotation = float3x3(float3(0, 0, 1), radians(90.0f));
	lbm.voxelize_stl(get_resource_path("city.stl"), center, rotation, lbm_building_size);

	// ######################################################## define atmospheric boundary layer and initial conditions #######################################################
	const uint Nx = lbm.get_Nx(), Ny = lbm.get_Ny(), Nz = lbm.get_Nz();

	// Parameters for the power-law velocity profile U(z) = U_ref * (z / z_ref)^alpha
	const float lbm_h_ref = units.x(si_h_ref); // Reference height in LBM units
	const float alpha = 0.25f; // Power-law exponent for urban/suburban terrain

	parallel_for(lbm.get_N(), [&](ulong n) {
		uint x = 0u, y = 0u, z = 0u;
		lbm.coordinates(n, x, y, z);

		// Calculate velocity at this height using the power-law profile
		// Use z+0.5f to get cell-center height, add small epsilon to avoid z=0
		float height = (float)z + 0.5f;
		float velocity_at_height = lbm_u_ref * powf(height / lbm_h_ref, alpha);

		// Initialize velocity for all non-solid cells with the profile
		if (lbm.flags[n] != TYPE_S) {
			lbm.u.y[n] = velocity_at_height;
		}

		// --- Set Boundary Conditions ---
		// Inlet (y=0): Enforce the velocity profile
		if (y == 0u) {
			lbm.flags[n] = TYPE_E;
		}
		// Outlets (y=Ny-1, x=0, x=Nx-1, z=Nz-1): Open boundaries to simulate atmosphere
		else if (x == 0u || x == Nx - 1u || y == Ny - 1u || z == Nz - 1u) {
			lbm.flags[n] = TYPE_E;
			// Also initialize outflow/top/side velocities to the profile to prevent
			// a large pressure drop at the start of the simulation.
			if (lbm.flags[n] != TYPE_S) {
				lbm.u.y[n] = velocity_at_height;
			}
		}

		// Ground (z=0): No-slip solid wall
		if (z == 0u) {
			lbm.flags[n] = TYPE_S;
		}
	});

	// ####################################################################### run simulation and set graphics #######################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE | VIS_Q_CRITERION;
	lbm.graphics.set_camera_free(float3(-1.088245f * (float)Nx, -0.443919f * (float)Ny, 1.717979f * (float)Nz), 215.0f, 39.0f, 70.0f);

	// Run in interactive mode. Use mouse to rotate, press P to start/pause.
	lbm.run();
}