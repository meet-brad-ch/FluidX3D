#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // Poiseuille flow validation; required extensions in defines.hpp: VOLUME_FORCE
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint R = 63u; // channel radius (default: 63)
	const float umax = 0.1f; // maximum velocity in channel center (must be < 0.57735027f)
	const float tau = 1.0f; // relaxation time (must be > 0.5f), tau = nu*3+0.5
	const float nu = units.nu_from_tau(tau); // nu = (tau-0.5)/3
	const uint H = 2u*(R+1u);
#ifndef D2Q9
	LBM lbm(H, lcm(sq(H), WORKGROUP_SIZE)/sq(H), H, nu, 0.0f, units.f_from_u_Poiseuille_3D(umax, 1.0f, nu, R), 0.0f); // 3D
#else // D2Q9
	LBM lbm(lcm(H, WORKGROUP_SIZE)/H, H, 1u, nu, units.f_from_u_Poiseuille_2D(umax, 1.0f, nu, R), 0.0f, 0.0f); // 2D
#endif // D2Q9
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
#ifndef D2Q9
		if(!cylinder(x, y, z, lbm.center(), float3(0u, Ny, 0u), 0.5f*(float)min(Nx, Nz)-1.0f)) lbm.flags[n] = TYPE_S; // 3D
#else // D2Q9
		if(y==0u||y==Ny-1u) lbm.flags[n] = TYPE_S; // 2D
#endif // D2Q9
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	double error_min = max_double;
	while(true) { // main simulation loop
		lbm.run(1000u);
		lbm.u.read_from_device();
		double error_dif=0.0, error_sum=0.0;
#ifndef D2Q9
		for(uint x=0u; x<Nx; x++) {
			for(uint y=Ny/2u; y<Ny/2u+1u; y++) {
				for(uint z=0; z<Nz; z++) {
					const uint n = x+(y+z*Ny)*Nx;
					const double r = (double)sqrt(sq(x+0.5f-0.5f*(float)Nx)+sq(z+0.5f-0.5f*(float)Nz)); // radius from channel center
					if(r<R) {
						const double unum = (double)sqrt(sq(lbm.u.x[n])+sq(lbm.u.y[n])+sq(lbm.u.z[n])); // numerical velocity
						const double uref = umax*(sq(R)-sq(r))/sq(R); // theoretical velocity profile u = G*(R^2-r^2)
						error_dif += sq(unum-uref); // L2 error (Krüger p. 138)
						error_sum += sq(uref);
					}
				}
			}
		}
#else // D2Q9
		for(uint x=Nx/2u; x<Nx/2u+1u; x++) {
			for(uint y=1u; y<Ny-1u; y++) {
				const uint n = x+(y+0u*Ny)*Nx;
				const double r = (double)(y+0.5f-0.5f*(float)Ny); // radius from channel center
				const double unum = (double)sqrt(sq(lbm.u.x[n])+sq(lbm.u.y[n])); // numerical velocity
				const double uref = umax*(sq(R)-sq(r))/sq(R); // theoretical velocity profile u = G*(R^2-r^2)
				error_dif += sq(unum-uref); // L2 error (Krüger p. 138)
				error_sum += sq(uref);
			}
		}
#endif // D2Q9
		if(sqrt(error_dif/error_sum)>=error_min) { // stop when error has converged
			print_info("Poiseuille flow error converged after "+to_string(lbm.get_t())+" steps to "+to_string(100.0*error_min, 3u)+"%"); // typical expected L2 errors: 2-5% (Krüger p. 256)
			wait();
			exit(0);
		}
		error_min = fmin(error_min, sqrt(error_dif/error_sum));
		print_info("Poiseuille flow error after t="+to_string(lbm.get_t())+" is "+to_string(100.0*error_min, 3u)+"%"); // typical expected L2 errors: 2-5% (Krüger p. 256)
	}
} /**/
