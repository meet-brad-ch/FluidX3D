// Characterization baseline: prints the host-side state an example's setup produced, then exits.
// Linked only into the <example>_baseline targets, which compile the core with FLUIDX3D_BASELINE;
// LBM::run() calls fluidx3d_baseline_dump() before the first initialization.
#include "lbm.hpp"
#include <cstdio>
#include <cstdlib>

extern Units units;

namespace {

void print_line(const char* key, const double value) {
	std::printf("BASELINE %s %.6g\n", key, value);
}

void print_line(const char* key, const double x, const double y, const double z) {
	std::printf("BASELINE %s %.6g %.6g %.6g\n", key, x, y, z);
}

void print_count(const char* key, const ulong value) { // integers are printed exactly
	std::printf("BASELINE %s %llu\n", key, (unsigned long long)value);
}

void print_count(const char* key, const ulong x, const ulong y, const ulong z) {
	std::printf("BASELINE %s %llu %llu %llu\n", key, (unsigned long long)x, (unsigned long long)y, (unsigned long long)z);
}

void print_flags(LBM& lbm) { // non-const LBM&: Memory_Container has no usable const operator[]
	constexpr uchar types[] = { TYPE_S, TYPE_E, TYPE_T, TYPE_F, TYPE_I, TYPE_G, TYPE_X, TYPE_Y };
	constexpr const char* names[] = { "cells_S", "cells_E", "cells_T", "cells_F", "cells_I", "cells_G", "cells_X", "cells_Y" };
	ulong counts[8] = {};
	ulong solid_count = 0ull, object_count = 0ull; // object: solid cells not on a domain face (excludes floors and walls)
	uint object_min[3] = { max_uint, max_uint, max_uint }, object_max[3] = { 0u, 0u, 0u };
	const uint N[3] = { lbm.get_Nx(), lbm.get_Ny(), lbm.get_Nz() };
	double u_sum[3] = { 0.0, 0.0, 0.0 }, rho_sum = 0.0;
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		const uchar flag = lbm.flags[n];
		for(uint i=0u; i<8u; i++) if(flag&types[i]) counts[i]++;
		if(flag&TYPE_S) {
			solid_count++;
			uint xyz[3];
			lbm.coordinates(n, xyz[0], xyz[1], xyz[2]);
			bool on_face = false;
			for(uint i=0u; i<3u; i++) on_face = on_face || xyz[i]==0u || xyz[i]==N[i]-1u;
			if(on_face) continue;
			object_count++;
			for(uint i=0u; i<3u; i++) {
				object_min[i] = min(object_min[i], xyz[i]);
				object_max[i] = max(object_max[i], xyz[i]);
			}
		} else {
			u_sum[0] += (double)lbm.u.x[n];
			u_sum[1] += (double)lbm.u.y[n];
			u_sum[2] += (double)lbm.u.z[n];
			rho_sum += (double)lbm.rho[n];
		}
	}
	for(uint i=0u; i<8u; i++) print_count(names[i], counts[i]);
	print_count("object_cells", object_count);
	if(object_count>0ull) {
		print_count("object_min", object_min[0], object_min[1], object_min[2]);
		print_count("object_max", object_max[0], object_max[1], object_max[2]);
	}
	const double fluid_count = (double)(lbm.get_N()-solid_count);
	if(fluid_count>0.0) {
		print_line("u_mean", u_sum[0]/fluid_count, u_sum[1]/fluid_count, u_sum[2]/fluid_count);
		print_line("rho_mean", rho_sum/fluid_count);
	}
}

void print_extensions(LBM& lbm) {
#ifdef VOLUME_FORCE
	print_line("force_per_volume", lbm.get_fx(), lbm.get_fy(), lbm.get_fz());
#endif // VOLUME_FORCE
#ifdef SURFACE
	print_line("sigma", lbm.get_sigma());
	double phi_sum = 0.0;
	for(ulong n=0ull; n<lbm.get_N(); n++) phi_sum += (double)lbm.phi[n];
	print_line("phi_sum", phi_sum);
#endif // SURFACE
#ifdef TEMPERATURE
	print_line("alpha", lbm.get_alpha());
	print_line("beta", lbm.get_beta());
	double T_sum = 0.0;
	for(ulong n=0ull; n<lbm.get_N(); n++) T_sum += (double)lbm.T[n];
	print_line("T_mean", T_sum/(double)lbm.get_N());
#endif // TEMPERATURE
#ifdef PARTICLES
	const ulong particle_count = lbm.particles->length();
	double p_sum[3] = { 0.0, 0.0, 0.0 };
	for(ulong i=0ull; i<particle_count; i++) {
		p_sum[0] += (double)lbm.particles->x[i];
		p_sum[1] += (double)lbm.particles->y[i];
		p_sum[2] += (double)lbm.particles->z[i];
	}
	print_count("particles", particle_count);
	if(particle_count>0ull) print_line("particles_mean", p_sum[0]/(double)particle_count, p_sum[1]/(double)particle_count, p_sum[2]/(double)particle_count);
#endif // PARTICLES
	(void)lbm;
}

void print_graphics(LBM& lbm) {
#ifdef GRAPHICS
	print_line("vis_modes", (double)lbm.graphics.visualization_modes);
	print_line("field_mode", (double)lbm.graphics.field_mode);
	print_line("slice_mode", (double)lbm.graphics.slice_mode);
	print_line("camera_angles", camera.rx, camera.ry, (double)camera.fov);
	print_line("camera_zoom", (double)camera.zoom);
	print_line("camera_free", camera.free ? 1.0 : 0.0);
	if(camera.free) print_line("camera_pos", camera.pos.x, camera.pos.y, camera.pos.z);
#endif // GRAPHICS
	(void)lbm;
}

} // namespace

void fluidx3d_baseline_dump(LBM& lbm) {
	print_count("grid", lbm.get_Nx(), lbm.get_Ny(), lbm.get_Nz());
	print_count("domains", lbm.get_Dx(), lbm.get_Dy(), lbm.get_Dz());
	print_line("nu", lbm.get_nu());
	print_line("unit_m", units.si_x(1.0f));
	print_line("unit_s", units.si_t(1ull));
	print_line("unit_kg", units.si_m(1.0f));
	print_flags(lbm);
	print_extensions(lbm);
	print_graphics(lbm);
	std::fflush(stdout);
	std::_Exit(0); // skip destructors: the baseline run ends before any simulation
}
