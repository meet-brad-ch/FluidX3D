// Stokes drag validation in physical units, using Setup API: the force on a sphere in creeping flow, compared with
// Stokes' law F = 6*pi*mu*R*u in newtons

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // Stokes drag validation; required extensions: FORCE_FIELD, EQUILIBRIUM_BOUNDARIES
	const Length radius = 1.0_mm;        // the sphere's
	const Length cell = radius / 32.0f;  // 256³ cells, as the original
	const float reynolds = 0.01f;        // over the sphere's diameter
	const KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	const Density density = Fluid::WATER.density;
	const Speed flow_speed = reynolds * viscosity / (2.0f * radius); // 5 micrometres per second in water

	SimulationSetup sim(Domain::box(8.0f * radius, 8.0f * radius, 8.0f * radius).cell_size(cell));
	sim.setup();
	sim.configure_units(flow_speed, Fluid::WATER, reynolds / 64.0f); // lattice viscosity 1 (relaxation time 3.5), as the original

	LBM lbm = sim.create_lbm(viscosity);

	// Stokes' solution around the sphere as the start, for the flow along -X; r from the sphere's center
	const Position center { 4.0f * radius, 4.0f * radius, 4.0f * radius };
	const float3 u0(-flow_speed.si(), 0.0f, 0.0f);
	const auto from_center = [=](Position p) { return float3((p.x - center.x).si(), (p.y - center.y).si(), (p.z - center.z).si()); };
	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.add_solid(Shape::sphere(center, radius), TYPE_S | TYPE_X) // TYPE_X: its force is measured
		.initialize_velocity([=](Position p) {
			const float3 u = units.u_Stokes(from_center(p), u0, radius.si());
			return Velocity{ Speed::from_si(u.x), Speed::from_si(u.y), Speed::from_si(u.z) };
		})
		.initialize_pressure([=](Position p) {
			const float3 r = from_center(p);
			return Pressure::from_si(-1.5f * density.si() * viscosity.si() * radius.si() * dot(u0, r) / cb(length(r)));
		})
		.apply();

	ForceAnalyzer drag(lbm);
	const double F_theory = units.F_Stokes(density.si(), flow_speed.si(), viscosity.si(), radius.si()); // N
	double E1 = 1000.0, E2 = 1000.0;
	while(true) { // main simulation loop
		lbm.run(100u); // check the error every 100 time steps
		const double F_simulated = (double)length(drag.get_force_si()); // N
		const double E0 = fabs(F_simulated - F_theory) / F_theory;
		print_info(to_string(lbm.get_t()) + ", expected: " + to_string(1E9 * F_theory, 6u) + " nN, measured: " + to_string(1E9 * F_simulated, 6u) + " nN, error = " + to_string((float)(100.0 * E0), 1u) + "%");
		if(converged(E2, E1, E0, 1E-4)) { // stop when error has sufficiently converged
			print_info("Error converged after " + to_string(lbm.get_t()) + " steps to " + to_string(100.0 * E0, 1u) + "%");
			wait();
			break;
		}
		E2 = E1;
		E1 = E0;
	}
} /**/
