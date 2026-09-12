// Stokes drag: the force on a sphere in creeping flow of water (Re 0.01), against Stokes' law F = 6*pi*mu*R*u in newtons.
// Passes when the converged force is within 5 % of Stokes' law.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"
#include "physics_check.hpp"

void main_setup() { // required extensions: FORCE_FIELD, EQUILIBRIUM_BOUNDARIES
	const Length radius = 1.0_mm;
	const Length cell = radius / 32.0f; // 256³ cells
	const float reynolds = 0.01f;       // over the diameter
	const KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	const Density density = Fluid::WATER.density;
	const Speed flow_speed = reynolds * viscosity / (2.0f * radius);

	SimulationSetup sim(Domain::box(8.0f * radius, 8.0f * radius, 8.0f * radius).cell_size(cell));
	sim.setup();
	sim.configure_units(flow_speed, Fluid::WATER, LatticeMach(sqrtf(3.0f) * reynolds / 64.0f)); // lattice viscosity 1

	LBM lbm = sim.create_lbm(viscosity);

	// Stokes' solution around the sphere as the start, for the flow along -X
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
	const double F_stokes = units.F_Stokes(density.si(), flow_speed.si(), viscosity.si(), radius.si()); // N
	double E1 = 1000.0, E2 = 1000.0;
	Runner runner(lbm);
	runner.every(0.1_s, [&](Duration t) { // about every 100 time steps
		if(t == Duration{}) return; // no flow has developed at the start
		const double E0 = fabs((double)length(drag.get_force_si()) - F_stokes) / F_stokes;
		if(converged(E2, E1, E0, 1E-4) || t > 30.0_s) { // converged (or not converging)
			PhysicsCheck::report("Stokes drag, force on the sphere", E0, 0.05);
			runner.stop();
		}
		E2 = E1;
		E1 = E0;
	});
	runner.run();
}
