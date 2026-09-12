// Hydrostatic pressure: a closed column of glycerol at rest under gravity, against dp/dz = -rho*g.
// Passes when the pressure difference between two heights, in pascals, is within 1 % of rho*g*dz.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"
#include "physics_check.hpp"

void main_setup() { // required extensions: VOLUME_FORCE
	const Length height = 10.0_cm, width = 2.5_cm;
	const Length cell = height / 64.0f;                // 16 x 16 x 64 cells
	const Density density = 1260.0_kgpm3;              // glycerol at 20 °C: its viscosity damps the sound waves of the start
	const KinematicViscosity viscosity = 1.12E-3_m2ps;
	const Acceleration g = 9.81_mps2;

	SimulationSetup sim(Domain::box(width, width, height).cell_size(cell));
	sim.setup();
	sim.configure_units(sqrt(g * height), density); // the speed of a fall through the column

	LBM lbm = sim.create_lbm(viscosity, { Acceleration{}, Acceleration{}, -g });
	BoundaryBuilder(lbm)
		.set_solid_faces({ Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Y_MAX, Face::Z_MIN, Face::Z_MAX })
		.apply();

	// the mean pressure of the liquid in cell layer z, in Pa (the lattice pressure is (rho-1)/3)
	const uint Nx = lbm.get_Nx(), Ny = lbm.get_Ny(), Nz = lbm.get_Nz();
	const auto pressure = [&](uint z) {
		double rho = 0.0;
		for(uint y = 1u; y < Ny - 1u; y++) {
			for(uint x = 1u; x < Nx - 1u; x++) rho += (double)lbm.rho[lbm.index(x, y, z)];
		}
		const double mean_rho = rho / (double)((Nx - 2u) * (Ny - 2u));
		return (double)units.si_p((float)((mean_rho - 1.0) / 3.0));
	};
	const uint z_low = Nz / 4u, z_high = 3u * Nz / 4u;
	const double expected = density.si() * g.si() * (double)(z_high - z_low) * cell.si(); // Pa

	double P1 = 0.0, P2 = 0.0;
	Runner runner(lbm);
	runner.every(0.1_s, [&](Duration t) {
		lbm.rho.read_from_device();
		const double P0 = pressure(z_low) - pressure(z_high);
		if((t > 1.0_s && converged(P2, P1, P0, 1E-5)) || t > 60.0_s) { // settled (or not settling)
			PhysicsCheck::report("hydrostatic pressure difference", fabs(P0 / expected - 1.0), 0.01);
			runner.stop();
		}
		P2 = P1;
		P1 = P0;
	});
	runner.run();
}
