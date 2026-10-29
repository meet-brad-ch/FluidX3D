// Hydrostatic pressure: a closed column of glycerol at rest under gravity, against dp/dz = -rho*g.
// Passes when the pressure difference between two heights, in pascals, is within 1 % of rho*g*dz.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>

namespace {

/// @brief Measures the pressure difference between two layers of the column until it settles, and reports it
/// against rho g dz.
class PressureCheck {
public:
	/// @brief Makes the check of a column.
	/// @param sim the simulation of the column (not owned, outlives the check)
	/// @param center_xy the column's centre along x and y
	/// @param cell the cell size
	/// @param z_low the height of the lower layer
	/// @param z_high the height of the upper layer
	/// @param expected the expected pressure difference between the two layers, in Pa
	PressureCheck(Simulation& sim, si::Length center_xy, si::Length cell, si::Length z_low, si::Length z_high, double expected)
		: sim_(sim), center_xy_(center_xy), cell_(cell), z_low_(z_low), z_high_(z_high), expected_(expected) {}

	/// @brief Measures the difference; reports it and stops the run when it settled or the run took too long.
	/// @param t the simulated time
	void at(si::Duration t) {
		using namespace si::symbols;
		const FieldReader fields = sim_.fields();
		const double p_low{ layer_pressure(fields, z_low_) };
		const double p_high{ layer_pressure(fields, z_high_) };
		const double p0{ p_low - p_high };
		const bool settled{ t > settle_start && converged(p2_, p1_, p0, settle_change) };
		const bool too_long{ t > time_limit };
		if(settled || too_long) { // settled (or not settling)
			const double error{ std::fabs(p0 / expected_ - 1.0) };
			PhysicsCheck::report("hydrostatic pressure difference", error, error_limit);
			sim_.stop();
		}
		p2_ = p1_;
		p1_ = p0;
	}

	static constexpr double error_limit = 0.01;    ///< the largest relative error that passes (1 %)
	static constexpr double settle_change = 1E-5;  ///< the relative change between checks under which it settled

private:
	/// @brief Gives the mean pressure of the liquid in the layer of cells at a height.
	/// @param fields the simulation's fields
	/// @param z a height in the layer
	/// @return the mean pressure in Pa
	double layer_pressure(const FieldReader& fields, si::Length z) const {
		using namespace si::symbols;
		const si::Position point{ center_xy_, center_xy_, z };
		const si::Length layer = fields.cell_at(point).center.z; // the center of the layer containing z
		const si::Length tolerance = layer_tolerance * cell_;
		double pressure_sum{ 0.0 };
		uint32_t cells{ 0u };
		const std::uint64_t count{ fields.cell_count() };
		for(std::uint64_t n{ 0u }; n < count; ++n) {
			const FieldReader::Cell c{ fields.cell(n) };
			const si::Length off_layer = au::abs(c.center.z - layer);
			if(c.solid || off_layer > tolerance) {
				continue;
			}
			pressure_sum += c.pressure.in(Pa);
			cells++;
		}
		const double cell_count{ static_cast<double>(cells) };
		const double mean{ pressure_sum / cell_count };
		return mean;
	}

	static constexpr double layer_tolerance = 0.1;               ///< the part of a cell within which a centre is on the layer
	static constexpr si::Duration settle_start = au::seconds(1.0); ///< the time before which the column is not settled
	static constexpr si::Duration time_limit = au::seconds(60.0);  ///< the time after which the check reports anyway

	Simulation& sim_;         ///< the simulation (not owned)
	si::Length center_xy_{};  ///< the column's centre along x and y
	si::Length cell_{};       ///< the cell size
	si::Length z_low_{};      ///< the height of the lower layer
	si::Length z_high_{};     ///< the height of the upper layer
	double expected_{};       ///< the expected difference in Pa
	double p1_{ 0.0 };        ///< the difference of the last check in Pa
	double p2_{ 0.0 };        ///< the difference of the check before it in Pa
};

} // namespace

void main_setup() { // extensions: VOLUME_FORCE
	using namespace si::symbols;
	const si::Length height = 10.0 * cm;
	const si::Length width = 2.5 * cm;
	const si::Length cell = height / 64.0;                 // 16 x 16 x 64 cells
	const si::Acceleration g = 9.81 * m / (s * s);
	// glycerol at 20 °C: its viscosity damps the sound waves of the start
	const FluidProperties glycerol{ .density = 1260.0 * kg / (m * m * m), .kinematic_viscosity = 1.12E-3 * m * m / s };
	const si::Speed fall_speed = au::sqrt(g * height); // the speed of a fall through the column

	Domain domain = Domain::box(width, width, height);
	domain.cell_size(cell);
	Simulation sim(domain, glycerol, fall_speed);
	sim.set_gravity(g);

	sim.boundaries()
		.set_solid_box()
		.apply();

	const si::Length center_xy = 0.5 * width;
	const si::Length z_low = 0.25 * height;
	const si::Length z_high = 0.75 * height;
	const si::Length layer_high = sim.fields().cell_at({ center_xy, center_xy, z_high }).center.z;
	const si::Length layer_low = sim.fields().cell_at({ center_xy, center_xy, z_low }).center.z;
	const si::Pressure expected_difference = glycerol.density * g * (layer_high - layer_low);
	const double expected = expected_difference.in(Pa);

	PressureCheck check(sim, center_xy, cell, z_low, z_high, expected);
	const si::Duration check_interval = 0.1 * s;
	sim.every(check_interval, [&](si::Duration t) { check.at(t); });
	sim.run();
}
