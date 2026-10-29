#pragma once

#include "setup/core/types.hpp"
#include "setup/core/fluids.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include "lbm.hpp"

#ifndef FORCE_FIELD
#error "setup/analysis/force_analyzer.hpp needs FORCE_FIELD in defines.hpp"
#endif // FORCE_FIELD

/// @file force_analyzer.hpp
/// @brief The fluid's force on the measured solids and their drag coefficient (ForceAnalyzer).

/// @brief Measures the fluid's force on the measured solids in newtons and their drag coefficient (FORCE_FIELD
/// extension: Simulation::measure_forces() or Solid::MEASURED; Simulation::forces()).
///
/// Owned by Simulation, which makes it with the LBM; it keeps a reference to the LBM. force() and center_of_mass()
/// read the core's force field from the device; the values become physical through the simulation's converter (which
/// needs its density).
/// @code
/// using namespace si::symbols;
/// sim.forces().set_reference_area(0.389 * m * (0.288 * m)).set_reference_velocity(60.0 * m / s).set_fluid(Fluid::AIR).set_flow_direction(Axis::Y);
/// sim.every(0.01 * s, [&](si::Duration) { print_info("Cd = " + to_string((float)sim.forces().drag_coefficient(), 3u)); });
/// @endcode
class ForceAnalyzer {
public:
    /// @brief Makes the analyzer of an LBM.
    /// @param lbm the LBM with the measured solids (cells flagged TYPE_S|TYPE_X)
    /// @param converter the simulation's conversions, with its density
    ForceAnalyzer(LBM& lbm, const si::LatticeConverter& converter) : lbm_(lbm), converter_(converter) {}

    /// @brief Sets the area of the drag coefficient, usually the frontal area (default 1 m²).
    ///
    /// Ends the program with print_error() for an area that is not more than 0.
    /// @param area the reference area, more than 0
    /// @return this analyzer
    ForceAnalyzer& set_reference_area(si::Area area) {
        const si::Area none{ au::ZERO };
        if(!(area > none)) {
            print_error("ForceAnalyzer::set_reference_area(): the reference area must be more than 0 m2");
        }
        reference_area_ = area;
        return *this;
    }

    /// @brief Sets the speed of the drag coefficient, usually the free stream (default 1 m/s).
    ///
    /// Ends the program with print_error() for a speed that is not more than 0.
    /// @param velocity the reference speed, more than 0
    /// @return this analyzer
    ForceAnalyzer& set_reference_velocity(si::Speed velocity) {
        const si::Speed none{ au::ZERO };
        if(!(velocity > none)) {
            print_error("ForceAnalyzer::set_reference_velocity(): the reference speed must be more than 0 m/s");
        }
        reference_velocity_ = velocity;
        return *this;
    }

    /// @brief Sets the fluid whose density the dynamic pressure uses (default: air).
    ///
    /// Ends the program with print_error() for a density that is not more than 0.
    /// @param fluid the fluid whose density the dynamic pressure uses, its density more than 0
    /// @return this analyzer
    ForceAnalyzer& set_fluid(const FluidProperties& fluid) {
        const si::Density none{ au::ZERO };
        if(!(fluid.density > none)) {
            print_error("ForceAnalyzer::set_fluid(): the fluid's density must be more than 0 kg/m3");
        }
        fluid_density_ = fluid.density;
        return *this;
    }

    /// @brief Sets the axis along which the drag is the force component (default Y).
    /// @param axis the flow's axis
    /// @return this analyzer
    ForceAnalyzer& set_flow_direction(Axis axis) {
        flow_axis_ = axis;
        return *this;
    }

    /// @brief Gives the fluid's force on the measured solids. It reads the force field from the device.
    /// @return the force in newtons along x, y and z
    /// @throws SetupError when the converter has no density (from LatticeConverter::si_force())
    si::ForceVector force() {
        const float3 lbm_force{ lbm_.object_force(measured_flag) };
        const si::Force x{ converter_.si_force(static_cast<double>(lbm_force.x)) };
        const si::Force y{ converter_.si_force(static_cast<double>(lbm_force.y)) };
        const si::Force z{ converter_.si_force(static_cast<double>(lbm_force.z)) };
        const si::ForceVector f{ x, y, z };
        return f;
    }

    /// @brief Gives the measured solids' centre of mass. It reads the flags from the device.
    /// @return the centre in metres from the domain's origin corner
    si::Position center_of_mass() {
        const float3 cells{ lbm_.object_center_of_mass(measured_flag) }; // the core's cell coordinates: cell i's centre at i
        const double half_cell{ 0.5 };
        const si::Length x{ converter_.si_length(static_cast<double>(cells.x) + half_cell) };
        const si::Length y{ converter_.si_length(static_cast<double>(cells.y) + half_cell) };
        const si::Length z{ converter_.si_length(static_cast<double>(cells.z) + half_cell) };
        const si::Position centre{ x, y, z };
        return centre;
    }

    /// @brief Gives the drag coefficient: the force along the flow's axis over the dynamic pressure times the area.
    /// It reads the force field from the device.
    /// @return Cd = F_drag / (0.5 rho u² A) (a ratio; the setters keep rho, u and A above 0)
    /// @throws SetupError when the converter has no density (from force())
    double drag_coefficient() {
        const si::ForceVector f{ force() };
        const si::Force drag{ along_flow(f) };
        const si::Pressure q{ dynamic_pressure() };
        const si::Force reference_force{ q * reference_area_ };
        const double cd{ au::as_raw_number(drag / reference_force) };
        return cd;
    }

    /// @brief Gives the dynamic pressure of the reference speed in the fluid.
    /// @return 0.5 rho u²
    si::Pressure dynamic_pressure() const {
        const si::SpeedVariance speed_squared{ reference_velocity_ * reference_velocity_ };
        const si::Pressure q{ fluid_density_ * speed_squared * 0.5 };
        return q;
    }

private:
    static constexpr uchar measured_flag = TYPE_S | TYPE_X; ///< the flag of the measured solids' cells

    /// @brief Gives a force's component along the flow's axis.
    /// @param f the force
    /// @return the component along flow_axis_
    si::Force along_flow(const si::ForceVector& f) const {
        switch(flow_axis_) {
            case Axis::X: return f.x;
            case Axis::Y: return f.y;
            case Axis::Z: break;
        }
        return f.z;
    }

    LBM& lbm_;                                                       ///< the LBM with the measured solids (not owned)
    si::LatticeConverter converter_;                                 ///< the simulation's conversions
    si::Area reference_area_{ au::squared(au::meters)(1.0) };        ///< set_reference_area()
    si::Speed reference_velocity_{ (au::meters / au::second)(1.0) }; ///< set_reference_velocity()
    si::Density fluid_density_{ Fluid::AIR.density };                ///< set_fluid(): the fluid's density
    Axis flow_axis_ = Axis::Y;                                       ///< set_flow_direction()
};
