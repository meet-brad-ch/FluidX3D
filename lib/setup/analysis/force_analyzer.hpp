#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/unit_scale.hpp"
#include "setup/core/fluids.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "lbm.hpp"

#ifndef FORCE_FIELD
#error "setup/analysis/force_analyzer.hpp needs FORCE_FIELD in defines.hpp"
#endif // FORCE_FIELD

/// @brief The fluid's force in newtons and the drag coefficient on the measured solids (FORCE_FIELD extension:
/// Simulation::measure_forces() or Solid::MEASURED; Simulation::forces()).
/// @code
/// sim.forces().set_reference_area(0.389_m * 0.288_m).set_reference_velocity(60.0_mps).set_fluid(Fluid::AIR).set_flow_direction(Axis::Y);
/// sim.every(0.01_s, [&](Duration) { print_info("Cd = " + to_string(sim.forces().drag_coefficient(), 3u)); });
/// @endcode
class ForceAnalyzer {
public:
    /// @param lbm        the LBM with the measured solids (TYPE_S|TYPE_X)
    /// @param unit_scale the simulation's unit scale
    ForceAnalyzer(LBM& lbm, const UnitScale& unit_scale) : lbm_(lbm), units_(unit_scale) {}

    /// @brief The area of the drag coefficient, usually the frontal area (default 1 m²).
    /// @param area the area
    /// @return this analyzer
    ForceAnalyzer& set_reference_area(Area area) {
        reference_area_ = area;
        return *this;
    }

    /// @brief The velocity of the drag coefficient, usually the free stream (default 1 m/s).
    /// @param velocity the velocity
    /// @return this analyzer
    ForceAnalyzer& set_reference_velocity(Speed velocity) {
        reference_velocity_ = velocity;
        return *this;
    }

    /// @brief The fluid, for its density (default: air).
    /// @param fluid the fluid
    /// @return this analyzer
    ForceAnalyzer& set_fluid(const FluidProperties& fluid) {
        fluid_density_ = fluid.density;
        return *this;
    }

    /// @brief The drag is the force component along an axis (default Y).
    /// @param axis the flow's axis
    /// @return this analyzer
    ForceAnalyzer& set_flow_direction(Axis axis) {
        flow_axis_ = axis;
        return *this;
    }

    /// @return the fluid's force on the measured solids (reads the device)
    ForceVector force() {
        const float3 lbm_force = lbm_.object_force(measured_flag);
        return { units_.si_force(lbm_force.x), units_.si_force(lbm_force.y), units_.si_force(lbm_force.z) };
    }

    /// @return the measured solids' center of mass, in metres from the domain's origin corner
    Position center_of_mass() {
        const float3 cells = lbm_.object_center_of_mass(measured_flag); // the core's cell coordinates: cell i's center at i
        return { units_.si_length(cells.x + 0.5f), units_.si_length(cells.y + 0.5f), units_.si_length(cells.z + 0.5f) };
    }

    /// @return Cd = F_drag / (0.5*rho*u²*A); 0 when the reference area or velocity is 0
    float32_t drag_coefficient() {
        const ForceVector f = force();
        const Force drag = flow_axis_ == Axis::X ? f.x : flow_axis_ == Axis::Y ? f.y : f.z;
        const Pressure q = dynamic_pressure();
        if(q.si() * reference_area_.si() < 1e-10f) return 0.0f;
        return drag.si() / (q.si() * reference_area_.si());
    }

    /// @return 0.5*rho*u² of the reference velocity
    Pressure dynamic_pressure() const {
        return Pressure::from_si(0.5f * fluid_density_.si() * reference_velocity_.si() * reference_velocity_.si());
    }

private:
    static constexpr uchar measured_flag = TYPE_S | TYPE_X; ///< the measured solids' cells

    LBM& lbm_;                                        ///< the LBM with the measured solids
    UnitScale units_;                                 ///< the simulation's unit scale
    Area reference_area_ = Area::from_si(1.0f);       ///< set_reference_area()
    Speed reference_velocity_ = Speed::from_si(1.0f); ///< set_reference_velocity()
    Density fluid_density_ = Fluid::AIR.density;      ///< set_fluid()
    Axis flow_axis_ = Axis::Y;                        ///< set_flow_direction()
};
