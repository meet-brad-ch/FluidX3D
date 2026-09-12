#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/fluids.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "lbm.hpp"
#include "units.hpp"

extern Units units; // global units object from lbm.cpp

// Force and drag coefficient on the object voxelized with TYPE_S|TYPE_X (SimulationSetup::enable_force_tracking(),
// FORCE_FIELD extension). Reference values in SI units; units must be configured.
class ForceAnalyzer {
public:
    explicit ForceAnalyzer(LBM& lbm, uchar flag_marker = TYPE_S | TYPE_X)
        : lbm_(lbm), flag_marker_(flag_marker) {}

    /// The area of the drag coefficient, usually the frontal area.
    ForceAnalyzer& set_reference_area(Area area) {
        reference_area_ = area.si();
        return *this;
    }

    /// The velocity of the drag coefficient, usually the free stream.
    ForceAnalyzer& set_reference_velocity(Speed velocity) {
        reference_velocity_ = velocity.si();
        return *this;
    }

    /// The fluid, for its density (default: air).
    ForceAnalyzer& set_fluid(const FluidProperties& fluid) {
        fluid_density_ = fluid.density;
        return *this;
    }

    ForceAnalyzer& set_flow_direction(Axis axis) { // drag is the force component along this axis
        flow_axis_ = axis;
        return *this;
    }

    float3 get_force_lbm() { return lbm_.object_force(flag_marker_); }
    float3 get_center_of_mass_lbm() { return lbm_.object_center_of_mass(flag_marker_); } // cells

    float3 get_force_si() { // N
        const float3 lbm_force = get_force_lbm();
        return float3(units.si_F(lbm_force.x), units.si_F(lbm_force.y), units.si_F(lbm_force.z));
    }

    // Cd = F_drag/(0.5*rho*u²*A)
    float32_t get_drag_coefficient() {
        const float32_t drag_force = get_force_component(get_force_si(), flow_axis_);
        const float32_t q = get_dynamic_pressure();
        if (q * reference_area_ < 1e-10f) return 0.0f;
        return drag_force / (q * reference_area_);
    }

    float32_t get_dynamic_pressure() const { // Pa
        return 0.5f * fluid_density_ * reference_velocity_ * reference_velocity_;
    }

private:
    LBM& lbm_;
    uchar flag_marker_;

    float32_t reference_area_ = 1.0f;      // m²
    float32_t reference_velocity_ = 1.0f;  // m/s
    float32_t fluid_density_ = 1.225f;     // kg/m³
    Axis flow_axis_ = Axis::Y;

    static float32_t get_force_component(const float3& force, Axis axis) {
        switch (axis) {
            case Axis::X: return force.x;
            case Axis::Y: return force.y;
            case Axis::Z: return force.z;
            default: return force.y;
        }
    }
};
