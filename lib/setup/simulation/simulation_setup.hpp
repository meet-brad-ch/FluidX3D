#pragma once
#include "setup/core/types.hpp"
#include "setup/core/unit_scale.hpp"
#include "setup/core/temperature_scale.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include "setup/core/fluids.hpp"
#include "setup/boundaries/boundary_flags.hpp"

#include "setup/core/setup_error.hpp"
#include "setup/domain/domain.hpp"
#include "setup/domain/domain_plan.hpp"
#include "setup/domain/lattice.hpp"
#include "setup/simulation/mesh_loader.hpp"
#include <optional>

extern Units units; // global units object from lbm.cpp

// Sizes the Domain, configures SI<->LBM units, creates the LBM and voxelizes the model.
// Call order: setup(), configure_units*(), create_lbm*(), voxelize().
class SimulationSetup {
public:
    using Results = DomainPlan; // computed by setup()

private:
    Domain domain_;
    Results results;
    UnitScale scale_;               // SI <-> lattice units, set by configure_units()
    std::optional<TemperatureScale> temperature_scale_; // set by configure_temperatures()
    float32_t lbm_u_ref_ = 0.1f;    // reference velocity in LBM units
    bool force_tracking_ = false;   // voxelize with TYPE_S|TYPE_X for ForceAnalyzer

    // gravity as LBM volume force along -gravity_axis (rho*g with the LBM density 1)
    float3 lbm_gravity_force(Acceleration gravity, Axis gravity_axis) const {
        return to_lbm_acceleration(AccelerationVector{ gravity_axis == Axis::X ? -gravity : Acceleration{},
                                                       gravity_axis == Axis::Y ? -gravity : Acceleration{},
                                                       gravity_axis == Axis::Z ? -gravity : Acceleration{} });
    }

    void validate_geometry_file() {
        const Model& model = *domain_.model();
        const string geometry_path = get_resource_path(model.file());
        if(geometry_path.empty()) {
            const string exe_path = get_exe_path();
            const string file_type = model.is_sdf() ? "SDF" : "STL";
            std::cerr << "\n";
            std::cerr << "================================================================================\n";
            std::cerr << "FATAL ERROR: " << file_type << " file not found\n";
            std::cerr << "================================================================================\n";
            std::cerr << "File: " << model.file() << "\n\n";
            std::cerr << "Searched in:\n";
#ifdef FLUIDX3D_RESOURCE_DIR
            std::cerr << "  1. " << string(FLUIDX3D_RESOURCE_DIR) << "/" << model.file() << "\n";
#endif
            std::cerr << "  2. " << exe_path << "resources/" << model.file() << "\n\n";
            std::cerr << "Please ensure the file exists in one of these directories.\n";
            std::cerr << "================================================================================\n";
            std::cerr << std::endl;
            exit(1);
        }
    }

    static const Domain& validated(const Domain& domain) {
        try {
            domain.validate();
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        return domain;
    }

    // device memory per cell of this example's lattice (it depends on its defines.hpp)
    static LatticeMemory lattice_memory() {
#ifdef D2Q9
        return { bytes_per_cell_device(), 2u };
#else
        return { bytes_per_cell_device(), 3u };
#endif
    }

public:
    // the domain in physical units (exits with a message if its settings conflict or its geometry file is not found)
    explicit SimulationSetup(const Domain& domain) : domain_(validated(domain)) {
        if(domain_.model()) validate_geometry_file();
    }

    // compute the domain size and geometry placement (exits with a message if the setup cannot be simulated)
    Results setup() {
        std::optional<DomainPlan> plan;
        try {
            plan.emplace(DomainPlanner::plan(domain_, lattice_memory()));
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        results = *plan;
        DomainPlanner::choose_voxelization(results, domain_);
        return results;
    }

    /// The model's file in resources/ (empty for Domain::box()), for parts that move with it.
    string model_file() const { return domain_.model() ? domain_.model()->file() : string(); }

    // voxelize() marks the geometry TYPE_S|TYPE_X so ForceAnalyzer can measure forces (needs FORCE_FIELD); call before voxelize()
    SimulationSetup& enable_force_tracking() {
        force_tracking_ = true;
        return *this;
    }

    void voxelize(LBM& lbm) {
        const uchar voxel_flag = force_tracking_ ? (TYPE_S | TYPE_X) : TYPE_S;

        if(results.mirror) { // symmetric half model
            const std::unique_ptr<Mesh> mesh = MeshLoader::load_mirrored(results, *results.mirror);
            lbm.voxelize_mesh_on_device(mesh.get(), voxel_flag);
            return;
        }

        if(results.voxelize_sdf) {
            lbm.voxelize_sdf(results.voxelization_path, results.center_lbm, results.rotation_matrix, results.voxel_size, voxel_flag);
        } else {
            lbm.voxelize_stl(results.voxelization_path, results.center_lbm, results.rotation_matrix, results.voxel_size, voxel_flag);
        }
    }

    const Results& get_results() const { return results; }

    // ========================================================================
    // Units (call after setup())
    // ========================================================================

    /// @brief The scale between SI and lattice units, from the reference velocity and the fluid's density.
    ///
    /// Also sets the core's global units (used by the builders, the graphics and the file output).
    /// @param velocity  the reference velocity, usually the fastest in the flow
    /// @param density   the fluid's density: lattice density 1
    /// @param mach      how compressible the simulation makes the flow; it sets the time step (see LatticeMach)
    SimulationSetup& configure_units(Speed velocity, Density density, LatticeMach mach = LatticeMach()) {
        lbm_u_ref_ = mach.lattice_speed();
        scale_ = UnitScale::from_reference(Length::from_si(results.si_reference_size), results.lbm_reference_size,
                                           velocity, lbm_u_ref_, density);
        units.set_m_kg_s(scale_.cell_size().si(), scale_.mass_unit().si(), scale_.time_step().si());
        print_info("Lattice Mach number " + to_string(mach.value(), 4u) + " (compressibility error ~" +
                   to_string(100.0f * sq(mach.value()), 2u) + " %)");
        return *this;
    }

    /// As configure_units(Speed, Density, LatticeMach), with the density of this fluid.
    SimulationSetup& configure_units(Speed velocity, const FluidProperties& fluid, LatticeMach mach = LatticeMach()) {
        return configure_units(velocity, fluid.density, mach);
    }

    /// The scale set by configure_units().
    const UnitScale& unit_scale() const { return scale_; }

    /// @brief The scale of the temperatures (TEMPERATURE): cold and hot become the lattice temperatures 0.5 and 1.5.
    ///
    /// Call before create_lbm_thermal(), which converts the fluid's thermal expansion with it.
    SimulationSetup& configure_temperatures(Temperature cold, Temperature hot) {
        if(!(hot > cold)) print_error("SimulationSetup::configure_temperatures(): the hot temperature must be above the cold one");
        temperature_scale_ = TemperatureScale::between(cold, hot);
        return *this;
    }

    /// The scale set by configure_temperatures(), for ThermalBuilder; exits with a message if it is not set.
    const TemperatureScale& temperature_scale() const {
        if(!temperature_scale_) print_error("SimulationSetup: call configure_temperatures() before creating a thermal LBM");
        return *temperature_scale_;
    }

    /// @name SI to lattice units, with the scale of configure_units()
    /// @{
    float32_t to_lbm_viscosity(KinematicViscosity nu) const { return scale_.viscosity(nu); }
    float32_t to_lbm_velocity(Speed u) const { return scale_.velocity(u); }
    float32_t to_lbm_length(Length x) const { return scale_.length(x); } ///< in cells
    float32_t to_lbm_acceleration(Acceleration a) const { return scale_.acceleration(a); } ///< for gravity also the volume force rho*g (lattice density 1)
    /// A body force per mass as the LBM's volume force (lattice density 1), e.g. for LBM::set_f()
    float3 to_lbm_acceleration(const AccelerationVector& a) const {
        return float3(scale_.acceleration(a.x), scale_.acceleration(a.y), scale_.acceleration(a.z));
    }
    uint64_t to_lbm_timesteps(Duration t) const { return scale_.time_steps(t); }
    /// @}

    // ========================================================================
    // LBM creation (call after configure_units())
    // ========================================================================

    /// An LBM of the planned grid with this viscosity.
    LBM create_lbm(KinematicViscosity viscosity) {
        return LBM(results.Nx, results.Ny, results.Nz, to_lbm_viscosity(viscosity));
    }

    /// An LBM of the planned grid with this fluid's viscosity.
    LBM create_lbm(const FluidProperties& fluid) {
        return create_lbm(fluid.kinematic_viscosity);
    }

    /// An LBM driven by a body force per mass (VOLUME_FORCE): gravity, or a pressure gradient over the density.
    LBM create_lbm(KinematicViscosity viscosity, const AccelerationVector& body_force) {
        const float3 f = to_lbm_acceleration(body_force);
        return LBM(results.Nx, results.Ny, results.Nz, to_lbm_viscosity(viscosity), f.x, f.y, f.z);
    }

    /// An LBM with heat transport and buoyancy (TEMPERATURE and VOLUME_FORCE); gravity acts along -gravity_axis.
    LBM create_lbm_thermal(KinematicViscosity viscosity,
                           KinematicViscosity thermal_diffusivity,
                           ThermalExpansion thermal_expansion,
                           Acceleration gravity,
                           Axis gravity_axis = Axis::Z) {
        const float32_t lbm_nu = to_lbm_viscosity(viscosity);
        const float32_t lbm_alpha = to_lbm_viscosity(thermal_diffusivity); // same conversion as viscosity

        const float32_t lbm_beta = temperature_scale().expansion(thermal_expansion); // the buoyancy of the physical temperatures

        const float3 f = lbm_gravity_force(gravity, gravity_axis);
        return LBM(results.Nx, results.Ny, results.Nz, 1u, 1u, 1u,
                   lbm_nu, f.x, f.y, f.z, 0.0f, lbm_alpha, lbm_beta);
    }

    /// As create_lbm_thermal() above, with this fluid's properties.
    LBM create_lbm_thermal(const FluidProperties& fluid, Acceleration gravity = 9.81_mps2, Axis gravity_axis = Axis::Z) {
        return create_lbm_thermal(fluid.kinematic_viscosity, fluid.thermal_diffusivity, fluid.thermal_expansion, gravity, gravity_axis);
    }

    /// A free surface LBM (SURFACE and VOLUME_FORCE); gravity acts along -gravity_axis, surface tension 0 is none.
    LBM create_lbm_surface(KinematicViscosity viscosity,
                           Acceleration gravity = 9.81_mps2,
                           SurfaceTension surface_tension = {},
                           Axis gravity_axis = Axis::Z) {
        const float3 f = lbm_gravity_force(gravity, gravity_axis);
        return LBM(results.Nx, results.Ny, results.Nz, to_lbm_viscosity(viscosity), f.x, f.y, f.z, scale_.surface_tension(surface_tension));
    }

    /// A free surface LBM (SURFACE and VOLUME_FORCE) with any body force per mass, such as gravity on a slope.
    LBM create_lbm_surface(KinematicViscosity viscosity, const AccelerationVector& body_force, SurfaceTension surface_tension) {
        const float3 f = to_lbm_acceleration(body_force);
        return LBM(results.Nx, results.Ny, results.Nz, to_lbm_viscosity(viscosity), f.x, f.y, f.z, scale_.surface_tension(surface_tension));
    }

    /// As create_lbm_surface() above, with this fluid's viscosity.
    LBM create_lbm_surface(const FluidProperties& fluid,
                           Acceleration gravity = 9.81_mps2,
                           SurfaceTension surface_tension = {},
                           Axis gravity_axis = Axis::Z) {
        return create_lbm_surface(fluid.kinematic_viscosity, gravity, surface_tension, gravity_axis);
    }

    /// @brief An LBM with particles (PARTICLES), its viscosity from a Reynolds number of the reference velocity over the
    /// domain width (Nx cells).
    /// @param particle_density the particles' density relative to the fluid's
    LBM create_lbm_particles_reynolds(float32_t reynolds,
                                       uint32_t particle_count,
                                       float32_t particle_density = 1.0f,
                                       Acceleration gravity = {},
                                       Axis gravity_axis = Axis::Z) {
        const float32_t lbm_nu = units.nu_from_Re(reynolds, (float32_t)results.Nx, lbm_u_ref_);
        const float3 f = lbm_gravity_force(gravity, gravity_axis);
        return LBM(results.Nx, results.Ny, results.Nz, lbm_nu, f.x, f.y, f.z,
                   particle_count, particle_density);
    }

    // ========================================================================
    // Reynolds number (after setup() and configure_units())
    // ========================================================================

    /// The Reynolds number of the reference length and velocity with this viscosity.
    float32_t reynolds_number(KinematicViscosity viscosity) const {
        return results.si_reference_size * scale_.si_velocity(lbm_u_ref_).si() / viscosity.si();
    }

    /// Prints the Reynolds number of the reference length and velocity with this viscosity.
    void print_reynolds_number(KinematicViscosity viscosity) const {
        print_info("Re = " + to_string(static_cast<uint32_t>(reynolds_number(viscosity))));
    }

    void print_reynolds_number(const FluidProperties& fluid) const {
        print_reynolds_number(fluid.kinematic_viscosity);
    }
};
