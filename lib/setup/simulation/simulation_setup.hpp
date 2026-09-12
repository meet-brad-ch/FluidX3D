#pragma once
#include "setup/core/types.hpp"
#include "setup/core/unit_scale.hpp"
#include "units.hpp"
#include "setup/core/fluids.hpp"
#include "setup/boundaries/boundary_flags.hpp"

#include "setup/config/simulation_config.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/domain.hpp"
#include "setup/domain/domain_plan.hpp"
#include "setup/domain/lattice.hpp"
#include "setup/simulation/mesh_loader.hpp"
#include <optional>

extern Units units; // global units object from lbm.cpp

// Sizes the domain from a SimulationConfig, configures SI<->LBM units, creates the LBM and voxelizes the geometry.
// Call order: setup(), configure_units*(), create_lbm*(), voxelize().
class SimulationSetup {
public:
    using Results = DomainPlan; // computed by setup()

private:
    SimulationConfig config;
    Results results;
    UnitScale scale_;               // SI <-> lattice units, set by configure_units()
    float32_t lbm_u_ref_ = 0.1f;    // reference velocity in LBM units
    bool force_tracking_ = false;   // voxelize with TYPE_S|TYPE_X for ForceAnalyzer

    // gravity as LBM volume force along -gravity_axis (rho*g with the LBM density 1)
    float3 lbm_gravity_force(Acceleration gravity, Axis gravity_axis) const {
        const float32_t g = to_lbm_acceleration(gravity);
        return float3(gravity_axis == Axis::X ? -g : 0.0f, gravity_axis == Axis::Y ? -g : 0.0f, gravity_axis == Axis::Z ? -g : 0.0f);
    }

    void validate_geometry_file() {
        const string geometry_path = get_resource_path(config.geometry_filename);
        if(geometry_path.empty()) {
            const string exe_path = get_exe_path();
            const string file_type = config.use_sdf ? "SDF" : "STL";
            std::cerr << "\n";
            std::cerr << "================================================================================\n";
            std::cerr << "FATAL ERROR: " << file_type << " file not found\n";
            std::cerr << "================================================================================\n";
            std::cerr << "File: " << config.geometry_filename << "\n\n";
            std::cerr << "Searched in:\n";
#ifdef FLUIDX3D_RESOURCE_DIR
            std::cerr << "  1. " << string(FLUIDX3D_RESOURCE_DIR) << "/" << config.geometry_filename << "\n";
#endif
            std::cerr << "  2. " << exe_path << "resources/" << config.geometry_filename << "\n\n";
            std::cerr << "Please ensure the file exists in one of these directories.\n";
            std::cerr << "================================================================================\n";
            std::cerr << std::endl;
            exit(1);
        }
    }

    static SimulationConfig config_of(const Domain& domain) {
        std::optional<SimulationConfig> config;
        try {
            config.emplace(domain.config());
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        return *config;
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
    // the domain in physical units (exits with a message if its settings conflict)
    explicit SimulationSetup(const Domain& domain) : SimulationSetup(config_of(domain)) {}

    SimulationSetup(const SimulationConfig& cfg) : config(cfg) { // exits if the geometry file is not found
        std::cout.flush();
        if(config.domain_mode_ != SimulationConfig::DomainMode::DOMAIN_ONLY) {
            validate_geometry_file();
        }
        std::cout.flush();
    }

    // compute the domain size and geometry placement (exits with a message if the setup cannot be simulated)
    Results setup() {
        std::optional<DomainPlan> plan;
        try {
            plan.emplace(DomainPlanner::plan(config, lattice_memory()));
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        results = *plan;
        DomainPlanner::choose_voxelization(results, config);
        return results;
    }

    const string& get_stl_path() const { return results.stl_path; }
    const string& get_geometry_filename() const { return config.geometry_filename; }
    float32_t get_mesh_scale_factor() const { return results.lbm_reference_size / results.si_reference_size; } // cells per m

    // voxelize() marks the geometry TYPE_S|TYPE_X so ForceAnalyzer can measure forces (needs FORCE_FIELD); call before voxelize()
    SimulationSetup& enable_force_tracking() {
        force_tracking_ = true;
        return *this;
    }

    void voxelize(LBM& lbm) {
        const uchar voxel_flag = force_tracking_ ? (TYPE_S | TYPE_X) : TYPE_S;

        if(config.mirror_plane_ != SimulationConfig::MirrorPlane::NONE) { // symmetric half-model
            Mesh* mesh = load_mesh_mirrored();
            lbm.voxelize_mesh_on_device(mesh, voxel_flag);
            delete mesh;
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
    /// @param lbm_u     the reference velocity in lattice units (at most about 0.3; lower is more accurate)
    SimulationSetup& configure_units(Speed velocity, Density density, float32_t lbm_u = 0.1f) {
        lbm_u_ref_ = lbm_u;
        scale_ = UnitScale::from_reference(Length::from_si(results.si_reference_size), results.lbm_reference_size,
                                           velocity, lbm_u, density);
        units.set_m_kg_s(scale_.cell_size().si(), scale_.mass_unit().si(), scale_.time_step().si());
        return *this;
    }

    /// As configure_units(Speed, Density, float), with the density of this fluid.
    SimulationSetup& configure_units(Speed velocity, const FluidProperties& fluid, float32_t lbm_u = 0.1f) {
        return configure_units(velocity, fluid.density, lbm_u);
    }

    /// The scale set by configure_units().
    const UnitScale& unit_scale() const { return scale_; }

    /// @name SI to lattice units, with the scale of configure_units()
    /// @{
    float32_t to_lbm_viscosity(KinematicViscosity nu) const { return scale_.viscosity(nu); }
    float32_t to_lbm_velocity(Speed u) const { return scale_.velocity(u); }
    float32_t to_lbm_length(Length x) const { return scale_.length(x); } ///< in cells
    float32_t to_lbm_acceleration(Acceleration a) const { return scale_.acceleration(a); } ///< for gravity also the volume force rho*g (lattice density 1)
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

    /// An LBM with heat transport and buoyancy (TEMPERATURE and VOLUME_FORCE); gravity acts along -gravity_axis.
    LBM create_lbm_thermal(KinematicViscosity viscosity,
                           KinematicViscosity thermal_diffusivity,
                           ThermalExpansion thermal_expansion,
                           Acceleration gravity,
                           Axis gravity_axis = Axis::Z) {
        const float32_t lbm_nu = to_lbm_viscosity(viscosity);
        const float32_t lbm_alpha = to_lbm_viscosity(thermal_diffusivity); // same conversion as viscosity

        // thermal expansion: beta_lbm = beta_si * T_si / T_lbm with a 300 K reference and T_lbm ~ 1
        const float32_t T_ref_si = 300.0f;
        const float32_t T_ref_lbm = 1.0f;
        const float32_t lbm_beta = thermal_expansion.si() * T_ref_si / T_ref_lbm;

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
        const float32_t lbm_nu = to_lbm_viscosity(viscosity);
        const float32_t lbm_sigma = scale_.surface_tension(surface_tension);
        const float3 f = lbm_gravity_force(gravity, gravity_axis);
        return LBM(results.Nx, results.Ny, results.Nz, lbm_nu, f.x, f.y, f.z, lbm_sigma);
    }

    /// As create_lbm_surface() above, with this fluid's viscosity.
    LBM create_lbm_surface(const FluidProperties& fluid,
                           Acceleration gravity = 9.81_mps2,
                           SurfaceTension surface_tension = {},
                           Axis gravity_axis = Axis::Z) {
        return create_lbm_surface(fluid.kinematic_viscosity, gravity, surface_tension, gravity_axis);
    }

    /// @brief An LBM with particles (PARTICLES), its viscosity from a Reynolds number over the domain width Nx and lbm_u.
    /// @param particle_density the particles' density relative to the fluid's
    LBM create_lbm_particles_reynolds(float32_t reynolds,
                                       uint32_t particle_count,
                                       float32_t particle_density = 1.0f,
                                       Acceleration gravity = {},
                                       Axis gravity_axis = Axis::Z,
                                       float32_t lbm_u = 0.1f) {
        const float32_t lbm_nu = units.nu_from_Re(reynolds, (float32_t)results.Nx, lbm_u);
        lbm_u_ref_ = lbm_u;
        const float3 f = lbm_gravity_force(gravity, gravity_axis);
        return LBM(results.Nx, results.Ny, results.Nz, lbm_nu, f.x, f.y, f.z,
                   particle_count, particle_density);
    }

    // ========================================================================
    // Meshes
    // ========================================================================

    MeshLoader::MeshParams get_mesh_params() const {
        MeshLoader::MeshParams params;
        params.stl_path = results.stl_path;
        params.center_lbm = results.center_lbm;
        params.rotation_matrix = results.rotation_matrix;
        params.lbm_reference_size = results.lbm_reference_size;
        return params;
    }

    // full mesh from a half model, mirrored across the configured plane (caller deletes)
    Mesh* load_mesh_mirrored() {
        return MeshLoader::load_mirrored(get_mesh_params(), config.mirror_plane_);
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
