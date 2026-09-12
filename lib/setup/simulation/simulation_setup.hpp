#pragma once
#include "setup/core/types.hpp"
#include "setup/core/unit_scale.hpp"
#include "units.hpp"
#include "setup/core/fluids.hpp"
#include "setup/boundaries/boundary_flags.hpp"

#include "setup/config/simulation_config.hpp"
#include "setup/domain/geometry_scaler.hpp"
#include "setup/domain/lattice.hpp"
#include "setup/sdf/sdf_generator.hpp"
#include "setup/simulation/mesh_loader.hpp"
#include <optional>

extern Units units; // global units object from lbm.cpp

// Sizes the domain from a SimulationConfig, configures SI<->LBM units, creates the LBM and voxelizes the geometry.
// Call order: setup(), configure_units*(), create_lbm*(), voxelize().
class SimulationSetup {
public:
    struct Results { // computed by setup()
        float3 stl_size_si{};           // STL dimensions in m
        uint32_t Nx{}, Ny{}, Nz{};      // domain size in cells
        uint3 base_grid{};              // mesh size in cells without clearances (SDF resolution)
        float3 center_lbm{};            // geometry center in cells
        float3x3 rotation_matrix{};
        float32_t lbm_reference_size{}; // reference length in cells
        float32_t si_reference_size{};  // reference length in m
    };

private:
    SimulationConfig config;
    Results results;
    string resolved_geometry_path;  // geometry file to voxelize (STL, or the cached SDF)
    string original_stl_path_;
    UnitScale scale_;               // SI <-> lattice units, set by configure_units()
    float32_t lbm_u_ref_ = 0.1f;    // reference velocity in LBM units
    bool force_tracking_ = false;   // voxelize with TYPE_S|TYPE_X for ForceAnalyzer

    // gravity as LBM volume force along -gravity_axis (rho*g with the LBM density 1)
    float3 lbm_gravity_force(float32_t si_gravity, Axis gravity_axis) const {
        const float32_t g = to_lbm_acceleration(si_gravity);
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

    float3x3 create_rotation_matrix() {
        const float3x3 Rx = float3x3(float3(1, 0, 0), radians(config.rotation_x));
        const float3x3 Ry = float3x3(float3(0, 1, 0), radians(config.rotation_y));
        const float3x3 Rz = float3x3(float3(0, 0, 1), radians(config.rotation_z));
        float3x3 base_rotation = Rz * Ry * Rx;
        if(config.angle_of_attack_deg_ != 0.0f) { // angle of attack: additional pitch
            const float3x3 R_aoa = float3x3(float3(1, 0, 0), radians(config.angle_of_attack_deg_));
            return R_aoa * base_rotation;
        }
        return base_rotation;
    }

    // device memory per cell of this example's lattice (it depends on its defines.hpp)
    static LatticeMemory lattice_memory() {
#ifdef D2Q9
        return { bytes_per_cell_device(), 2u };
#else
        return { bytes_per_cell_device(), 3u };
#endif
    }

    // grid with this aspect ratio that fills the VRAM budget
    uint3 grid_for_vram(const float3& aspect) const {
        const GridSize grid = grid_for_memory(aspect.x, aspect.y, aspect.z, config.vram_mb, lattice_memory());
        return uint3(grid.x, grid.y, grid.z);
    }

    uint32_t get_reference_axis_dimension(const uint3& dims) {
        switch(config.reference_axis) {
            case SimulationConfig::ReferenceAxis::X: return dims.x;
            case SimulationConfig::ReferenceAxis::Y: return dims.y;
            case SimulationConfig::ReferenceAxis::Z: return dims.z;
            case SimulationConfig::ReferenceAxis::MAX: return max(max(dims.x, dims.y), dims.z);
            case SimulationConfig::ReferenceAxis::MIN: return min(min(dims.x, dims.y), dims.z);
            default: return dims.y;
        }
    }

    // ASPECT_RATIO mode: domain from aspect ratio and VRAM budget, geometry scaled to fit
    Results setup_aspect_ratio_mode() {
        const float3 aspect(config.aspect_x_, config.aspect_y_, config.aspect_z_);
        const uint3 lbm_N = grid_for_vram(aspect);

        results.Nx = lbm_N.x;
        results.Ny = lbm_N.y;
        results.Nz = lbm_N.z;

        const uint32_t ref_axis_size = get_reference_axis_dimension(lbm_N);
        results.lbm_reference_size = config.geometry_scale_ * (float32_t)ref_axis_size;
        results.si_reference_size = 1.0f; // no SI size in this mode; configure_units_with_length() sets it
        results.rotation_matrix = create_rotation_matrix();
        resolved_geometry_path = get_resource_path(config.geometry_filename);

        float3 center;
        if(config.has_pmin_offset_) {
            // load the mesh for its bounding box after rotation
            Mesh* mesh = read_stl(resolved_geometry_path, 1.0f, results.rotation_matrix);
            const float3 mesh_size = mesh->get_bounding_box_size();
            const float32_t mesh_ref_dim = get_reference_dimension(mesh_size);
            const float32_t scale = results.lbm_reference_size / mesh_ref_dim; // cells per mesh unit
            const float3 half_size = 0.5f * scale * mesh_size;
            results.base_grid = uint3(
                (uint32_t)(scale * mesh_size.x + 0.5f),
                (uint32_t)(scale * mesh_size.y + 0.5f),
                (uint32_t)(scale * mesh_size.z + 0.5f)
            );
            delete mesh;

            // X centered in the domain, Y/Z from the pmin offset
            center.x = 0.5f * (float32_t)lbm_N.x;
            center.y = config.pmin_offset_ratio_.y * results.lbm_reference_size + half_size.y;
            center.z = config.pmin_offset_ratio_.z * results.lbm_reference_size + half_size.z;
        } else {
            const float3 domain_center = 0.5f * float3((float32_t)lbm_N.x, (float32_t)lbm_N.y, (float32_t)lbm_N.z);
            const float3 offset(config.center_offset_x_, config.center_offset_y_, config.center_offset_z_);
            center = domain_center + offset * results.lbm_reference_size;
            results.base_grid = uint3( // estimate from the geometry scale (uniform assumption)
                (uint32_t)(config.geometry_scale_ * (float32_t)lbm_N.x),
                (uint32_t)(config.geometry_scale_ * (float32_t)lbm_N.y),
                (uint32_t)(config.geometry_scale_ * (float32_t)lbm_N.z)
            );
        }
        results.center_lbm = center;

        print_info("Geometry: " + config.geometry_filename + " (aspect ratio mode)");
        print_info("Domain: Nx=" + to_string(results.Nx) + ", Ny=" + to_string(results.Ny) + ", Nz=" + to_string(results.Nz));
        print_info("Geometry scale: " + to_string(config.geometry_scale_ * 100.0f, 1u) + "% of reference axis");
        print_info("LBM reference size: " + to_string(results.lbm_reference_size, 1u) + " cells");
        print_info("VRAM usage: ~" + to_string(config.vram_mb) + " MB");
        if(config.angle_of_attack_deg_ != 0.0f) {
            print_info("Angle of attack: " + to_string(config.angle_of_attack_deg_, 1u) + " deg");
        }
        return results;
    }

    float32_t get_reference_dimension(const float3& size) {
        switch(config.reference_axis) {
            case SimulationConfig::ReferenceAxis::X: return size.x;
            case SimulationConfig::ReferenceAxis::Y: return size.y;
            case SimulationConfig::ReferenceAxis::Z: return size.z;
            case SimulationConfig::ReferenceAxis::MAX: return fmax(fmax(size.x, size.y), size.z);
            case SimulationConfig::ReferenceAxis::MIN: return fmin(fmin(size.x, size.y), size.z);
            default: return size.y;
        }
    }

    // DOMAIN_ONLY mode: no geometry, domain from SI dimensions and VRAM budget
    Results setup_domain_only_mode() {
        const float max_dim = fmax(fmax(config.domain_size_x_m_, config.domain_size_y_m_), config.domain_size_z_m_);
        const float3 aspect(config.domain_size_x_m_ / max_dim, config.domain_size_y_m_ / max_dim, config.domain_size_z_m_ / max_dim);
        const uint3 lbm_N = grid_for_vram(aspect);

        results.Nx = lbm_N.x;
        results.Ny = lbm_N.y;
        results.Nz = lbm_N.z;
        results.si_reference_size = max_dim;
        results.lbm_reference_size = (float32_t)max(max(lbm_N.x, lbm_N.y), lbm_N.z);
        results.stl_size_si = float3(config.domain_size_x_m_, config.domain_size_y_m_, config.domain_size_z_m_);
        results.center_lbm = 0.5f * float3((float32_t)lbm_N.x, (float32_t)lbm_N.y, (float32_t)lbm_N.z);
        results.base_grid = uint3(0, 0, 0);
        results.rotation_matrix = float3x3(1.0f);

        print_info("Domain-only simulation (no geometry)");
        print_info("SI dimensions: " + to_string(config.domain_size_x_m_) + "m x " +
                  to_string(config.domain_size_y_m_) + "m x " + to_string(config.domain_size_z_m_) + "m");
        print_info("Domain: Nx=" + to_string(results.Nx) + ", Ny=" + to_string(results.Ny) + ", Nz=" + to_string(results.Nz));
        print_info("VRAM usage: ~" + to_string(config.vram_mb) + " MB");
        return results;
    }

public:
    SimulationSetup(const SimulationConfig& cfg) : config(cfg) { // exits if the geometry file is not found
        std::cout.flush();
        if(config.domain_mode_ != SimulationConfig::DomainMode::DOMAIN_ONLY) {
            validate_geometry_file();
        }
        std::cout.flush();
    }

    // compute the domain size and geometry placement
    Results setup() {
        if(config.domain_mode_ == SimulationConfig::DomainMode::DOMAIN_ONLY) {
            return setup_domain_only_mode();
        }

        original_stl_path_ = get_resource_path(config.geometry_filename);

        if(config.domain_mode_ == SimulationConfig::DomainMode::ASPECT_RATIO) {
            return setup_aspect_ratio_mode();
        }

        // GEOMETRY_BASED mode (default): domain from STL + clearances
        GeometryScaler::ReferenceAxis scaler_axis;
        switch(config.reference_axis) {
            case SimulationConfig::ReferenceAxis::X: scaler_axis = GeometryScaler::ReferenceAxis::X; break;
            case SimulationConfig::ReferenceAxis::Y: scaler_axis = GeometryScaler::ReferenceAxis::Y; break;
            case SimulationConfig::ReferenceAxis::Z: scaler_axis = GeometryScaler::ReferenceAxis::Z; break;
            case SimulationConfig::ReferenceAxis::MAX: scaler_axis = GeometryScaler::ReferenceAxis::MAX; break;
            case SimulationConfig::ReferenceAxis::MIN: scaler_axis = GeometryScaler::ReferenceAxis::MIN; break;
            default: scaler_axis = GeometryScaler::ReferenceAxis::Y; break;
        }

        GeometryScaler::Clearances clearances; // included in the VRAM budget
        clearances.bottom_m = config.bottom_clearance_m;
        clearances.top_m = config.top_clearance_m;
        clearances.side_m = config.side_clearance_m;

        std::optional<GeometryScaler> scaler;
        try {
            if(config.resolution_mode_ == SimulationConfig::ResolutionMode::VOXEL_SIZE) {
                scaler.emplace(original_stl_path_, config.voxel_size_m_, config.max_vram_mb_, clearances, lattice_memory(), scaler_axis);
            } else {
                scaler.emplace(original_stl_path_, config.vram_mb, clearances, lattice_memory(), scaler_axis);
            }
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }

        results.base_grid = scaler->get_stl_size_cells();

        if(!config.use_sdf && config.geometry_filename.find(".stl") != string::npos) {
            // convert the STL to an SDF at the base grid resolution (cached), for smoother voxelization
            SDFGenerator sdf_gen;
            sdf_gen.set_cache_dir("resources/sdf_cache/")
                   .enable_cache(true)
                   .set_fix_mesh(config.fix_mesh_)
                   .set_verbose(true);
            std::string sdf_path = sdf_gen.generate(original_stl_path_, results.base_grid.x, results.base_grid.y, results.base_grid.z, 1);
            if(!sdf_path.empty()) {
                resolved_geometry_path = sdf_path;
                config.use_sdf = true;
                print_info("Using SDF voxelization (cached: " + sdf_path + ")");
            } else {
                resolved_geometry_path = get_resource_path(config.geometry_filename);
                print_info("SDF caching failed, falling back to STL voxelization");
            }
        } else if(config.use_sdf && config.geometry_filename.find("sdf_cache") != string::npos) {
            resolved_geometry_path = config.geometry_filename;
        } else {
            resolved_geometry_path = get_resource_path(config.geometry_filename);
        }

        results.stl_size_si = scaler->get_stl_size_meters();
        results.lbm_reference_size = scaler->get_reference_size_cells();
        results.si_reference_size = scaler->get_reference_size_meters();

        const uint3 domain_size = scaler->calculate_domain_size(clearances);
        results.Nx = domain_size.x;
        results.Ny = domain_size.y;
        results.Nz = domain_size.z;
        results.center_lbm = scaler->calculate_center(domain_size, clearances);
        results.rotation_matrix = create_rotation_matrix();

        print_info("Geometry: " + config.geometry_filename);
        print_info("Dimensions: X=" + to_string(results.stl_size_si.x) + "m, Y=" + to_string(results.stl_size_si.y) + "m, Z=" + to_string(results.stl_size_si.z) + "m");
        print_info("Scale: " + to_string(scaler->get_scale_factor()) + " m/cell");
        print_info("Base grid (mesh only): " + to_string(results.base_grid.x) + " x " + to_string(results.base_grid.y) + " x " + to_string(results.base_grid.z));
        print_info("Domain: Nx=" + to_string(results.Nx) + ", Ny=" + to_string(results.Ny) + ", Nz=" + to_string(results.Nz));
        print_info("VRAM usage: ~" + to_string(config.vram_mb) + " MB");
        return results;
    }

    const string& get_stl_path() const { return original_stl_path_; }
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

        // voxelize_stl/voxelize_sdf expect the size of the longest dimension
        float voxel_size;
        if(config.domain_mode_ == SimulationConfig::DomainMode::ASPECT_RATIO) {
            voxel_size = results.lbm_reference_size; // geometry_scale * domain reference axis
        } else {
            voxel_size = fmax(fmax((float)results.base_grid.x, (float)results.base_grid.y), (float)results.base_grid.z);
        }

        if(config.use_sdf) {
            lbm.voxelize_sdf(resolved_geometry_path, results.center_lbm, results.rotation_matrix, voxel_size, voxel_flag);
        } else {
            lbm.voxelize_stl(resolved_geometry_path, results.center_lbm, results.rotation_matrix, voxel_size, voxel_flag);
        }
    }

    const Results& get_results() const { return results; }

    // ========================================================================
    // Units (call after setup())
    // ========================================================================

    // reference velocity in m/s, fluid density in kg/m³; lbm_u is the reference velocity in LBM units.
    // Also sets the core's global units (used by the builders, the graphics and the file output).
    SimulationSetup& configure_units(float32_t si_velocity, float32_t si_density = 1.225f, float32_t lbm_u = 0.1f) {
        lbm_u_ref_ = lbm_u;
        scale_ = UnitScale::from_reference(Length::from_si(results.si_reference_size), results.lbm_reference_size,
                                           Speed::from_si(si_velocity), lbm_u, Density::from_si(si_density));
        units.set_m_kg_s(scale_.cell_size().si(), scale_.mass_unit().si(), scale_.time_step().si());
        return *this;
    }

    const UnitScale& unit_scale() const { return scale_; }

    SimulationSetup& configure_units(float32_t si_velocity, const FluidProperties& fluid, float32_t lbm_u = 0.1f) {
        return configure_units(si_velocity, fluid.density, lbm_u);
    }

    // as configure_units(), with the reference length in m (for ASPECT_RATIO mode, which has no SI size)
    SimulationSetup& configure_units_with_length(float32_t si_reference_length, float32_t si_velocity,
                                                  float32_t si_density = 1.225f, float32_t lbm_u = 0.1f) {
        results.si_reference_size = si_reference_length;
        return configure_units(si_velocity, si_density, lbm_u);
    }

    SimulationSetup& configure_units_with_length(float32_t si_reference_length, float32_t si_velocity,
                                                  const FluidProperties& fluid, float32_t lbm_u = 0.1f) {
        return configure_units_with_length(si_reference_length, si_velocity, fluid.density, lbm_u);
    }

    float32_t to_lbm_viscosity(float32_t si_viscosity) const { return scale_.viscosity(KinematicViscosity::from_si(si_viscosity)); } // m²/s
    float32_t to_lbm_velocity(float32_t si_velocity) const { return scale_.velocity(Speed::from_si(si_velocity)); }              // m/s
    float32_t to_lbm_length(float32_t si_length) const { return scale_.length(Length::from_si(si_length)); }                     // m
    float32_t to_lbm_acceleration(float32_t si_acceleration) const { return scale_.acceleration(Acceleration::from_si(si_acceleration)); } // m/s²; for gravity also the volume force rho*g (LBM density 1)
    uint64_t to_lbm_timesteps(float32_t si_seconds) const { return scale_.time_steps(Duration::from_si(si_seconds)); }          // s

    // ========================================================================
    // LBM creation (call after configure_units())
    // ========================================================================

    LBM create_lbm(float32_t si_kinematic_viscosity) { // m²/s
        return LBM(results.Nx, results.Ny, results.Nz, to_lbm_viscosity(si_kinematic_viscosity));
    }

    LBM create_lbm(const FluidProperties& fluid) {
        return create_lbm(fluid.kinematic_viscosity);
    }

    // TEMPERATURE + VOLUME_FORCE: viscosity and thermal diffusivity in m²/s, expansion in 1/K, gravity in m/s² along -gravity_axis
    LBM create_lbm_thermal(float32_t si_kinematic_viscosity,
                           float32_t si_thermal_diffusivity,
                           float32_t si_thermal_expansion,
                           float32_t si_gravity,
                           Axis gravity_axis = Axis::Z) {
        const float32_t lbm_nu = to_lbm_viscosity(si_kinematic_viscosity);
        const float32_t lbm_alpha = to_lbm_viscosity(si_thermal_diffusivity); // same conversion as viscosity

        // thermal expansion: beta_lbm = beta_si * T_si / T_lbm with a 300 K reference and T_lbm ~ 1
        const float32_t T_ref_si = 300.0f;
        const float32_t T_ref_lbm = 1.0f;
        const float32_t lbm_beta = si_thermal_expansion * T_ref_si / T_ref_lbm;

        const float3 f = lbm_gravity_force(si_gravity, gravity_axis);
        return LBM(results.Nx, results.Ny, results.Nz, 1u, 1u, 1u,
                   lbm_nu, f.x, f.y, f.z, 0.0f, lbm_alpha, lbm_beta);
    }

    LBM create_lbm_thermal(const FluidProperties& fluid, float32_t si_gravity = 9.81f, Axis gravity_axis = Axis::Z) {
        return create_lbm_thermal(fluid.kinematic_viscosity, fluid.thermal_diffusivity, fluid.thermal_expansion, si_gravity, gravity_axis);
    }

    // SURFACE + VOLUME_FORCE: viscosity in m²/s, gravity in m/s² along -gravity_axis, surface tension in N/m (0: none)
    LBM create_lbm_surface(float32_t si_kinematic_viscosity,
                           float32_t si_gravity = 9.81f,
                           float32_t si_surface_tension = 0.0f,
                           Axis gravity_axis = Axis::Z) {
        const float32_t lbm_nu = to_lbm_viscosity(si_kinematic_viscosity);
        const float32_t lbm_sigma = scale_.surface_tension(SurfaceTension::from_si(si_surface_tension));
        const float3 f = lbm_gravity_force(si_gravity, gravity_axis);
        return LBM(results.Nx, results.Ny, results.Nz, lbm_nu, f.x, f.y, f.z, lbm_sigma);
    }

    LBM create_lbm_surface(const FluidProperties& fluid,
                           float32_t si_gravity = 9.81f,
                           float32_t si_surface_tension = 0.0f,
                           Axis gravity_axis = Axis::Z) {
        return create_lbm_surface(fluid.kinematic_viscosity, si_gravity, si_surface_tension, gravity_axis);
    }

    // PARTICLES: viscosity from the Reynolds number (domain width Nx, lbm_u); particle_density relative to the fluid; gravity in m/s²
    LBM create_lbm_particles_reynolds(float32_t reynolds,
                                       uint32_t particle_count,
                                       float32_t particle_density = 1.0f,
                                       float32_t si_gravity = 0.0f,
                                       Axis gravity_axis = Axis::Z,
                                       float32_t lbm_u = 0.1f) {
        const float32_t lbm_nu = units.nu_from_Re(reynolds, (float32_t)results.Nx, lbm_u);
        lbm_u_ref_ = lbm_u;
        const float3 f = lbm_gravity_force(si_gravity, gravity_axis);
        return LBM(results.Nx, results.Ny, results.Nz, lbm_nu, f.x, f.y, f.z,
                   particle_count, particle_density);
    }

    // ========================================================================
    // Meshes
    // ========================================================================

    MeshLoader::MeshParams get_mesh_params() const {
        MeshLoader::MeshParams params;
        params.stl_path = original_stl_path_;
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

    float32_t reynolds_number(float32_t si_kinematic_viscosity) const {
        return results.si_reference_size * scale_.si_velocity(lbm_u_ref_).si() / si_kinematic_viscosity;
    }

    void print_reynolds_number(float32_t si_kinematic_viscosity) const {
        print_info("Re = " + to_string(static_cast<uint32_t>(reynolds_number(si_kinematic_viscosity))));
    }

    void print_reynolds_number(const FluidProperties& fluid) const {
        print_reynolds_number(fluid.kinematic_viscosity);
    }
};
