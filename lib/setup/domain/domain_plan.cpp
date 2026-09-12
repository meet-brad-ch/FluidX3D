#include "setup/domain/domain_plan.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/geometry_scaler.hpp"
#include "setup/sdf/sdf_generator.hpp"

#include <filesystem>

DomainPlan DomainPlanner::plan(const SimulationConfig& config, LatticeMemory lattice) {
    switch(config.domain_mode_) {
        case SimulationConfig::DomainMode::DOMAIN_ONLY: return plan_domain_only(config, lattice);
        case SimulationConfig::DomainMode::ASPECT_RATIO: return plan_aspect_ratio(config, lattice);
        default: return plan_geometry_based(config, lattice);
    }
}

void DomainPlanner::choose_voxelization(DomainPlan& plan, const SimulationConfig& config) {
    plan.voxelize_sdf = config.use_sdf;
    if(config.domain_mode_ == SimulationConfig::DomainMode::DOMAIN_ONLY) return;
    if(config.domain_mode_ == SimulationConfig::DomainMode::ASPECT_RATIO) {
        plan.voxelization_path = get_resource_path(config.geometry_filename);
        return;
    }
    if(!config.use_sdf && config.geometry_filename.find(".stl") != string::npos) {
        // convert the STL to an SDF at the base grid resolution (cached), for smoother voxelization
        SDFGenerator sdf_gen;
        sdf_gen.set_cache_dir("resources/sdf_cache/")
               .enable_cache(true)
               .set_fix_mesh(config.fix_mesh_)
               .set_verbose(true);
        const std::string sdf_path = sdf_gen.generate(plan.stl_path, plan.base_grid.x, plan.base_grid.y, plan.base_grid.z, 1);
        if(!sdf_path.empty()) {
            plan.voxelization_path = sdf_path;
            plan.voxelize_sdf = true;
            print_info("Using SDF voxelization (cached: " + sdf_path + ")");
        } else {
            plan.voxelization_path = get_resource_path(config.geometry_filename);
            print_info("SDF caching failed, falling back to STL voxelization");
        }
    } else if(config.use_sdf && config.geometry_filename.find("sdf_cache") != string::npos) {
        plan.voxelization_path = config.geometry_filename;
    } else {
        plan.voxelization_path = get_resource_path(config.geometry_filename);
    }
}

// DOMAIN_ONLY mode: no geometry, domain from SI dimensions and VRAM budget
DomainPlan DomainPlanner::plan_domain_only(const SimulationConfig& config, LatticeMemory lattice) {
    DomainPlan plan;
    const float max_dim = fmax(fmax(config.domain_size_x_m_, config.domain_size_y_m_), config.domain_size_z_m_);
    const float3 aspect(config.domain_size_x_m_ / max_dim, config.domain_size_y_m_ / max_dim, config.domain_size_z_m_ / max_dim);
    const uint3 lbm_N = grid_for_vram(config, aspect, lattice);

    plan.Nx = lbm_N.x;
    plan.Ny = lbm_N.y;
    plan.Nz = lbm_N.z;
    plan.si_reference_size = max_dim;
    plan.lbm_reference_size = (float32_t)max(max(lbm_N.x, lbm_N.y), lbm_N.z);
    plan.stl_size_si = float3(config.domain_size_x_m_, config.domain_size_y_m_, config.domain_size_z_m_);
    plan.center_lbm = 0.5f * float3((float32_t)lbm_N.x, (float32_t)lbm_N.y, (float32_t)lbm_N.z);
    plan.base_grid = uint3(0, 0, 0);
    plan.rotation_matrix = float3x3(1.0f);

    print_info("Domain-only simulation (no geometry)");
    print_info("SI dimensions: " + to_string(config.domain_size_x_m_) + "m x " +
              to_string(config.domain_size_y_m_) + "m x " + to_string(config.domain_size_z_m_) + "m");
    print_info("Domain: Nx=" + to_string(plan.Nx) + ", Ny=" + to_string(plan.Ny) + ", Nz=" + to_string(plan.Nz));
    print_info("VRAM usage: ~" + to_string(config.vram_mb) + " MB");
    return plan;
}

// ASPECT_RATIO mode: domain from aspect ratio and VRAM budget, geometry scaled to fit
DomainPlan DomainPlanner::plan_aspect_ratio(const SimulationConfig& config, LatticeMemory lattice) {
    DomainPlan plan;
    const float3 aspect(config.aspect_x_, config.aspect_y_, config.aspect_z_);
    const uint3 lbm_N = grid_for_vram(config, aspect, lattice);

    plan.Nx = lbm_N.x;
    plan.Ny = lbm_N.y;
    plan.Nz = lbm_N.z;
    plan.lbm_reference_size = config.geometry_scale_ * (float32_t)reference_dimension(config, lbm_N);
    // the geometry's real length if the config has it (Model::length()); otherwise 1 m until configure_units_with_length()
    plan.si_reference_size = config.reference_length_m_ > 0.0f ? config.reference_length_m_ : 1.0f;
    plan.rotation_matrix = rotation(config);
    plan.stl_path = get_resource_path(config.geometry_filename);
    plan.voxel_size = plan.lbm_reference_size; // the core scales the model's longest side (after the whole rotation) to this

    // Model::length(): the model's real extent along the reference axis, measured after its rotation but before the angle
    // of attack. The core scales the longest side instead, so the voxel size is converted from one to the other.
    const bool real_length = config.reference_length_m_ > 0.0f;
    float3 center;
    if(config.has_pmin_offset_ || real_length) {
        if(!std::filesystem::exists(plan.stl_path)) throw SetupError("Geometry file not found: " + config.geometry_filename);
        Mesh* mesh = read_stl(plan.stl_path, 1.0f, plan.rotation_matrix); // bounding box after the whole rotation
        const float3 mesh_size = mesh->get_bounding_box_size();
        float32_t reference = reference_dimension(config, mesh_size);
        if(real_length) {
            Mesh* unpitched = read_stl(plan.stl_path, 1.0f, rotation(config, false));
            reference = reference_dimension(config, unpitched->get_bounding_box_size());
            delete unpitched;
            plan.voxel_size = plan.lbm_reference_size * (mesh->get_max_size() / reference); // exact when the axis is the longest
        }
        delete mesh;
        const float32_t scale = plan.lbm_reference_size / reference; // cells per mesh unit
        const float3 half_size = 0.5f * scale * mesh_size;
        plan.base_grid = uint3(
            (uint32_t)(scale * mesh_size.x + 0.5f),
            (uint32_t)(scale * mesh_size.y + 0.5f),
            (uint32_t)(scale * mesh_size.z + 0.5f)
        );

        if(config.has_pmin_offset_) { // X centered in the domain, Y/Z from the pmin offset
            center.x = 0.5f * (float32_t)lbm_N.x;
            center.y = config.pmin_offset_ratio_.y * plan.lbm_reference_size + half_size.y;
            center.z = config.pmin_offset_ratio_.z * plan.lbm_reference_size + half_size.z;
        } else {
            const float3 domain_center = 0.5f * float3((float32_t)lbm_N.x, (float32_t)lbm_N.y, (float32_t)lbm_N.z);
            const float3 offset(config.center_offset_x_, config.center_offset_y_, config.center_offset_z_);
            center = domain_center + offset * plan.lbm_reference_size;
        }
    } else {
        const float3 domain_center = 0.5f * float3((float32_t)lbm_N.x, (float32_t)lbm_N.y, (float32_t)lbm_N.z);
        const float3 offset(config.center_offset_x_, config.center_offset_y_, config.center_offset_z_);
        center = domain_center + offset * plan.lbm_reference_size;
        plan.base_grid = uint3( // estimate from the geometry scale (uniform assumption)
            (uint32_t)(config.geometry_scale_ * (float32_t)lbm_N.x),
            (uint32_t)(config.geometry_scale_ * (float32_t)lbm_N.y),
            (uint32_t)(config.geometry_scale_ * (float32_t)lbm_N.z)
        );
    }
    plan.center_lbm = center;

    print_info("Geometry: " + config.geometry_filename + " (aspect ratio mode)");
    print_info("Domain: Nx=" + to_string(plan.Nx) + ", Ny=" + to_string(plan.Ny) + ", Nz=" + to_string(plan.Nz));
    print_info("Geometry scale: " + to_string(config.geometry_scale_ * 100.0f, 1u) + "% of reference axis");
    print_info("LBM reference size: " + to_string(plan.lbm_reference_size, 1u) + " cells");
    print_info("VRAM usage: ~" + to_string(config.vram_mb) + " MB");
    if(config.angle_of_attack_deg_ != 0.0f) {
        print_info("Angle of attack: " + to_string(config.angle_of_attack_deg_, 1u) + " deg");
    }
    return plan;
}

// GEOMETRY_BASED mode (default): domain from STL + clearances
DomainPlan DomainPlanner::plan_geometry_based(const SimulationConfig& config, LatticeMemory lattice) {
    GeometryScaler::ReferenceAxis scaler_axis;
    switch(config.reference_axis) {
        case SimulationConfig::ReferenceAxis::X: scaler_axis = GeometryScaler::ReferenceAxis::X; break;
        case SimulationConfig::ReferenceAxis::Y: scaler_axis = GeometryScaler::ReferenceAxis::Y; break;
        case SimulationConfig::ReferenceAxis::Z: scaler_axis = GeometryScaler::ReferenceAxis::Z; break;
        case SimulationConfig::ReferenceAxis::MAX: scaler_axis = GeometryScaler::ReferenceAxis::MAX; break;
        case SimulationConfig::ReferenceAxis::MIN: scaler_axis = GeometryScaler::ReferenceAxis::MIN; break;
        default: scaler_axis = GeometryScaler::ReferenceAxis::Y; break;
    }
    const GeometryScaler::Clearances clearances { config.bottom_clearance_m, config.top_clearance_m, config.side_clearance_m };

    DomainPlan plan;
    plan.stl_path = get_resource_path(config.geometry_filename);
    const GeometryScaler scaler = (config.resolution_mode_ == SimulationConfig::ResolutionMode::VOXEL_SIZE)
        ? GeometryScaler(plan.stl_path, config.voxel_size_m_, config.max_vram_mb_, clearances, lattice, scaler_axis)
        : GeometryScaler(plan.stl_path, config.vram_mb, clearances, lattice, scaler_axis);

    plan.base_grid = scaler.get_stl_size_cells();
    plan.stl_size_si = scaler.get_stl_size_meters();
    plan.lbm_reference_size = scaler.get_reference_size_cells();
    plan.si_reference_size = scaler.get_reference_size_meters();

    const uint3 domain_size = scaler.calculate_domain_size(clearances);
    plan.Nx = domain_size.x;
    plan.Ny = domain_size.y;
    plan.Nz = domain_size.z;
    plan.center_lbm = scaler.calculate_center(domain_size, clearances);
    plan.rotation_matrix = rotation(config);
    plan.voxel_size = fmax(fmax((float)plan.base_grid.x, (float)plan.base_grid.y), (float)plan.base_grid.z);

    print_info("Geometry: " + config.geometry_filename);
    print_info("Dimensions: X=" + to_string(plan.stl_size_si.x) + "m, Y=" + to_string(plan.stl_size_si.y) + "m, Z=" + to_string(plan.stl_size_si.z) + "m");
    print_info("Scale: " + to_string(scaler.get_scale_factor()) + " m/cell");
    print_info("Base grid (mesh only): " + to_string(plan.base_grid.x) + " x " + to_string(plan.base_grid.y) + " x " + to_string(plan.base_grid.z));
    print_info("Domain: Nx=" + to_string(plan.Nx) + ", Ny=" + to_string(plan.Ny) + ", Nz=" + to_string(plan.Nz));
    print_info("VRAM usage: ~" + to_string(config.vram_mb) + " MB");
    return plan;
}

float3x3 DomainPlanner::rotation(const SimulationConfig& config, bool with_angle_of_attack) {
    const float3x3 Rx = float3x3(float3(1, 0, 0), radians(config.rotation_x));
    const float3x3 Ry = float3x3(float3(0, 1, 0), radians(config.rotation_y));
    const float3x3 Rz = float3x3(float3(0, 0, 1), radians(config.rotation_z));
    const float3x3 base_rotation = Rz * Ry * Rx;
    if(with_angle_of_attack && config.angle_of_attack_deg_ != 0.0f) { // angle of attack: additional pitch
        const float3x3 R_aoa = float3x3(float3(1, 0, 0), radians(config.angle_of_attack_deg_));
        return R_aoa * base_rotation;
    }
    return base_rotation;
}

float32_t DomainPlanner::reference_dimension(const SimulationConfig& config, const float3& size) {
    switch(config.reference_axis) {
        case SimulationConfig::ReferenceAxis::X: return size.x;
        case SimulationConfig::ReferenceAxis::Y: return size.y;
        case SimulationConfig::ReferenceAxis::Z: return size.z;
        case SimulationConfig::ReferenceAxis::MAX: return fmax(fmax(size.x, size.y), size.z);
        case SimulationConfig::ReferenceAxis::MIN: return fmin(fmin(size.x, size.y), size.z);
        default: return size.y;
    }
}

uint32_t DomainPlanner::reference_dimension(const SimulationConfig& config, const uint3& size) {
    switch(config.reference_axis) {
        case SimulationConfig::ReferenceAxis::X: return size.x;
        case SimulationConfig::ReferenceAxis::Y: return size.y;
        case SimulationConfig::ReferenceAxis::Z: return size.z;
        case SimulationConfig::ReferenceAxis::MAX: return max(max(size.x, size.y), size.z);
        case SimulationConfig::ReferenceAxis::MIN: return min(min(size.x, size.y), size.z);
        default: return size.y;
    }
}

uint3 DomainPlanner::grid_for_vram(const SimulationConfig& config, const float3& aspect, LatticeMemory lattice) {
    const GridSize grid = grid_for_memory(aspect.x, aspect.y, aspect.z, config.vram_mb, lattice);
    return uint3(grid.x, grid.y, grid.z);
}
