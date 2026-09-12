#include "setup/domain/domain_plan.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/geometry_scaler.hpp"
#include "setup/sdf/sdf_generator.hpp"

#include <filesystem>
#include <memory>

DomainPlan DomainPlanner::plan(const Domain& domain, LatticeMemory lattice) {
    domain.validate();
    if(!domain.model_) return plan_box(domain, lattice);
    DomainPlan plan = domain.size_ ? plan_in_size(domain, *domain.model_, lattice)
                                   : plan_with_clearances(domain, *domain.model_, lattice);
    plan.mirror = domain.model_->mirror_;
    return plan;
}

void DomainPlanner::choose_voxelization(DomainPlan& plan, const Domain& domain) {
    if(!domain.model_) return;
    const Model& model = *domain.model_;
    plan.voxelize_sdf = model.is_sdf();
    if(domain.size_) {
        plan.voxelization_path = get_resource_path(model.file_);
        return;
    }
    if(!model.is_sdf() && model.file_.find(".stl") != string::npos) {
        // convert the STL to an SDF at the base grid resolution (cached), for smoother voxelization
        SDFGenerator sdf_gen;
        sdf_gen.set_cache_dir("resources/sdf_cache/")
               .enable_cache(true)
               .set_fix_mesh(model.repair_mesh_)
               .set_verbose(true);
        const std::string sdf_path = sdf_gen.generate(plan.stl_path, plan.base_grid.x, plan.base_grid.y, plan.base_grid.z, 1);
        if(!sdf_path.empty()) {
            plan.voxelization_path = sdf_path;
            plan.voxelize_sdf = true;
            print_info("Using SDF voxelization (cached: " + sdf_path + ")");
        } else {
            plan.voxelization_path = get_resource_path(model.file_);
            print_info("SDF caching failed, falling back to STL voxelization");
        }
    } else if(model.is_sdf() && model.file_.find("sdf_cache") != string::npos) {
        plan.voxelization_path = model.file_;
    } else {
        plan.voxelization_path = get_resource_path(model.file_);
    }
}

// Domain::box(): no model, the grid from the size in metres
DomainPlan DomainPlanner::plan_box(const Domain& domain, LatticeMemory lattice) {
    const float3 size_m(domain.size_->x.si(), domain.size_->y.si(), domain.size_->z.si());
    const float max_dim = fmax(fmax(size_m.x, size_m.y), size_m.z);
    const float3 aspect(size_m.x / max_dim, size_m.y / max_dim, size_m.z / max_dim);
    const uint3 lbm_N = grid_for_resolution(domain, aspect, size_m, lattice);

    DomainPlan plan;
    plan.Nx = lbm_N.x;
    plan.Ny = lbm_N.y;
    plan.Nz = lbm_N.z;
    plan.si_reference_size = max_dim;
    plan.lbm_reference_size = (float32_t)max(max(lbm_N.x, lbm_N.y), lbm_N.z);
    plan.stl_size_si = size_m;
    plan.center_lbm = 0.5f * float3((float32_t)lbm_N.x, (float32_t)lbm_N.y, (float32_t)lbm_N.z);
    plan.base_grid = uint3(0, 0, 0);
    plan.rotation_matrix = float3x3(1.0f);

    print_info("Domain without a model: " + to_string(size_m.x) + " m x " + to_string(size_m.y) + " m x " + to_string(size_m.z) + " m");
    print_info("Domain: Nx=" + to_string(plan.Nx) + ", Ny=" + to_string(plan.Ny) + ", Nz=" + to_string(plan.Nz) +
               " (" + to_string(required_memory_mb({ plan.Nx, plan.Ny, plan.Nz }, lattice)) + " MB)");
    return plan;
}

// Domain::around(model).size(): the grid from the size in metres, the model scaled to its real length and placed
DomainPlan DomainPlanner::plan_in_size(const Domain& domain, const Model& model, LatticeMemory lattice) {
    const float3 size_m(domain.size_->x.si(), domain.size_->y.si(), domain.size_->z.si());
    const float32_t length = model.length_->si();
    const Axis axis = model.length_axis_;
    const float32_t share = length / along(size_m, axis); // of the domain along the model's axis
    const uint3 lbm_N = grid_for_resolution(domain, size_m, size_m, lattice); // the size in metres is the aspect ratio

    DomainPlan plan;
    plan.Nx = lbm_N.x;
    plan.Ny = lbm_N.y;
    plan.Nz = lbm_N.z;
    plan.lbm_reference_size = share * (float32_t)along(lbm_N, axis);
    plan.si_reference_size = length;
    plan.rotation_matrix = model.rotation_matrix();
    plan.stl_path = get_resource_path(model.file_);
    if(!std::filesystem::exists(plan.stl_path)) throw SetupError("Geometry file not found: " + model.file_);

    // Model::length() is the extent along its axis after the rotation, before the angle of attack. The core scales the
    // longest side after the whole rotation instead, so the voxel size is converted from one to the other.
    const std::unique_ptr<Mesh> mesh(read_stl(plan.stl_path, 1.0f, plan.rotation_matrix));
    const std::unique_ptr<Mesh> unpitched(read_stl(plan.stl_path, 1.0f, model.rotation_matrix(false)));
    const float3 mesh_size = mesh->get_bounding_box_size();
    const float32_t reference = along(unpitched->get_bounding_box_size(), axis);
    plan.voxel_size = plan.lbm_reference_size * (mesh->get_max_size() / reference); // exact when the axis is the longest

    const float32_t scale = plan.lbm_reference_size / reference; // cells per mesh unit
    const float3 half_size = 0.5f * scale * mesh_size;
    plan.base_grid = uint3(
        (uint32_t)(scale * mesh_size.x + 0.5f),
        (uint32_t)(scale * mesh_size.y + 0.5f),
        (uint32_t)(scale * mesh_size.z + 0.5f)
    );

    if(domain.has_gaps()) { // X centered, the bounding box minimum at the gaps to the inlet and the floor
        const float32_t gap_y = domain.gap_to_inlet_ ? domain.gap_to_inlet_->si() : 0.0f;
        const float32_t gap_z = domain.gap_to_floor_ ? domain.gap_to_floor_->si() : 0.0f;
        plan.center_lbm.x = 0.5f * (float32_t)lbm_N.x;
        plan.center_lbm.y = (gap_y / length) * plan.lbm_reference_size + half_size.y;
        plan.center_lbm.z = (domain.on_floor_ ? 1.0f : (gap_z / length) * plan.lbm_reference_size) + half_size.z;
    } else { // centered, moved by the model offset
        const Domain::Box offset_m = domain.model_offset_.value_or(Domain::Box{});
        const float3 offset(offset_m.x.si() / length, offset_m.y.si() / length, offset_m.z.si() / length); // in model lengths
        const float3 domain_center = 0.5f * float3((float32_t)lbm_N.x, (float32_t)lbm_N.y, (float32_t)lbm_N.z);
        plan.center_lbm = domain_center + offset * plan.lbm_reference_size;
    }

    print_info("Geometry: " + model.file_);
    print_info("Domain: Nx=" + to_string(plan.Nx) + ", Ny=" + to_string(plan.Ny) + ", Nz=" + to_string(plan.Nz) +
               " (" + to_string(required_memory_mb({ plan.Nx, plan.Ny, plan.Nz }, lattice)) + " MB)");
    print_info("Model length: " + to_string(length) + " m = " + to_string(plan.lbm_reference_size, 1u) + " cells (" +
               to_string(share * 100.0f, 1u) + "% of the domain)");
    if(model.angle_of_attack_.deg() != 0.0f) print_info("Angle of attack: " + to_string(model.angle_of_attack_.deg(), 1u) + " deg");
    return plan;
}

// Domain::around(model).clearances(): the model's STL in metres plus the clearances
DomainPlan DomainPlanner::plan_with_clearances(const Domain& domain, const Model& model, LatticeMemory lattice) {
    const GeometryScaler::Clearances clearances = domain.clearances_
        ? GeometryScaler::Clearances { domain.clearances_->bottom.si(), domain.clearances_->top.si(), domain.clearances_->sides.si() }
        : GeometryScaler::Clearances { 0.0f, 0.0f, 0.0f };

    DomainPlan plan;
    plan.stl_path = get_resource_path(model.file_);
    plan.rotation_matrix = model.rotation_matrix();
    const GeometryScaler scaler = domain.cell_size_
        ? GeometryScaler(plan.stl_path, domain.cell_size_->si(), domain.max_vram_mb(), clearances, lattice, plan.rotation_matrix)
        : GeometryScaler(plan.stl_path, domain.vram_mb(), clearances, lattice, plan.rotation_matrix);

    plan.base_grid = scaler.get_stl_size_cells();
    plan.stl_size_si = scaler.get_stl_size_meters();
    plan.lbm_reference_size = scaler.get_reference_size_cells();
    plan.si_reference_size = scaler.get_reference_size_meters();

    const uint3 domain_size = scaler.calculate_domain_size(clearances);
    plan.Nx = domain_size.x;
    plan.Ny = domain_size.y;
    plan.Nz = domain_size.z;
    plan.center_lbm = scaler.calculate_center(domain_size, clearances);
    plan.voxel_size = fmax(fmax((float)plan.base_grid.x, (float)plan.base_grid.y), (float)plan.base_grid.z);

    print_info("Geometry: " + model.file_);
    print_info("Dimensions: X=" + to_string(plan.stl_size_si.x) + "m, Y=" + to_string(plan.stl_size_si.y) + "m, Z=" + to_string(plan.stl_size_si.z) + "m");
    print_info("Scale: " + to_string(scaler.get_scale_factor()) + " m/cell");
    print_info("Base grid (mesh only): " + to_string(plan.base_grid.x) + " x " + to_string(plan.base_grid.y) + " x " + to_string(plan.base_grid.z));
    print_info("Domain: Nx=" + to_string(plan.Nx) + ", Ny=" + to_string(plan.Ny) + ", Nz=" + to_string(plan.Nz) +
               " (" + to_string(required_memory_mb({ plan.Nx, plan.Ny, plan.Nz }, lattice)) + " MB)");
    return plan;
}

uint3 DomainPlanner::grid_for_resolution(const Domain& domain, const float3& aspect, const float3& size_m, LatticeMemory lattice) {
    if(!domain.cell_size_) {
        const GridSize grid = grid_for_memory(aspect.x, aspect.y, aspect.z, domain.vram_mb(), lattice);
        return uint3(grid.x, grid.y, grid.z);
    }
    const float32_t cell = domain.cell_size_->si();
    const GridSize grid { to_uint(size_m.x / cell), to_uint(size_m.y / cell), to_uint(size_m.z / cell) };
    if(grid.x == 0u || grid.y == 0u || grid.z == 0u) {
        throw SetupError("Domain: a cell size of " + to_string(cell) + " m is larger than the domain");
    }
    const uint32_t required_mb = (uint32_t)required_memory_mb(grid, lattice);
    if(required_mb > domain.max_vram_mb()) {
        throw SetupError("VRAM requirement exceeds the limit: a cell size of " + to_string(cell) + " m gives a grid of " +
            to_string(grid.x) + " x " + to_string(grid.y) + " x " + to_string(grid.z) + " cells, which needs " +
            to_string(required_mb) + " MB of the " + to_string(domain.max_vram_mb()) + " MB allowed (max_vram)");
    }
    return uint3(grid.x, grid.y, grid.z);
}
