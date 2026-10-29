#include "setup/domain/domain_plan.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/geometry_scaler.hpp"
#include "sdf_cache/sdf_cache.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>

DomainPlan DomainPlanner::plan(const Domain& domain, LatticeMemory lattice) {
    domain.validate();
    if(!domain.model_) {
        DomainPlan plan{ plan_box(domain, lattice) };
        split_for_buffers(plan, domain, lattice);
        return plan;
    }
    const Model& model{ *domain.model_ };
    DomainPlan plan{};
    if(domain.size_) {
        plan = plan_in_size(domain, model, lattice);
    } else {
        plan = plan_with_clearances(domain, model, lattice);
    }
    plan.mirror = model.mirror_;
    split_for_buffers(plan, domain, lattice);
    return plan;
}

std::uint64_t DomainPlanner::domain_cells(GridSize grid, uint3 gpus, std::uint32_t halo_layers) {
    const std::uint64_t halo{ 2u * static_cast<std::uint64_t>(halo_layers) }; // both sides of a split axis, as LBM::LBM() sizes a domain
    const std::uint64_t halo_x{ gpus.x > 1u ? halo : 0u };
    const std::uint64_t halo_y{ gpus.y > 1u ? halo : 0u };
    const std::uint64_t halo_z{ gpus.z > 1u ? halo : 0u };
    const std::uint64_t x{ grid.x / gpus.x + halo_x };
    const std::uint64_t y{ grid.y / gpus.y + halo_y };
    const std::uint64_t z{ grid.z / gpus.z + halo_z };
    const std::uint64_t cells{ x * y * z };
    return cells;
}

si::MemoryAmount DomainPlanner::domain_buffer_memory(GridSize grid, uint3 gpus, LatticeMemory lattice) {
    const std::uint64_t cells{ domain_cells(grid, gpus, lattice.halo_layers) };
    const std::uint64_t per_cell{ LatticeMemory::whole_bytes(lattice.buffer_memory_per_cell) };
    const std::uint64_t bytes{ cells * per_cell };
    const double bytes_double{ static_cast<double>(bytes) }; // exact below 2^53 bytes
    const si::MemoryAmount memory{ au::bytes(bytes_double) };
    return memory;
}

std::string DomainPlanner::whole_mb_text(si::MemoryAmount amount) {
    const double mib{ amount.in(si::symbols::MiB) };
    const std::uint64_t whole_mib{ static_cast<std::uint64_t>(mib) }; // rounded down, as the core prints MB
    const std::string text{ std::to_string(whole_mib) };
    return text;
}

void DomainPlanner::split_for_buffers(DomainPlan& plan, const Domain& domain, LatticeMemory lattice) {
    plan.gpus = domain.gpus_;
    const GridSize grid{ plan.Nx, plan.Ny, plan.Nz };
    const si::MemoryAmount limit{ lattice.max_buffer };
    const bool limited{ limit > au::ZERO && lattice.buffer_memory_per_cell > au::ZERO };
    const si::MemoryAmount whole_buffer{ domain_buffer_memory(grid, plan.gpus, lattice) };
    if(!limited || whole_buffer <= limit) {
        return;
    }
    const bool chosen{ domain.gpus_.x != 1u || domain.gpus_.y != 1u || domain.gpus_.z != 1u };
    if(chosen) {
        const std::string split{ to_string(domain.gpus_.x) + ", " + to_string(domain.gpus_.y) + ", " + to_string(domain.gpus_.z) };
        const std::string buffer_mb{ whole_mb_text(whole_buffer) };
        const std::string limit_mb{ whole_mb_text(limit) };
        std::string message{ "Domain: with gpus(" };
        message += split;
        message += ") each domain's DDF buffer is " + buffer_mb;
        message += " MB, over the device's limit of " + limit_mb;
        message += " MB on one buffer";
        throw SetupError(message);
    }
    split_along_longest(plan, lattice, limit);
}

void DomainPlanner::split_along_longest(DomainPlan& plan, LatticeMemory lattice, si::MemoryAmount limit) {
    const GridSize grid{ plan.Nx, plan.Ny, plan.Nz };
    // the longest horizontal axis: the faces the domains exchange each step are the smallest
    const bool along_x{ grid.x >= grid.y };
    const std::uint32_t cells{ along_x ? grid.x : grid.y };
    const std::string axis_name{ along_x ? "X" : "Y" };
    for(std::uint32_t k = 2u; k <= cells / 2u; k++) {
        const uint3 gpus{ along_x ? uint3(k, 1u, 1u) : uint3(1u, k, 1u) };
        const si::MemoryAmount buffer{ domain_buffer_memory(grid, gpus, lattice) };
        if(buffer > limit) {
            continue;
        }
        plan.gpus = gpus;
        if(along_x) {
            plan.Nx = (plan.Nx / k) * k;
        } else {
            plan.Ny = (plan.Ny / k) * k;
        }
        const std::string buffer_mb{ whole_mb_text(buffer) };
        const std::string limit_mb{ whole_mb_text(limit) };
        std::string message{ "Domain split in " };
        message += to_string(k);
        message += " domains along " + axis_name;
        message += " (Nx=" + to_string(plan.Nx);
        message += ", Ny=" + to_string(plan.Ny);
        message += "): each domain's DDF buffer " + buffer_mb;
        message += " MB, the device's limit " + limit_mb;
        message += " MB on one buffer";
        print_info(message);
        return;
    }
    const si::MemoryAmount whole_buffer{ domain_buffer_memory(grid, plan.gpus, lattice) };
    const std::string buffer_mb{ whole_mb_text(whole_buffer) };
    const std::string limit_mb{ whole_mb_text(limit) };
    throw SetupError("Domain: the DDF buffer of " + buffer_mb + " MB is over the device's limit of " + limit_mb + " MB on one buffer, and no split along " +
                     axis_name + " fits it");
}

std::optional<std::string> DomainPlanner::level_forcing_refusal(const DomainPlan& plan, const TransportBuild& build) {
    if(!build.level_forcing || plan.gpus.z <= 1u) {
        return std::nullopt;
    }
    const std::string gpus_z{ std::to_string(plan.gpus.z) };
    std::optional<std::string> reason{ "Domain: LEVEL_FORCING indexes its levels by the domain's local z: a split along z (gpus z " + gpus_z +
                                       ") is refused; split along x or y, or keep_levels_whole()" };
    return reason;
}

void DomainPlanner::choose_voxelization(DomainPlan& plan, const Domain& domain) {
    if(!domain.model_) {
        return;
    }
    const Model& model{ *domain.model_ };
    plan.voxelize_sdf = model.is_sdf();
    if(domain.size_) {
        plan.voxelization_path = get_resource_path(model.file_);
        return;
    }
    const bool stl{ !model.is_sdf() && model.file_.find(".stl") != string::npos };
    const bool cached_sdf{ model.is_sdf() && model.file_.find("sdf_cache") != string::npos };
    if(stl) {
        // convert the STL to an SDF at the base grid resolution (cached in resources/sdf_cache/), for smoother voxelization
        SDFCacheConfig cache{};
        cache.fix_mesh = model.repair_mesh_;
        cache.verbose = true;
        SDFCacheManager manager(cache);
        const std::string sdf_path{ manager.get_or_generate(plan.stl_path, plan.base_grid.x, plan.base_grid.y, plan.base_grid.z) };
        if(sdf_path.empty()) {
            throw SetupError("Domain: the SDF of " + plan.stl_path + " cannot be made (see the [SDF Cache] lines above)");
        }
        plan.voxelization_path = sdf_path;
        plan.voxelize_sdf = true;
        print_info("Using SDF voxelization (cached: " + sdf_path + ")");
    } else if(cached_sdf) {
        plan.voxelization_path = model.file_;
    } else {
        plan.voxelization_path = get_resource_path(model.file_);
    }
}

float DomainPlanner::metres(si::Length x) {
    const double value{ x.in(au::meters) };
    const float metres_float{ static_cast<float>(value) };
    return metres_float;
}

float3 DomainPlanner::metres(si::Length x, si::Length y, si::Length z) {
    const float3 v{ metres(x), metres(y), metres(z) };
    return v;
}

// Domain::box(): no model, the grid from the size in metres
DomainPlan DomainPlanner::plan_box(const Domain& domain, LatticeMemory lattice) {
    if(!domain.size_.has_value()) {
        throw SetupError("DomainPlanner::plan_box(): the domain has no size (use Domain::box())");
    }
    const Domain::Box& size{ domain.size_.value() };
    const float3 size_m{ metres(size.x, size.y, size.z) };
    const float max_dim{ std::fmax(std::fmax(size_m.x, size_m.y), size_m.z) };
    const float3 aspect{ size_m.x / max_dim, size_m.y / max_dim, size_m.z / max_dim };
    const uint3 lbm_N{ grid_for_resolution(domain, aspect, size_m, lattice) };
    const uint32_t longest_cells{ max(max(lbm_N.x, lbm_N.y), lbm_N.z) };

    DomainPlan plan{};
    plan.Nx = lbm_N.x;
    plan.Ny = lbm_N.y;
    plan.Nz = lbm_N.z;
    plan.reference_length = au::meters(static_cast<double>(max_dim));
    plan.lbm_reference_size = static_cast<float32_t>(longest_cells);
    plan.stl_size = si::Position{ au::meters(static_cast<double>(size_m.x)), au::meters(static_cast<double>(size_m.y)), au::meters(static_cast<double>(size_m.z)) };
    plan.center_lbm = center_of(lbm_N);
    plan.base_grid = uint3(0u, 0u, 0u);
    plan.rotation_matrix = float3x3(1.0f);

    const GridSize grid{ plan.Nx, plan.Ny, plan.Nz };
    const si::MemoryAmount required{ lattice.required_memory(grid) };
    const double required_mib{ required.in(si::symbols::MiB) };
    const std::uint64_t required_mb{ static_cast<std::uint64_t>(required_mib) };
    const std::string memory_mb{ to_string(required_mb) };
    print_info("Domain without a model: " + to_string(size_m.x) + " m x " + to_string(size_m.y) + " m x " + to_string(size_m.z) + " m");
    print_info("Domain: Nx=" + to_string(plan.Nx) + ", Ny=" + to_string(plan.Ny) + ", Nz=" + to_string(plan.Nz) + " (" + memory_mb + " MB)");
    return plan;
}

// Domain::around(model).size(): the grid from the size in metres, the model scaled to its real length and placed
DomainPlan DomainPlanner::plan_in_size(const Domain& domain, const Model& model, LatticeMemory lattice) {
    if(!domain.size_.has_value() || !model.length_.has_value()) {
        throw SetupError("DomainPlanner::plan_in_size(): the domain needs a size() and its model a Model::length()");
    }
    const Domain::Box& size{ domain.size_.value() };
    const float3 size_m{ metres(size.x, size.y, size.z) };
    const si::Length model_length{ model.length_.value() };
    const float32_t length{ metres(model_length) };
    const Axis axis{ model.length_axis_ };
    const float32_t share{ length / along(size_m, axis) }; // of the domain along the model's axis
    const uint3 lbm_N{ grid_for_resolution(domain, size_m, size_m, lattice) }; // the size in metres is the aspect ratio
    const float32_t cells_along_axis{ static_cast<float32_t>(along(lbm_N, axis)) };

    DomainPlan plan{};
    plan.Nx = lbm_N.x;
    plan.Ny = lbm_N.y;
    plan.Nz = lbm_N.z;
    plan.lbm_reference_size = share * cells_along_axis;
    plan.reference_length = au::meters(static_cast<double>(length));
    plan.rotation_matrix = model.rotation_matrix();
    plan.stl_path = get_resource_path(model.file_);
    if(!std::filesystem::exists(plan.stl_path)) {
        throw SetupError("Geometry file not found: " + model.file_);
    }

    // Model::length() is the extent along its axis after the rotation, before the angle of attack. The core scales
    // another size (ModelBox::scale_size), so the voxel size is converted from one to the other.
    const ModelBox box{ model_box(model, plan.stl_path) };
    const float3 mesh_size{ box.size };
    const float32_t reference{ along(box.unpitched_size, axis) };
    plan.voxel_size = plan.lbm_reference_size * (box.scale_size / reference); // exact when the axis is the longest

    const float32_t scale{ plan.lbm_reference_size / reference }; // cells per mesh unit
    const float3 half_size{ 0.5f * scale * mesh_size };
    const float3 base_cells{ scale * mesh_size.x, scale * mesh_size.y, scale * mesh_size.z };
    // the core's rounding to the nearest whole cell (0 or more)
    const uint32_t base_x{ to_uint(base_cells.x) };
    const uint32_t base_y{ to_uint(base_cells.y) };
    const uint32_t base_z{ to_uint(base_cells.z) };
    plan.base_grid = uint3(base_x, base_y, base_z);
    plan.center_lbm = model_centre(domain, center_of(lbm_N), half_size, length, plan.lbm_reference_size);

    const GridSize grid{ plan.Nx, plan.Ny, plan.Nz };
    const si::MemoryAmount required{ lattice.required_memory(grid) };
    const double required_mib{ required.in(si::symbols::MiB) };
    const std::uint64_t required_mb{ static_cast<std::uint64_t>(required_mib) };
    const std::string memory_mb{ to_string(required_mb) };
    const std::string length_cells{ to_string(plan.lbm_reference_size, 1u) };
    const float32_t share_percent{ share * 100.0f };
    print_info("Geometry: " + model.file_);
    print_info("Domain: Nx=" + to_string(plan.Nx) + ", Ny=" + to_string(plan.Ny) + ", Nz=" + to_string(plan.Nz) + " (" + memory_mb + " MB)");
    print_info("Model length: " + to_string(length) + " m = " + length_cells + " cells (" + to_string(share_percent, 1u) + "% of the domain)");
    if(model.angle_of_attack_ != au::ZERO) {
        const float pitch_deg{ static_cast<float>(model.angle_of_attack_.in(au::degrees)) };
        print_info("Angle of attack: " + to_string(pitch_deg, 1u) + " deg");
    }
    return plan;
}

float3 DomainPlanner::model_centre(const Domain& domain, const float3& domain_centre, const float3& half_size, float32_t length, float32_t length_cells) {
    if(!domain.has_gaps()) { // centred, moved by the model offset
        const Domain::Box offset_m{ domain.model_offset_.value_or(Domain::Box{}) };
        const float3 offset_metres{ metres(offset_m.x, offset_m.y, offset_m.z) };
        const float3 offset{ offset_metres.x / length, offset_metres.y / length, offset_metres.z / length }; // in model lengths
        const float3 centre{ domain_centre + offset * length_cells };
        return centre;
    }
    // X centred, the bounding box minimum at the gaps to the inlet and the floor
    const float32_t gap_y{ domain.gap_to_inlet_ ? metres(*domain.gap_to_inlet_) : 0.0f };
    const float32_t gap_z{ domain.gap_to_floor_ ? metres(*domain.gap_to_floor_) : 0.0f };
    const float32_t bottom_cells{ domain.on_floor_ ? 1.0f : (gap_z / length) * length_cells };
    float3 centre{};
    centre.x = domain_centre.x;
    centre.y = (gap_y / length) * length_cells + half_size.y;
    centre.z = bottom_cells + half_size.z;
    return centre;
}

// Domain::around(model).clearances(): the model's STL in metres plus the clearances
DomainPlan DomainPlanner::plan_with_clearances(const Domain& domain, const Model& model, LatticeMemory lattice) {
    GeometryScaler::Clearances clearances{};
    if(domain.clearances_.has_value()) {
        const Domain::Clearances& given{ domain.clearances_.value() };
        clearances = GeometryScaler::Clearances{ .bottom = given.bottom, .top = given.top, .sides = given.sides };
    }

    DomainPlan plan{};
    plan.stl_path = get_resource_path(model.file_);
    plan.rotation_matrix = model.rotation_matrix();
    std::unique_ptr<GeometryScaler> scaler{};
    if(domain.cell_size_.has_value()) {
        const si::Length cell{ domain.cell_size_.value() };
        const si::MemoryAmount limit{ domain.memory_limit() };
        scaler = std::make_unique<GeometryScaler>(plan.stl_path, cell, limit, clearances, lattice, plan.rotation_matrix);
    } else {
        const si::MemoryAmount budget{ domain.memory_budget() };
        scaler = std::make_unique<GeometryScaler>(plan.stl_path, budget, clearances, lattice, plan.rotation_matrix);
    }

    plan.base_grid = scaler->stl_cells();
    plan.stl_size = scaler->stl_size(); // the scaler's float metres, exact in double
    plan.lbm_reference_size = scaler->reference_cells();
    plan.reference_length = scaler->reference_length();

    const uint3 domain_size{ scaler->calculate_domain_size(clearances) };
    plan.Nx = domain_size.x;
    plan.Ny = domain_size.y;
    plan.Nz = domain_size.z;
    plan.center_lbm = scaler->calculate_center(domain_size, clearances);
    const float base_x{ static_cast<float>(plan.base_grid.x) };
    const float base_y{ static_cast<float>(plan.base_grid.y) };
    const float base_z{ static_cast<float>(plan.base_grid.z) };
    plan.voxel_size = std::fmax(std::fmax(base_x, base_y), base_z);

    const GridSize grid{ plan.Nx, plan.Ny, plan.Nz };
    const si::MemoryAmount required{ lattice.required_memory(grid) };
    const double required_mib{ required.in(si::symbols::MiB) };
    const std::uint64_t required_mb{ static_cast<std::uint64_t>(required_mib) };
    const std::string memory_mb{ to_string(required_mb) };
    const si::Length cell_size{ scaler->cell_size() };
    const std::string scale_m{ to_string(metres(cell_size)) };
    print_info("Geometry: " + model.file_);
    const float3 printed_size{ metres(plan.stl_size.x, plan.stl_size.y, plan.stl_size.z) }; // the core's float metres
    print_info("Dimensions: X=" + to_string(printed_size.x) + "m, Y=" + to_string(printed_size.y) + "m, Z=" + to_string(printed_size.z) + "m");
    print_info("Scale: " + scale_m + " m/cell");
    print_info("Base grid (mesh only): " + to_string(plan.base_grid.x) + " x " + to_string(plan.base_grid.y) + " x " + to_string(plan.base_grid.z));
    print_info("Domain: Nx=" + to_string(plan.Nx) + ", Ny=" + to_string(plan.Ny) + ", Nz=" + to_string(plan.Nz) + " (" + memory_mb + " MB)");
    return plan;
}

DomainPlanner::ModelBox DomainPlanner::model_box(const Model& model, const std::string& path) {
    if(!model.is_sdf()) {
        const std::unique_ptr<Mesh> mesh(read_stl(path, 1.0f, model.rotation_matrix()));
        const std::unique_ptr<Mesh> unpitched(read_stl(path, 1.0f, model.rotation_matrix(false)));
        const ModelBox box{ .size = mesh->get_bounding_box_size(), .unpitched_size = unpitched->get_bounding_box_size(), .scale_size = mesh->get_max_size() };
        return box;
    }
    // the SDF's header (the format of the core's read_sdf()): its grid and bounds
    std::ifstream file(path, std::ios::binary);
    constexpr std::size_t bound_count{ 6 }; // minimum x, y, z, maximum x, y, z
    std::array<std::int32_t, 3> n{};
    std::array<float, bound_count> bounds{};
    file.read(reinterpret_cast<char*>(n.data()), sizeof(n)); // the binary header, as the core's read_sdf() reads it
    file.read(reinterpret_cast<char*>(bounds.data()), sizeof(bounds));
    const bool grid_valid{ n[0] > 0 && n[1] > 0 && n[2] > 0 };
    if(!file || !grid_valid) {
        throw SetupError("Cannot read the SDF file " + path);
    }
    // the world size as read_sdf() computes it: the grid's cells times their mean size
    const float3 bounds_size{ bounds[3] - bounds[0], bounds[4] - bounds[1], bounds[5] - bounds[2] };
    const float n_x{ static_cast<float>(n[0]) };
    const float n_y{ static_cast<float>(n[1]) };
    const float n_z{ static_cast<float>(n[2]) };
    const float cell_size{ (bounds_size.x / n_x + bounds_size.y / n_y + bounds_size.z / n_z) / 3.0f };
    const float3 world{ n_x * cell_size, n_y * cell_size, n_z * cell_size };
    const float longest{ std::fmax(std::fmax(world.x, world.y), world.z) };
    const ModelBox box{ .size = turned_size(world, model.rotation_matrix()), .unpitched_size = turned_size(world, model.rotation_matrix(false)), .scale_size = longest };
    return box;
}

float3 DomainPlanner::turned_size(const float3& world, const float3x3& rotation) {
    // the bounding box of the grid's box turned about its centre: the extremes of its 8 turned corners
    const float3 half{ 0.5f * world.x, 0.5f * world.y, 0.5f * world.z };
    float3 low{ std::numeric_limits<float>::max() };
    float3 high{ -std::numeric_limits<float>::max() };
    constexpr int corner_count{ 8 };
    for(int i = 0; i < corner_count; i++) {
        const bool high_x{ (i & 1) != 0 };
        const bool high_y{ (i & 2) != 0 };
        const bool high_z{ (i & 4) != 0 };
        const float corner_x{ high_x ? half.x : -half.x };
        const float corner_y{ high_y ? half.y : -half.y };
        const float corner_z{ high_z ? half.z : -half.z };
        const float3 corner{ rotation * float3(corner_x, corner_y, corner_z) };
        low = float3(fmin(low.x, corner.x), fmin(low.y, corner.y), fmin(low.z, corner.z));
        high = float3(fmax(high.x, corner.x), fmax(high.y, corner.y), fmax(high.z, corner.z));
    }
    const float3 size{ high - low };
    return size;
}

uint3 DomainPlanner::grid_for_resolution(const Domain& domain, const float3& aspect, const float3& size_m, LatticeMemory lattice) {
    if(!domain.cell_size_.has_value()) {
        const si::MemoryAmount budget{ domain.memory_budget() };
        const GridSize grid{ lattice.grid_for_memory(aspect.x, aspect.y, aspect.z, budget) };
        const uint3 cells{ grid.x, grid.y, grid.z };
        return cells;
    }
    const float32_t cell{ metres(domain.cell_size_.value()) };
    const GridSize grid{ to_uint(size_m.x / cell), to_uint(size_m.y / cell), to_uint(size_m.z / cell) };
    const std::string cell_text{ to_string(cell) };
    if(grid.x == 0u || grid.y == 0u || grid.z == 0u) {
        throw SetupError("Domain: a cell size of " + cell_text + " m is larger than the domain");
    }
    const si::MemoryAmount required{ lattice.required_memory(grid) };
    const si::MemoryAmount allowed{ domain.memory_limit() };
    if(required > allowed) {
        const std::string cells_text{ to_string(grid.x) + " x " + to_string(grid.y) + " x " + to_string(grid.z) };
        const double required_mib{ required.in(si::symbols::MiB) };
        const double allowed_mib{ allowed.in(si::symbols::MiB) };
        const std::uint64_t required_mb{ static_cast<std::uint64_t>(required_mib) };
        const std::uint64_t allowed_mb{ static_cast<std::uint64_t>(allowed_mib) };
        std::string message{ "VRAM requirement exceeds the limit: a cell size of " };
        message += cell_text;
        message += " m gives a grid of " + cells_text;
        message += " cells, which needs " + to_string(required_mb);
        message += " MB of the " + to_string(allowed_mb);
        message += " MB allowed (vram_limit)";
        throw SetupError(message);
    }
    const uint3 cells{ grid.x, grid.y, grid.z };
    return cells;
}
