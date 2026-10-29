#include "setup/domain/geometry_scaler.hpp"

#include <cmath>
#include <filesystem>
#include <memory>

GeometryScaler::GeometryScaler(
    const string& stl_path,
    si::MemoryAmount budget,
    const Clearances& clearances,
    LatticeMemory lattice,
    const float3x3& rotation
) : stl_path_(stl_path), budget_(budget), bottom_m_(metres(clearances.bottom)), top_m_(metres(clearances.top)),
    sides_m_(metres(clearances.sides)), lattice_(lattice), rotation_(rotation) {
    load_stl_and_calculate_scaling();
}

GeometryScaler::GeometryScaler(
    const string& stl_path,
    si::Length cell,
    si::MemoryAmount limit,
    const Clearances& clearances,
    LatticeMemory lattice,
    const float3x3& rotation
) : stl_path_(stl_path), bottom_m_(metres(clearances.bottom)), top_m_(metres(clearances.top)), sides_m_(metres(clearances.sides)),
    lattice_(lattice), rotation_(rotation) {
    read_stl_size();
    si_reference_size_m_ = get_reference_dimension(stl_size_m_);
    const float32_t cell_m{ metres(cell) };
    calculate_from_voxel_size(cell_m, limit);
}

void GeometryScaler::read_stl_size() {
    // checked here: the core's read_stl() reports a missing file with print_error(), which waits for Enter and exits
    if(!std::filesystem::exists(stl_path_)) {
        throw SetupError("GeometryScaler: STL file not found: " + stl_path_);
    }
    // measured as the core voxelizes it (review: ahmed_body's X and Y were swapped); the core's read_stl() returns a
    // Mesh made with new, so the unique_ptr owns it
    const std::unique_ptr<const Mesh> mesh(read_stl(stl_path_, 1.0f, rotation_));
    if(mesh == nullptr) {
        throw SetupError("GeometryScaler: failed to load STL file: " + stl_path_);
    }
    stl_size_m_ = mesh->get_bounding_box_size();
}

void GeometryScaler::load_stl_and_calculate_scaling() {
    read_stl_size();

    // the VRAM budget applies to the whole domain: STL + clearances
    const float32_t both_sides_m{ 2.0f * sides_m_ };
    const float32_t domain_x_m{ stl_size_m_.x + both_sides_m };
    const float32_t domain_y_m{ stl_size_m_.y + both_sides_m };
    const float32_t domain_z_m{ stl_size_m_.z + bottom_m_ + top_m_ };
    const float3 domain_size_m{ domain_x_m, domain_y_m, domain_z_m };
    const float32_t domain_reference{ get_reference_dimension(domain_size_m) };
    const float32_t aspect_x{ domain_size_m.x / domain_reference };
    const float32_t aspect_y{ domain_size_m.y / domain_reference };
    const float32_t aspect_z{ domain_size_m.z / domain_reference };
    const GridSize domain_lbm{ lattice_.grid_for_memory(aspect_x, aspect_y, aspect_z, budget_) };

    const float3 domain_cells{ static_cast<float32_t>(domain_lbm.x), static_cast<float32_t>(domain_lbm.y), static_cast<float32_t>(domain_lbm.z) };
    meters_per_cell_ = domain_reference / get_reference_dimension(domain_cells);

    stl_size_lbm_.x = cells_of(stl_size_m_.x);
    stl_size_lbm_.y = cells_of(stl_size_m_.y);
    stl_size_lbm_.z = cells_of(stl_size_m_.z);

    // the reference size (for the unit conversion) is the STL's, not the domain's
    si_reference_size_m_ = get_reference_dimension(stl_size_m_);
    const float3 stl_cells{ static_cast<float32_t>(stl_size_lbm_.x), static_cast<float32_t>(stl_size_lbm_.y), static_cast<float32_t>(stl_size_lbm_.z) };
    lbm_reference_size_ = get_reference_dimension(stl_cells);
}

uint3 GeometryScaler::calculate_domain_size(const Clearances& clearances) const {
    const uint32_t side_cells{ meters_to_cells(clearances.sides) };
    const uint32_t bottom_cells{ meters_to_cells(clearances.bottom) };
    const uint32_t top_cells{ meters_to_cells(clearances.top) };

    const uint32_t x{ stl_size_lbm_.x + 2u * side_cells };
    const uint32_t y{ stl_size_lbm_.y + 2u * side_cells };
    const uint32_t z{ stl_size_lbm_.z + bottom_cells + top_cells };
    const uint3 domain_size{ x, y, z };
    return domain_size;
}

float3 GeometryScaler::calculate_center(const uint3& domain_size, const Clearances& clearances) const {
    const uint32_t bottom_cells{ meters_to_cells(clearances.bottom) };
    // X and Y at the domain's center, the core's lbm.center() (cell i's center is at i)
    const float32_t domain_x{ static_cast<float32_t>(domain_size.x) };
    const float32_t domain_y{ static_cast<float32_t>(domain_size.y) };
    const float32_t stl_z{ static_cast<float32_t>(stl_size_lbm_.z) };
    const float32_t bottom{ static_cast<float32_t>(bottom_cells) };
    const float32_t centre_x{ domain_x / 2.0f - 0.5f };
    const float32_t centre_y{ domain_y / 2.0f - 0.5f };
    const float32_t centre_z{ stl_z / 2.0f + bottom };
    const float3 centre{ centre_x, centre_y, centre_z };
    return centre;
}

void GeometryScaler::calculate_from_voxel_size(float32_t cell_m, si::MemoryAmount limit) {
    meters_per_cell_ = cell_m;

    stl_size_lbm_.x = cells_of(stl_size_m_.x);
    stl_size_lbm_.y = cells_of(stl_size_m_.y);
    stl_size_lbm_.z = cells_of(stl_size_m_.z);

    const float3 stl_cells{ static_cast<float32_t>(stl_size_lbm_.x), static_cast<float32_t>(stl_size_lbm_.y), static_cast<float32_t>(stl_size_lbm_.z) };
    lbm_reference_size_ = get_reference_dimension(stl_cells);

    // the whole domain has to fit: geometry and clearances
    const Clearances clearances{ .bottom = length_of(bottom_m_), .top = length_of(top_m_), .sides = length_of(sides_m_) };
    const uint3 domain{ calculate_domain_size(clearances) };
    const GridSize grid{ domain.x, domain.y, domain.z };
    const si::MemoryAmount required{ lattice_.required_memory(grid) };
    const double required_mib{ required.in(si::symbols::MiB) };
    const double limit_mib{ limit.in(si::symbols::MiB) };
    const uint64_t required_mb{ static_cast<uint64_t>(required_mib) };
    const uint64_t limit_mb{ static_cast<uint64_t>(limit_mib) };

    if(required > limit) {
        const std::string cells_text{ to_string(domain.x) + " x " + to_string(domain.y) + " x " + to_string(domain.z) };
        const float32_t smallest_cell_m{ calculate_min_voxel_size(limit) };
        std::string message{ "VRAM requirement exceeds the limit: a cell size of " };
        message += to_string(cell_m);
        message += " m gives a grid of " + cells_text;
        message += " cells with the clearances, which needs " + to_string(required_mb);
        message += " MB of the " + to_string(limit_mb);
        message += " MB allowed (vram_limit). Use a cell size of at least " + to_string(smallest_cell_m);
        message += " m.";
        throw SetupError(message);
    }

    print_info("Voxel size mode: " + to_string(cell_m) + " m/cell");
    print_info("Required VRAM: " + to_string(required_mb) + " MB / " + to_string(limit_mb) + " MB");
}

float32_t GeometryScaler::calculate_min_voxel_size(si::MemoryAmount limit) const {
    // cells = V/v³ <= max_cells for the domain volume V (geometry and clearances), so v >= cbrt(V/max_cells)
    const double limit_mib{ limit.in(si::symbols::MiB) };
    const float32_t limit_mb{ static_cast<float32_t>(limit_mib) };
    const double cell_bytes{ lattice_.memory_per_cell.in(au::bytes) };
    const float32_t bytes_per_cell{ static_cast<float32_t>(cell_bytes) };
    const float32_t max_cells{ limit_mb * bytes_per_mib / bytes_per_cell };
    const float32_t both_sides_m{ 2.0f * sides_m_ };
    const float32_t domain_x_m{ stl_size_m_.x + both_sides_m };
    const float32_t domain_y_m{ stl_size_m_.y + both_sides_m };
    const float32_t domain_z_m{ stl_size_m_.z + bottom_m_ + top_m_ };
    const float32_t volume_m3{ domain_x_m * domain_y_m * domain_z_m };
    const float32_t smallest{ std::cbrt(volume_m3 / max_cells) };
    return smallest;
}
