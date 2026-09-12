#include "setup/domain/geometry_scaler.hpp"

#include <filesystem>

GeometryScaler::GeometryScaler(
    const string& stl_path,
    uint32_t vram_mb,
    const Clearances& clearances,
    LatticeMemory lattice,
    const float3x3& rotation
) : stl_path_(stl_path), vram_mb_(vram_mb), clearances_(clearances), lattice_(lattice), rotation_(rotation) {
    load_stl_and_calculate_scaling();
}

GeometryScaler::GeometryScaler(
    const string& stl_path,
    float32_t voxel_size_meters,
    uint32_t max_vram_mb,
    const Clearances& clearances,
    LatticeMemory lattice,
    const float3x3& rotation
) : stl_path_(stl_path), vram_mb_(0u), clearances_(clearances), lattice_(lattice), rotation_(rotation) {
    read_stl_size();
    si_reference_size_ = get_reference_dimension(stl_size_si_);
    calculate_from_voxel_size(voxel_size_meters, max_vram_mb);
}

void GeometryScaler::read_stl_size() {
    // checked here: the core's read_stl() reports a missing file with print_error(), which waits for Enter and exits
    if(!std::filesystem::exists(stl_path_)) {
        throw SetupError("GeometryScaler: STL file not found: " + stl_path_);
    }
    Mesh* mesh = read_stl(stl_path_, 1.0f, rotation_); // measured as the core voxelizes it (review: ahmed_body's X and Y were swapped)
    if(mesh == nullptr) {
        throw SetupError("GeometryScaler: failed to load STL file: " + stl_path_);
    }
    stl_size_si_ = mesh->get_bounding_box_size();
    delete mesh;
}

void GeometryScaler::load_stl_and_calculate_scaling() {
    read_stl_size();

    // the VRAM budget applies to the whole domain: STL + clearances
    const float3 domain_size_si = float3(
        stl_size_si_.x + 2.0f * clearances_.side_m,
        stl_size_si_.y + 2.0f * clearances_.side_m,
        stl_size_si_.z + clearances_.bottom_m + clearances_.top_m
    );
    const float32_t domain_reference = get_reference_dimension(domain_size_si);
    const GridSize domain_lbm = grid_for_memory(
        domain_size_si.x / domain_reference,
        domain_size_si.y / domain_reference,
        domain_size_si.z / domain_reference,
        vram_mb_, lattice_
    );

    meters_per_cell_ = domain_reference / get_reference_dimension(
        float3((float32_t)domain_lbm.x, (float32_t)domain_lbm.y, (float32_t)domain_lbm.z)
    );

    stl_size_lbm_.x = (uint32_t)(stl_size_si_.x / meters_per_cell_ + 0.5f);
    stl_size_lbm_.y = (uint32_t)(stl_size_si_.y / meters_per_cell_ + 0.5f);
    stl_size_lbm_.z = (uint32_t)(stl_size_si_.z / meters_per_cell_ + 0.5f);

    // the reference size (for the unit conversion) is the STL's, not the domain's
    si_reference_size_ = get_reference_dimension(stl_size_si_);
    lbm_reference_size_ = get_reference_dimension(
        float3((float32_t)stl_size_lbm_.x, (float32_t)stl_size_lbm_.y, (float32_t)stl_size_lbm_.z)
    );
}

uint3 GeometryScaler::calculate_domain_size(const Clearances& clearances) const {
    const uint32_t side_cells = meters_to_cells(clearances.side_m);
    const uint32_t bottom_cells = meters_to_cells(clearances.bottom_m);
    const uint32_t top_cells = meters_to_cells(clearances.top_m);

    uint3 domain_size;
    domain_size.x = stl_size_lbm_.x + 2 * side_cells;
    domain_size.y = stl_size_lbm_.y + 2 * side_cells;
    domain_size.z = stl_size_lbm_.z + bottom_cells + top_cells;
    return domain_size;
}

float3 GeometryScaler::calculate_center(const uint3& domain_size, const Clearances& clearances) const {
    const uint32_t bottom_cells = meters_to_cells(clearances.bottom_m);
    return float3( // X and Y at the domain's center, the core's lbm.center() (cell i's center is at i)
        (float32_t)domain_size.x / 2.0f - 0.5f,
        (float32_t)domain_size.y / 2.0f - 0.5f,
        (float32_t)stl_size_lbm_.z / 2.0f + (float32_t)bottom_cells
    );
}

void GeometryScaler::calculate_from_voxel_size(float32_t voxel_size_m, uint32_t max_vram_mb) {
    meters_per_cell_ = voxel_size_m;

    stl_size_lbm_.x = (uint32_t)(stl_size_si_.x / voxel_size_m + 0.5f);
    stl_size_lbm_.y = (uint32_t)(stl_size_si_.y / voxel_size_m + 0.5f);
    stl_size_lbm_.z = (uint32_t)(stl_size_si_.z / voxel_size_m + 0.5f);

    lbm_reference_size_ = get_reference_dimension(
        float3((float32_t)stl_size_lbm_.x, (float32_t)stl_size_lbm_.y, (float32_t)stl_size_lbm_.z)
    );

    // the whole domain has to fit: geometry and clearances
    const uint3 domain = calculate_domain_size(clearances_);
    const uint32_t required_mb = (uint32_t)required_memory_mb({ domain.x, domain.y, domain.z }, lattice_);

    if(required_mb > max_vram_mb) {
        throw SetupError("VRAM requirement exceeds the limit: a cell size of " + to_string(voxel_size_m) + " m gives a grid of " +
            to_string(domain.x) + " x " + to_string(domain.y) + " x " + to_string(domain.z) + " cells with the clearances, which needs " +
            to_string(required_mb) + " MB of the " + to_string(max_vram_mb) + " MB allowed (set_max_vram_mb). " +
            "Use a cell size of at least " + to_string(calculate_min_voxel_size(max_vram_mb)) + " m.");
    }

    print_info("Voxel size mode: " + to_string(voxel_size_m) + " m/cell");
    print_info("Required VRAM: " + to_string(required_mb) + " MB / " + to_string(max_vram_mb) + " MB");
}

float32_t GeometryScaler::calculate_min_voxel_size(uint32_t max_vram_mb) const {
    // cells = V/v³ <= max_cells for the domain volume V (geometry and clearances), so v >= cbrt(V/max_cells)
    const float32_t max_cells = (float32_t)max_vram_mb * 1048576.0f / (float32_t)lattice_.bytes_per_cell;
    const float32_t volume_m3 = (stl_size_si_.x + 2.0f * clearances_.side_m) *
                                (stl_size_si_.y + 2.0f * clearances_.side_m) *
                                (stl_size_si_.z + clearances_.bottom_m + clearances_.top_m);
    return cbrt(volume_m3 / max_cells);
}
