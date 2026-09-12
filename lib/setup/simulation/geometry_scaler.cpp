#include "simulation/geometry_scaler.hpp"
#include "lbm.hpp"

GeometryScaler::GeometryScaler(
    const string& stl_path,
    uint32_t vram_mb,
    const Clearances& clearances,
    ReferenceAxis reference_axis
) : stl_path_(stl_path), vram_mb_(vram_mb), reference_axis_(reference_axis), clearances_(clearances) {
    load_stl_and_calculate_scaling();
}

GeometryScaler::GeometryScaler(
    const string& stl_path,
    float32_t voxel_size_meters,
    uint32_t max_vram_mb,
    ReferenceAxis reference_axis
) : stl_path_(stl_path), vram_mb_(0u), reference_axis_(reference_axis), clearances_{0.0f, 0.0f, 0.0f} {
    // Load STL to extract dimensions
    Mesh* mesh = read_stl(stl_path_, 1.0f);
    if(mesh == nullptr) {
        print_error("GeometryScaler: Failed to load STL file: " + stl_path_);
        exit(1);
    }

    stl_size_si_ = mesh->get_bounding_box_size();
    delete mesh;

    // Get reference dimension
    si_reference_size_ = get_reference_dimension(stl_size_si_);

    // Calculate using voxel-size mode
    calculate_from_voxel_size(voxel_size_meters, max_vram_mb);
}

void GeometryScaler::load_stl_and_calculate_scaling() {
    // Load STL to extract dimensions
    Mesh* mesh = read_stl(stl_path_, 1.0f);
    if(mesh == nullptr) {
        print_error("GeometryScaler: Failed to load STL file: " + stl_path_);
        exit(1);
    }

    stl_size_si_ = mesh->get_bounding_box_size();
    delete mesh;

    // Calculate TOTAL domain size in SI units (STL + clearances)
    // This is the key fix: VRAM budget applies to TOTAL domain, not just STL
    const float3 domain_size_si = float3(
        stl_size_si_.x + 2.0f * clearances_.side_m,
        stl_size_si_.y + 2.0f * clearances_.side_m,
        stl_size_si_.z + clearances_.bottom_m + clearances_.top_m
    );

    // Get reference dimension from TOTAL domain (for aspect ratio calculation)
    const float32_t domain_reference = get_reference_dimension(domain_size_si);

    // Calculate aspect ratios based on TOTAL domain
    const float32_t aspect_x = domain_size_si.x / domain_reference;
    const float32_t aspect_y = domain_size_si.y / domain_reference;
    const float32_t aspect_z = domain_size_si.z / domain_reference;

    // Calculate optimal lattice resolution for TOTAL domain from VRAM budget
    const float3 aspect_ratio = float3(aspect_x, aspect_y, aspect_z);
    const uint3 domain_lbm = resolution(aspect_ratio, vram_mb_);

    // Calculate scale factor from TOTAL domain: meters per cell
    meters_per_cell_ = domain_reference / get_reference_dimension(
        float3((float32_t)domain_lbm.x, (float32_t)domain_lbm.y, (float32_t)domain_lbm.z)
    );

    // Now calculate STL size in cells using the scale factor
    stl_size_lbm_.x = (uint32_t)(stl_size_si_.x / meters_per_cell_ + 0.5f);
    stl_size_lbm_.y = (uint32_t)(stl_size_si_.y / meters_per_cell_ + 0.5f);
    stl_size_lbm_.z = (uint32_t)(stl_size_si_.z / meters_per_cell_ + 0.5f);

    // Get reference dimension from STL (for unit conversion)
    si_reference_size_ = get_reference_dimension(stl_size_si_);
    lbm_reference_size_ = get_reference_dimension(
        float3((float32_t)stl_size_lbm_.x, (float32_t)stl_size_lbm_.y, (float32_t)stl_size_lbm_.z)
    );
}

uint3 GeometryScaler::calculate_domain_size(const Clearances& clearances) const {
    // Convert clearances from meters to cells using the conversion method
    const uint32_t side_cells = meters_to_cells(clearances.side_m);
    const uint32_t bottom_cells = meters_to_cells(clearances.bottom_m);
    const uint32_t top_cells = meters_to_cells(clearances.top_m);

    // Apply clearances
    uint3 domain_size;
    domain_size.x = stl_size_lbm_.x + 2 * side_cells;
    domain_size.y = stl_size_lbm_.y + 2 * side_cells;
    domain_size.z = stl_size_lbm_.z + bottom_cells + top_cells;

    return domain_size;
}

float3 GeometryScaler::calculate_center(const uint3& domain_size, const Clearances& clearances) const {
    const uint32_t bottom_cells = meters_to_cells(clearances.bottom_m);
    return float3(
        (float32_t)domain_size.x / 2.0f,
        (float32_t)domain_size.y / 2.0f,
        (float32_t)stl_size_lbm_.z / 2.0f + (float32_t)bottom_cells
    );
}

float32_t GeometryScaler::get_reference_dimension(const float3& size) const {
    switch(reference_axis_) {
        case ReferenceAxis::X: return size.x;
        case ReferenceAxis::Y: return size.y;
        case ReferenceAxis::Z: return size.z;
        case ReferenceAxis::MAX: return fmax(fmax(size.x, size.y), size.z);
        case ReferenceAxis::MIN: return fmin(fmin(size.x, size.y), size.z);
        default: return size.y;
    }
}

void GeometryScaler::calculate_from_voxel_size(float32_t voxel_size_m, uint32_t max_vram_mb) {
    // Set scale factor directly from voxel size
    meters_per_cell_ = voxel_size_m;

    // Calculate LBM grid dimensions from STL size
    stl_size_lbm_.x = (uint32_t)(stl_size_si_.x / voxel_size_m + 0.5f);
    stl_size_lbm_.y = (uint32_t)(stl_size_si_.y / voxel_size_m + 0.5f);
    stl_size_lbm_.z = (uint32_t)(stl_size_si_.z / voxel_size_m + 0.5f);

    // Calculate reference size in LBM units
    lbm_reference_size_ = get_reference_dimension(
        float3((float32_t)stl_size_lbm_.x, (float32_t)stl_size_lbm_.y, (float32_t)stl_size_lbm_.z)
    );

    // Calculate required memory
    const uint64_t total_cells = (uint64_t)stl_size_lbm_.x *
                              (uint64_t)stl_size_lbm_.y *
                              (uint64_t)stl_size_lbm_.z;
    const uint32_t bytes_per_cell = bytes_per_cell_device();
    const uint32_t required_mb = (uint32_t)(total_cells * (uint64_t)bytes_per_cell / 1048576ull);

    // Validate against max VRAM
    if(required_mb > max_vram_mb) {
        print_error("VRAM requirement exceeds limit!");
        print_error("  Voxel size: " + to_string(voxel_size_m) + " m");
        print_error("  Grid: " + to_string(stl_size_lbm_.x) + " x " +
                    to_string(stl_size_lbm_.y) + " x " +
                    to_string(stl_size_lbm_.z));
        print_error("  Total cells: " + to_string(total_cells));
        print_error("  Required VRAM: " + to_string(required_mb) + " MB");
        print_error("  Max VRAM: " + to_string(max_vram_mb) + " MB");

        const float32_t min_voxel_size = calculate_min_voxel_size(max_vram_mb);
        print_error("Suggestion: Increase voxel size to >= " +
                    to_string(min_voxel_size) + " m");
        exit(1);
    }

    // Print success info
    print_info("Voxel size mode: " + to_string(voxel_size_m) + " m/cell");
    print_info("Required VRAM: " + to_string(required_mb) + " MB / " +
               to_string(max_vram_mb) + " MB");
}

float32_t GeometryScaler::calculate_min_voxel_size(uint32_t max_vram_mb) const {
    const uint32_t bytes_per_cell = bytes_per_cell_device();
    const float32_t max_cells = (float32_t)max_vram_mb * 1048576.0f / (float32_t)bytes_per_cell;

    // Volume = Lx * Ly * Lz = (sx/v) * (sy/v) * (sz/v) = (sx*sy*sz)/v³
    // max_cells = volume
    // v³ = (sx*sy*sz) / max_cells
    const float32_t volume_m3 = stl_size_si_.x * stl_size_si_.y * stl_size_si_.z;
    const float32_t min_voxel_size = cbrt(volume_m3 / max_cells);

    return min_voxel_size;
}
