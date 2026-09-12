#pragma once
#include "setup/core/types.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/lattice.hpp"
#include "utilities.hpp"

// Scale between an STL geometry in m and the lattice: cell size from a VRAM budget (for geometry plus clearances)
// or from a given cell size, and the domain size and geometry center in cells.
// Throws SetupError if the STL file does not exist or the grid does not fit the memory limit.
class GeometryScaler {
public:
    enum class ReferenceAxis { X, Y, Z, MAX, MIN }; // geometry dimension used for scaling (default Y)

    struct Clearances { // in m
        float32_t bottom_m;
        float32_t top_m;
        float32_t side_m; // each of the four sides
    };

    // cell size so that geometry + clearances fit the VRAM budget
    GeometryScaler(
        const string& stl_path,
        uint32_t vram_mb,
        const Clearances& clearances,
        LatticeMemory lattice,
        ReferenceAxis reference_axis = ReferenceAxis::Y
    );

    // given cell size; the grid with its clearances must fit max_vram_mb
    GeometryScaler(
        const string& stl_path,
        float32_t voxel_size_meters,
        uint32_t max_vram_mb,
        const Clearances& clearances,
        LatticeMemory lattice,
        ReferenceAxis reference_axis = ReferenceAxis::Y
    );

    uint32_t meters_to_cells(float32_t meters) const { // rounded to whole cells
        return (uint32_t)(meters / meters_per_cell_ + 0.5f);
    }

    float3 get_stl_size_meters() const { return stl_size_si_; }
    uint3 get_stl_size_cells() const { return stl_size_lbm_; } // without clearances
    float32_t get_reference_size_cells() const { return lbm_reference_size_; }
    float32_t get_reference_size_meters() const { return si_reference_size_; }
    float32_t get_scale_factor() const { return meters_per_cell_; }

    uint3 calculate_domain_size(const Clearances& clearances) const; // geometry + clearances in cells
    float3 calculate_center(const uint3& domain_size, const Clearances& clearances) const; // centered in X/Y, on the bottom clearance

private:
    string stl_path_;
    float3 stl_size_si_;      // m
    uint3 stl_size_lbm_;      // cells, without clearances

    float32_t lbm_reference_size_;
    float32_t si_reference_size_;
    float32_t meters_per_cell_;

    uint32_t vram_mb_;
    ReferenceAxis reference_axis_;
    Clearances clearances_;
    LatticeMemory lattice_;

    void read_stl_size();                  // stl_size_si_ from the file's bounding box
    void load_stl_and_calculate_scaling(); // VRAM budget mode
    void calculate_from_voxel_size(float32_t voxel_size_m, uint32_t max_vram_mb);
    float32_t calculate_min_voxel_size(uint32_t max_vram_mb) const; // for the error message
    float32_t get_reference_dimension(const float3& size) const;
};
