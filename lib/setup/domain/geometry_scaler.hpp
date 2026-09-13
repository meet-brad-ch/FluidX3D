#pragma once
#include "setup/core/types.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/lattice.hpp"
#include "utilities.hpp"

/// @brief Scale between an STL geometry in metres and the lattice (internal, for DomainPlanner's clearances mode).
///
/// The cell size comes from a VRAM budget (for the geometry plus its clearances) or is given; the domain size and the
/// geometry's center follow in cells. The geometry is measured after its rotation, as the core voxelizes it. The
/// reference length (units, Reynolds number) is its extent along Y.
/// @throws SetupError if the STL file does not exist or the grid does not fit the memory limit
class GeometryScaler {
public:
    /// The fluid around the geometry, in metres.
    struct Clearances {
        float32_t bottom_m; ///< below the geometry
        float32_t top_m;    ///< above it
        float32_t side_m;   ///< on each of the four sides
    };

    /// @brief The cell size so that the geometry with its clearances fits a VRAM budget.
    /// @param stl_path the STL file
    /// @param vram_mb the budget in MB
    /// @param clearances the fluid around the geometry
    /// @param lattice the memory per cell
    /// @param rotation the geometry's rotation
    GeometryScaler(
        const string& stl_path,
        uint32_t vram_mb,
        const Clearances& clearances,
        LatticeMemory lattice,
        const float3x3& rotation = float3x3(1.0f)
    );

    /// @brief A given cell size; the grid with its clearances must fit a memory limit.
    /// @param stl_path the STL file
    /// @param voxel_size_meters the cell size in metres
    /// @param max_vram_mb the memory limit in MB
    /// @param clearances the fluid around the geometry
    /// @param lattice the memory per cell
    /// @param rotation the geometry's rotation
    GeometryScaler(
        const string& stl_path,
        float32_t voxel_size_meters,
        uint32_t max_vram_mb,
        const Clearances& clearances,
        LatticeMemory lattice,
        const float3x3& rotation = float3x3(1.0f)
    );

    /// @param meters a length in metres
    /// @return the length in whole cells, rounded
    uint32_t meters_to_cells(float32_t meters) const {
        return (uint32_t)(meters / meters_per_cell_ + 0.5f);
    }

    /// @return the geometry's bounding box size in metres
    float3 get_stl_size_meters() const { return stl_size_si_; }

    /// @return the geometry's bounding box size in cells, without clearances
    uint3 get_stl_size_cells() const { return stl_size_lbm_; }

    /// @return the reference length in cells
    float32_t get_reference_size_cells() const { return lbm_reference_size_; }

    /// @return the reference length in metres
    float32_t get_reference_size_meters() const { return si_reference_size_; }

    /// @return the cell size in metres
    float32_t get_scale_factor() const { return meters_per_cell_; }

    /// @param clearances the fluid around the geometry
    /// @return the geometry plus the clearances, in cells
    uint3 calculate_domain_size(const Clearances& clearances) const;

    /// @param domain_size the domain in cells
    /// @param clearances the fluid around the geometry
    /// @return the geometry's center in cells: centered in X and Y, on the bottom clearance
    float3 calculate_center(const uint3& domain_size, const Clearances& clearances) const;

private:
    string stl_path_;         ///< the STL file
    float3 stl_size_si_;      ///< the geometry's size in metres
    uint3 stl_size_lbm_;      ///< the geometry's size in cells, without clearances

    float32_t lbm_reference_size_; ///< the reference length in cells
    float32_t si_reference_size_;  ///< the reference length in metres
    float32_t meters_per_cell_;    ///< the cell size

    uint32_t vram_mb_;        ///< the VRAM budget in MB
    Clearances clearances_;   ///< the fluid around the geometry
    LatticeMemory lattice_;   ///< the memory per cell
    float3x3 rotation_;       ///< the geometry's rotation

    /// Reads stl_size_si_ from the bounding box of the rotated geometry.
    void read_stl_size();

    /// The VRAM budget mode: the cell size so that the geometry with its clearances fits vram_mb_.
    void load_stl_and_calculate_scaling();

    /// @brief The given cell size mode.
    /// @param voxel_size_m the cell size in metres
    /// @param max_vram_mb the memory limit in MB
    /// @throws SetupError if the grid does not fit the limit
    void calculate_from_voxel_size(float32_t voxel_size_m, uint32_t max_vram_mb);

    /// @param max_vram_mb the memory limit in MB
    /// @return the smallest cell size that fits the limit, for the error message
    float32_t calculate_min_voxel_size(uint32_t max_vram_mb) const;

    /// @param size a geometry's size
    /// @return its extent along Y, the reference length
    static float32_t get_reference_dimension(const float3& size) { return size.y; }
};
