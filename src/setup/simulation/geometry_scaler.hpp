#pragma once
#include "core/types.hpp"
#include "utilities.hpp"

/**
 * @file geometry_scaler.hpp
 * @brief Unit conversion and scaling for STL geometry in FluidX3D
 *
 * This class encapsulates all conversions between SI units (meters) and
 * LBM lattice units (cells). It owns the relationship between real-world
 * geometry dimensions and simulation grid dimensions.
 */

/**
 * @class GeometryScaler
 * @brief Encapsulates all unit conversions between SI (meters) and LBM (lattice cells)
 *
 * This class owns the relationship between real-world geometry dimensions (meters)
 * and simulation grid dimensions (lattice cells). All spatial conversions go through
 * this class - no raw multiplications/divisions scattered in the code.
 *
 * @par Typical Usage:
 * @code
 * // 1. Create scaler from STL file and VRAM budget
 * GeometryScaler scaler("hill.stl", 1000);  // 1000 MB VRAM
 *
 * // 2. Use clean conversion methods
 * uint top_clearance_cells = scaler.meters_to_cells(500.0f);  // 500m → cells
 * float domain_height_m = scaler.cells_to_meters(lbm_Nz);      // cells → meters
 *
 * // 3. Get calculated dimensions
 * uint3 domain_size = scaler.get_stl_size_cells();  // Nx, Ny, Nz (no clearances)
 * float3 stl_size_m = scaler.get_stl_size_meters();  // Real-world STL dimensions
 * @endcode
 */
class GeometryScaler {
public:
    /**
     * @enum ReferenceAxis
     * @brief Defines which STL dimension to use as the reference for scaling
     */
    enum class ReferenceAxis {
        X,      ///< Use X dimension as reference
        Y,      ///< Use Y dimension as reference (default, typically flow direction)
        Z,      ///< Use Z dimension as reference
        MAX,    ///< Use the largest dimension as reference
        MIN     ///< Use the smallest dimension as reference
    };

    /**
     * @struct Clearances
     * @brief Clearance distances in meters (SI units)
     */
    struct Clearances {
        float32_t bottom_m;  ///< Bottom clearance in meters
        float32_t top_m;     ///< Top clearance in meters
        float32_t side_m;    ///< Side clearance in meters (applied to both X and Y)
    };

    /**
     * @brief Construct scaler from STL file path, VRAM budget, and clearances
     * @param stl_path Path to STL file (will be loaded to extract dimensions)
     * @param vram_mb VRAM budget in MB
     * @param clearances Domain clearances in meters (included in VRAM calculation)
     * @param reference_axis Which axis to use for scaling (default: Y)
     *
     * This constructor uses VRAM-based mode: calculates voxel size to fit
     * the TOTAL domain (STL + clearances) within the specified VRAM budget.
     */
    GeometryScaler(
        const string& stl_path,
        uint32_t vram_mb,
        const Clearances& clearances,
        ReferenceAxis reference_axis = ReferenceAxis::Y
    );

    /**
     * @brief Construct scaler from STL file path and voxel size in meters
     * @param stl_path Path to STL file (will be loaded to extract dimensions)
     * @param voxel_size_meters Voxel size in meters (e.g., 10.0 = 10m voxels)
     * @param max_vram_mb Maximum VRAM limit in MB for validation
     * @param reference_axis Which axis to use for scaling (default: Y)
     *
     * This constructor uses voxel-size mode: calculates grid resolution
     * directly from voxel size. Validates that required memory doesn't
     * exceed max_vram_mb (terminates with error if it does).
     */
    GeometryScaler(
        const string& stl_path,
        float32_t voxel_size_meters,
        uint32_t max_vram_mb,
        ReferenceAxis reference_axis = ReferenceAxis::Y
    );

    /**
     * @brief Convert meters to lattice cells
     * @param meters Distance in meters
     * @return Distance in lattice cells (rounded to nearest integer)
     *
     * This is the primary conversion method - use this instead of raw division.
     */
    uint32_t meters_to_cells(float32_t meters) const {
        return (uint32_t)(meters / meters_per_cell_ + 0.5f);
    }

    /**
     * @brief Convert lattice cells to meters
     * @param cells Distance in lattice cells
     * @return Distance in meters
     *
     * This is the primary conversion method - use this instead of raw multiplication.
     */
    float32_t cells_to_meters(uint32_t cells) const {
        return (float32_t)cells * meters_per_cell_;
    }

    /**
     * @brief Get STL bounding box size in meters
     * @return STL dimensions (X, Y, Z) in meters
     */
    float3 get_stl_size_meters() const { return stl_size_si_; }

    /**
     * @brief Get optimal lattice resolution for STL (without clearances)
     * @return Lattice dimensions (Nx, Ny, Nz) for the STL geometry
     */
    uint3 get_stl_size_cells() const { return stl_size_lbm_; }

    /**
     * @brief Get LBM reference size (the dimension used for scaling)
     * @return Reference size in lattice cells
     */
    float32_t get_reference_size_cells() const { return lbm_reference_size_; }

    /**
     * @brief Get SI reference size (the dimension used for scaling)
     * @return Reference size in meters
     */
    float32_t get_reference_size_meters() const { return si_reference_size_; }

    /**
     * @brief Get scale factor (meters per lattice cell)
     * @return Conversion factor: meters_per_cell
     *
     * This is exposed for reporting/debugging purposes only.
     * Prefer using meters_to_cells() and cells_to_meters() for conversions.
     */
    float32_t get_scale_factor() const { return meters_per_cell_; }

    /**
     * @brief Calculate domain size with clearances applied
     * @param clearances Clearances structure with bottom/top/side in METERS
     * @return Domain dimensions (Nx, Ny, Nz) including clearances
     *
     * This method handles the conversion from meters to cells internally.
     */
    uint3 calculate_domain_size(const Clearances& clearances) const;

    /**
     * @brief Calculate center position in lattice units
     * @param domain_size Total domain size (Nx, Ny, Nz)
     * @param clearances Clearances structure in meters
     * @param offset_m Offset in meters
     * @return Center position in lattice coordinates
     *
     * This method handles the conversion from meters to cells internally.
     */
    float3 calculate_center(
        const uint3& domain_size,
        const Clearances& clearances,
        const float3& offset_m
    ) const;

private:
    // STL geometry data
    string stl_path_;
    float3 stl_size_si_;      ///< STL dimensions in meters
    uint3 stl_size_lbm_;      ///< STL dimensions in lattice cells (no clearances)

    // Scaling factors
    float32_t lbm_reference_size_;  ///< Reference dimension in lattice cells
    float32_t si_reference_size_;   ///< Reference dimension in meters
    float32_t meters_per_cell_;     ///< Scale factor: meters / cell

    // Configuration
    uint32_t vram_mb_;
    ReferenceAxis reference_axis_;
    Clearances clearances_;  ///< Stored clearances for domain calculation

    /**
     * @brief Load STL and calculate all scaling factors (VRAM-based mode)
     *
     * Called by VRAM-based constructor. Loads STL, extracts dimensions,
     * calculates optimal lattice resolution from VRAM budget, and computes scale factor.
     */
    void load_stl_and_calculate_scaling();

    /**
     * @brief Calculate scaling from voxel size (voxel-size mode)
     * @param voxel_size_m Voxel size in meters
     * @param max_vram_mb Maximum VRAM limit for validation
     *
     * Called by voxel-size constructor. Calculates grid dimensions directly
     * from voxel size, validates against VRAM limit, and terminates with
     * error if limit is exceeded.
     */
    void calculate_from_voxel_size(float32_t voxel_size_m, uint32_t max_vram_mb);

    /**
     * @brief Calculate minimum voxel size needed to fit in given VRAM
     * @param max_vram_mb Maximum VRAM in MB
     * @return Minimum voxel size in meters
     *
     * Helper function for error messages. Computes the minimum voxel size
     * that would allow the geometry to fit within the specified VRAM limit.
     */
    float32_t calculate_min_voxel_size(uint32_t max_vram_mb) const;

    /**
     * @brief Get reference dimension from size based on reference axis
     * @param size Size vector (X, Y, Z)
     * @return Reference dimension value
     */
    float32_t get_reference_dimension(const float3& size) const;
};
