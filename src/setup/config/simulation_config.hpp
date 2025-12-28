#pragma once

#include "core/types.hpp"

/**
 * @class SimulationConfig
 * @brief Configuration class for STL geometry setup using fluent interface
 *
 * This class holds all configuration parameters for loading and positioning
 * STL geometries in FluidX3D simulations. It uses the fluent interface pattern
 * (method chaining) for easy and readable configuration.
 *
 * @par Basic Usage:
 * @code
 * // Minimal configuration - just the STL filename
 * SimulationConfig config("mesh.stl");
 * @endcode
 *
 * @par Advanced Usage:
 * @code
 * // Full configuration with method chaining
 * SimulationConfig config("mesh.stl")
 *     .set_vram_mb(4000u)                     // Use 4GB VRAM
 *     .set_rotation_deg(0.0f, 0.0f, 90.0f)    // Rotate 90 deg around Z
 *     .set_offset_m(100.0f, 0.0f, 0.0f)       // Offset 100m in X
 *     .set_clearances_m(2.0f, 10.0f, 5.0f)    // Custom clearances
 *     .set_reference_axis(SimulationConfig::ReferenceAxis::MAX);
 * @endcode
 *
 * @note All spatial offsets are in meters (SI units)
 * @note All rotations are in degrees
 * @note All clearances are in meters (SI units)
 */
class SimulationConfig {
public:
    /**
     * @enum ReferenceAxis
     * @brief Defines which STL dimension to use as the reference for scaling
     *
     * The reference axis determines which dimension of the STL bounding box
     * is used to calculate the LBM domain size and unit conversion.
     */
    enum class ReferenceAxis {
        X,      ///< Use X dimension as reference
        Y,      ///< Use Y dimension as reference (default, typically flow direction)
        Z,      ///< Use Z dimension as reference
        MAX,    ///< Use the largest dimension as reference
        MIN     ///< Use the smallest dimension as reference
    };

    /**
     * @enum ResolutionMode
     * @brief Defines how domain resolution is determined
     */
    enum class ResolutionMode {
        VRAM_BUDGET,  ///< Calculate resolution to fit within VRAM budget
        VOXEL_SIZE    ///< Calculate resolution from specified voxel size in meters
    };

    /**
     * @enum DomainMode
     * @brief Defines how domain size is determined
     */
    enum class DomainMode {
        GEOMETRY_BASED,  ///< Domain sized from STL geometry + clearances (default)
        ASPECT_RATIO,    ///< Domain sized from specified aspect ratio + VRAM budget
        DOMAIN_ONLY      ///< No geometry, domain sized from explicit dimensions in meters
    };

    /**
     * @enum VoxelizationMode
     * @brief Defines which voxelization kernel to use
     */
    enum class VoxelizationMode {
        SDF,  ///< Use SDF voxelization with automatic STL->SDF conversion (default, smoother surfaces)
        STL   ///< Use direct STL ray-triangle voxelization (faster, may have artifacts on complex geometry)
    };

    /**
     * @enum MirrorPlane
     * @brief Defines the plane across which to mirror symmetric geometry
     *
     * Use this for half-model STL files (e.g., symmetric aircraft) that need
     * to be mirrored to create the full geometry during voxelization.
     */
    enum class MirrorPlane {
        NONE,   ///< No mirroring (default)
        X,      ///< Mirror across X=0 plane (symmetric about X)
        Y,      ///< Mirror across Y=0 plane (symmetric about Y)
        Z       ///< Mirror across Z=0 plane (symmetric about Z)
    };

private:
    string geometry_filename;     ///< Geometry filename (STL or SDF, loaded from FluidX3D/stl/ directory)
    bool use_sdf { false };       ///< If true, use SDF instead of STL

    // Positioning (in meters, relative to simulation center)
    float32_t offset_x{};             ///< X offset in meters
    float32_t offset_y{};             ///< Y offset in meters
    float32_t offset_z{};             ///< Z offset from bottom in meters

    // Rotation (in degrees)
    float32_t rotation_x{};           ///< Rotation around X axis in degrees
    float32_t rotation_y{};           ///< Rotation around Y axis in degrees
    float32_t rotation_z{};           ///< Rotation around Z axis in degrees

    // Scaling reference
    ReferenceAxis reference_axis { ReferenceAxis::Y };  ///< Reference axis for scaling

    // Clearances (in meters - SI units)
    float32_t bottom_clearance_m{};       ///< Bottom clearance in meters
    float32_t top_clearance_m{};          ///< Top clearance in meters
    float32_t side_clearance_m{};         ///< Side clearance in meters

    // Domain resolution configuration
    ResolutionMode resolution_mode_ { ResolutionMode::VRAM_BUDGET };  ///< Resolution determination mode
    uint32_t vram_mb { 2000u };           ///< VRAM budget in MB (for VRAM_BUDGET mode)
    float32_t voxel_size_m_ { 0.0f };     ///< Voxel size in meters (for VOXEL_SIZE mode)
    uint32_t max_vram_mb_ { 24000u };     ///< Maximum VRAM limit in MB (default: 24GB)

    // Domain sizing mode
    DomainMode domain_mode_ { DomainMode::GEOMETRY_BASED };  ///< How domain size is determined
    float32_t aspect_x_ { 1.0f };         ///< X aspect ratio (for ASPECT_RATIO mode)
    float32_t aspect_y_ { 1.0f };         ///< Y aspect ratio (for ASPECT_RATIO mode)
    float32_t aspect_z_ { 1.0f };         ///< Z aspect ratio (for ASPECT_RATIO mode)
    float32_t geometry_scale_ { 1.0f };   ///< Geometry scale relative to reference axis (for ASPECT_RATIO mode)

    // Domain-only mode (no geometry)
    float32_t domain_size_x_m_ { 1.0f };  ///< Domain X size in meters (for DOMAIN_ONLY mode)
    float32_t domain_size_y_m_ { 1.0f };  ///< Domain Y size in meters (for DOMAIN_ONLY mode)
    float32_t domain_size_z_m_ { 1.0f };  ///< Domain Z size in meters (for DOMAIN_ONLY mode)

    // Aerodynamic settings
    float32_t angle_of_attack_deg_ { 0.0f };  ///< Angle of attack in degrees (pitch adjustment)

    // Center offset as ratio of geometry length
    float32_t center_offset_x_ { 0.0f };  ///< X offset as ratio of geometry reference length
    float32_t center_offset_y_ { 0.0f };  ///< Y offset as ratio of geometry reference length
    float32_t center_offset_z_ { 0.0f };  ///< Z offset as ratio of geometry reference length

    // Voxelization kernel selection
    VoxelizationMode voxelization_mode_ { VoxelizationMode::SDF };  ///< Voxelization kernel (default: SDF)

    // Mesh repair
    bool fix_mesh_ { false };     ///< Repair non-watertight meshes before SDF generation

    // Mesh mirroring for symmetric geometry
    MirrorPlane mirror_plane_ { MirrorPlane::NONE };  ///< Plane to mirror across (for half-models)

    // Verbose output
    bool verbose { true };        ///< Enable verbose console output

    friend class SimulationSetup;

public:
    /**
     * @brief Construct a new SimulationConfig object
     * @param geometry_file Geometry filename (STL or SDF, without path, loaded from FluidX3D/stl/)
     *
     * Creates a configuration with default parameters:
     * - No rotation or offset
     * - Y axis as reference
     * - 2 lattice unit clearances at top and bottom
     * - 2000 MB VRAM budget
     * - Verbose output enabled
     * - File type (STL or SDF) auto-detected from extension
     *
     * The file type is automatically determined:
     * - Files ending in ".sdf" → SDF mode
     * - All other files → STL mode
     */
    SimulationConfig(const string& geometry_file) : geometry_filename(geometry_file) {
        // Auto-detect file type from extension
        const size_t dot_pos = geometry_file.rfind('.');
        if(dot_pos != string::npos) {
            string extension = to_lower(geometry_file.substr(dot_pos));
            use_sdf = (extension == ".sdf");
        }
    }

    /**
     * @brief Construct a domain-only SimulationConfig (no geometry)
     *
     * Creates a configuration for simulations without STL geometry.
     * Use set_domain_size_m() to specify the domain dimensions.
     *
     * @par Example:
     * @code
     * SimulationConfig()
     *     .set_domain_size_m(1.0f, 6.0f, 2.0f)  // 1m x 6m x 2m domain
     *     .set_vram_mb(2000u);
     * @endcode
     */
    SimulationConfig() : geometry_filename(""), domain_mode_(DomainMode::DOMAIN_ONLY) {}

    /**
     * @brief Set domain size in meters for DOMAIN_ONLY mode
     * @param x_m Domain X dimension in meters
     * @param y_m Domain Y dimension in meters
     * @param z_m Domain Z dimension in meters
     * @return Reference to this object for method chaining
     *
     * @par Example:
     * @code
     * SimulationConfig()
     *     .set_domain_size_m(1.0f, 6.0f, 2.0f)  // 1m x 6m x 2m box
     *     .set_vram_mb(2000u);
     * @endcode
     *
     * @note Only used in DOMAIN_ONLY mode (created with default constructor)
     */
    SimulationConfig& set_domain_size_m(float32_t x_m, float32_t y_m, float32_t z_m) {
        domain_mode_ = DomainMode::DOMAIN_ONLY;
        domain_size_x_m_ = x_m;
        domain_size_y_m_ = y_m;
        domain_size_z_m_ = z_m;
        geometry_filename = "";  // Ensure no geometry
        return *this;
    }

    /**
     * @brief Override automatic file type detection
     * @param enable True to force SDF mode, false to force STL mode
     * @return Reference to this object for method chaining
     *
     * This method is optional - file type is automatically detected from the extension.
     * Use this only if you need to override the automatic detection.
     *
     * When SDF mode is enabled, the geometry file will be loaded as a binary SDF instead of STL.
     * SDF files avoid ray-triangle intersection artifacts and provide smoother surfaces.
     */
    SimulationConfig& use_sdf_mode(bool enable = true) {
        use_sdf = enable;
        return *this;
    }

    /**
     * @brief Set spatial offset in meters
     * @param x X offset in meters (positive = right, negative = left)
     * @param y Y offset in meters (positive = forward, negative = backward)
     * @param z Z offset in meters (positive = up from bottom, negative = down)
     * @return Reference to this object for method chaining
     *
     * Offsets are relative to the automatic centering. The geometry is first
     * centered in the domain, then the offset is applied.
     */
    SimulationConfig& set_offset_m(float32_t x, float32_t y, float32_t z) {
        offset_x = x; offset_y = y; offset_z = z;
        return *this;
    }

    /**
     * @brief Set rotation angles in degrees
     * @param x Rotation around X axis in degrees
     * @param y Rotation around Y axis in degrees
     * @param z Rotation around Z axis in degrees
     * @return Reference to this object for method chaining
     *
     * Rotations are applied in order: X, then Y, then Z.
     * Positive angles follow the right-hand rule.
     */
    SimulationConfig& set_rotation_deg(float32_t x, float32_t y, float32_t z) {
        rotation_x = x; rotation_y = y; rotation_z = z;
        return *this;
    }

    /**
     * @brief Set which STL dimension to use as reference
     * @param axis Reference axis (X, Y, Z, MAX, or MIN)
     * @return Reference to this object for method chaining
     *
     * The reference axis determines which dimension is used for:
     * - LBM domain size calculation
     * - Unit conversion between SI and LBM units
     *
     * Default is Y (typically the flow direction).
     */
    SimulationConfig& set_reference_axis(ReferenceAxis axis) {
        reference_axis = axis;
        return *this;
    }

    /**
     * @brief Set clearances in meters (SI units)
     * @param bottom_m Bottom clearance in meters
     * @param top_m Top clearance in meters
     * @param side_m Side clearance in meters (default: 0.0)
     * @return Reference to this object for method chaining
     *
     * All clearance values are in SI units (meters) and will be automatically
     * converted to lattice units internally based on the calculated domain resolution.
     *
     * Clearances add extra space around the geometry:
     * - Bottom: Space between z=0 and bottom of geometry (prevents boundary artifacts)
     * - Top: Space between top of geometry and domain ceiling
     * - Side: Extra space on all four sides (x and y boundaries)
     *
     * Example: .set_clearances_m(2.0, 500.0, 100.0) adds 2m bottom, 500m top, 100m sides
     */
    SimulationConfig& set_clearances_m(float32_t bottom_m, float32_t top_m, float32_t side_m = 0.0f) {
        bottom_clearance_m = bottom_m;
        top_clearance_m = top_m;
        side_clearance_m = side_m;
        return *this;
    }

    /**
     * @brief Set VRAM budget for domain size calculation
     * @param mb VRAM budget in megabytes
     * @return Reference to this object for method chaining
     *
     * The domain resolution is automatically calculated to fit within this
     * VRAM budget while maintaining the aspect ratio of the STL geometry.
     *
     * Typical values:
     * - 2000 MB (2 GB): Default, suitable for most GPUs
     * - 4000 MB (4 GB): For larger simulations
     * - 8000 MB (8 GB): For high-resolution simulations
     * - 16000 MB (16 GB): For very large simulations on high-end GPUs
     *
     * @note Actual VRAM usage may vary slightly due to overhead
     * @note This sets the resolution mode to VRAM_BUDGET
     */
    SimulationConfig& set_vram_mb(uint32_t mb) {
        resolution_mode_ = ResolutionMode::VRAM_BUDGET;
        vram_mb = mb;
        return *this;
    }

    /**
     * @brief Set voxel size in meters for direct resolution control
     * @param size_meters Size of each voxel in meters (e.g., 10.0 = 10m voxels)
     * @return Reference to this object for method chaining
     *
     * This mode calculates domain resolution directly from voxel size:
     * - Nx = STL_width_m / voxel_size_m
     * - Ny = STL_depth_m / voxel_size_m
     * - Nz = STL_height_m / voxel_size_m
     *
     * The calculated memory requirement is validated against max_vram_mb_
     * (default 24GB). Use set_max_vram() to change the limit.
     *
     * Typical values:
     * - 20.0 m: Coarse resolution for quick tests
     * - 10.0 m: Medium resolution for terrain simulations
     * - 5.0 m: High resolution (requires more VRAM)
     * - 2.0 m: Very high resolution (may require 24GB+ VRAM)
     *
     * @note This sets the resolution mode to VOXEL_SIZE
     * @note The method name includes "_m" to explicitly indicate meters
     */
    SimulationConfig& set_voxel_size_m(float32_t size_meters) {
        resolution_mode_ = ResolutionMode::VOXEL_SIZE;
        voxel_size_m_ = size_meters;
        return *this;
    }

    /**
     * @brief Set maximum VRAM limit for validation
     * @param mb Maximum VRAM in megabytes
     * @return Reference to this object for method chaining
     *
     * In VOXEL_SIZE mode, this limit is used to validate that the
     * calculated grid fits in available memory. If exceeded, an error
     * is thrown with a suggestion for minimum voxel size.
     *
     * In VRAM_BUDGET mode, this parameter is not used (the vram_mb
     * budget is the target, not a limit).
     *
     * Default: 24000 MB (24 GB)
     *
     * @note Only affects VOXEL_SIZE mode
     */
    SimulationConfig& set_max_vram_mb(uint32_t mb) {
        max_vram_mb_ = mb;
        return *this;
    }

    /**
     * @brief Enable or disable verbose console output
     * @param v True to enable verbose output, false to disable
     * @return Reference to this object for method chaining
     *
     * When enabled, prints:
     * - STL filename and dimensions
     * - Calculated domain size
     * - VRAM usage estimate
     */
    SimulationConfig& set_verbose(bool v) {
        verbose = v;
        return *this;
    }

    /**
     * @brief Set the voxelization kernel
     * @param mode VoxelizationMode::SDF (default) or VoxelizationMode::STL
     * @return Reference to this object for method chaining
     *
     * - SDF: Converts STL to Signed Distance Field, then voxelizes. Produces
     *   smoother surfaces but may have artifacts on thin features.
     * - STL: Direct ray-triangle voxelization. Faster, preserves thin features,
     *   but may have minor surface artifacts on complex geometry.
     *
     * @par Example:
     * @code
     * SimulationConfig("mesh.stl")
     *     .set_voxelization(SimulationConfig::VoxelizationMode::STL);  // Use direct STL
     * @endcode
     */
    SimulationConfig& set_voxelization(VoxelizationMode mode) {
        voxelization_mode_ = mode;
        return *this;
    }

    /**
     * @brief Enable or disable mesh repair for non-watertight meshes
     * @param fix True to attempt hole-filling on non-watertight meshes
     * @return Reference to this object for method chaining
     *
     * When enabled, meshes with holes or non-manifold edges are automatically
     * repaired before SDF generation using ear-clipping triangulation.
     * This prevents incorrect inside/outside determination that can occur
     * with non-watertight meshes.
     *
     * @par Example:
     * @code
     * SimulationConfig("airplane.stl")
     *     .set_fix_mesh(true);  // Repair any holes before SDF generation
     * @endcode
     *
     * @note Mesh analysis is always performed and reported (even if fix_mesh is false)
     */
    SimulationConfig& set_fix_mesh(bool fix) {
        fix_mesh_ = fix;
        return *this;
    }

    /**
     * @brief Enable mesh mirroring for symmetric geometry
     * @param plane The plane to mirror across (X, Y, or Z)
     * @return Reference to this object for method chaining
     *
     * Use this for half-model STL files (e.g., symmetric aircraft) that need
     * to be mirrored to create the full geometry during voxelization.
     *
     * The mesh is mirrored across the specified plane:
     * - MirrorPlane::X: Mirror across X=0 plane (left/right symmetry)
     * - MirrorPlane::Y: Mirror across Y=0 plane (front/back symmetry)
     * - MirrorPlane::Z: Mirror across Z=0 plane (top/bottom symmetry)
     *
     * @par Example:
     * @code
     * SimulationConfig("aircraft_half.stl")
     *     .set_mirror_plane(SimulationConfig::MirrorPlane::X);  // Create full aircraft
     * @endcode
     *
     * @note Requires ASPECT_RATIO domain mode
     * @note The half-model should have its symmetry plane at the mesh origin
     */
    SimulationConfig& set_mirror_plane(MirrorPlane plane) {
        mirror_plane_ = plane;
        return *this;
    }

    /**
     * @brief Set domain aspect ratio for ASPECT_RATIO sizing mode
     * @param x X aspect ratio component
     * @param y Y aspect ratio component
     * @param z Z aspect ratio component
     * @return Reference to this object for method chaining
     *
     * In ASPECT_RATIO mode, the domain dimensions are calculated from the
     * aspect ratio and VRAM budget, rather than from STL geometry.
     * The geometry is then scaled to fit within this domain.
     *
     * @par Example:
     * @code
     * SimulationConfig("plane.stl")
     *     .set_domain_aspect_ratio(1.0f, 3.0f, 0.5f)  // Elongated domain in Y
     *     .set_geometry_scale(0.56f)  // Geometry fills 56% of reference axis
     *     .set_vram_mb(2000u);
     * @endcode
     *
     * @note This sets domain_mode to ASPECT_RATIO
     * @note Must also call set_geometry_scale() to specify geometry size
     */
    SimulationConfig& set_domain_aspect_ratio(float32_t x, float32_t y, float32_t z) {
        domain_mode_ = DomainMode::ASPECT_RATIO;
        aspect_x_ = x;
        aspect_y_ = y;
        aspect_z_ = z;
        return *this;
    }

    /**
     * @brief Set geometry scale relative to domain reference axis
     * @param scale Scale factor (0.0 to 1.0, e.g., 0.56 = 56% of reference axis)
     * @return Reference to this object for method chaining
     *
     * In ASPECT_RATIO mode, this determines what fraction of the reference
     * axis dimension the geometry will occupy. For example, with Y reference
     * axis and scale 0.56, the geometry length will be 56% of Ny.
     *
     * @par Example:
     * @code
     * SimulationConfig("plane.stl")
     *     .set_domain_aspect_ratio(1.0f, 3.0f, 0.5f)
     *     .set_geometry_scale(0.56f)  // 56% of Y dimension
     *     .set_reference_axis(SimulationConfig::ReferenceAxis::Y);
     * @endcode
     */
    SimulationConfig& set_geometry_scale(float32_t scale) {
        geometry_scale_ = scale;
        return *this;
    }

    // ========================================================================
    // Semantic Domain Configuration (convenience methods)
    // ========================================================================

    /**
     * @brief Set domain to cubic proportions (1:1:1 aspect ratio)
     * @return Reference to this object for method chaining
     *
     * Convenience method for simulations where equal proportions in all
     * directions are desired.
     *
     * @par Example:
     * @code
     * SimulationConfig("sphere.stl")
     *     .set_domain_cubic()
     *     .set_geometry_occupancy(0.5f)
     *     .set_vram_mb(2000u);
     * @endcode
     */
    SimulationConfig& set_domain_cubic() {
        return set_domain_aspect_ratio(1.0f, 1.0f, 1.0f);
    }

    /**
     * @brief Set domain elongated in Y direction (typical for external flow)
     * @param y_factor Elongation factor for Y axis (default: 3.0)
     * @return Reference to this object for method chaining
     *
     * Creates a domain elongated in the Y direction, typical for aircraft
     * and vehicle simulations where flow is along Y axis.
     *
     * @par Example:
     * @code
     * SimulationConfig("aircraft.stl")
     *     .set_domain_elongated_y(3.0f)  // Y is 3x longer than X
     *     .set_geometry_occupancy(0.5f)
     *     .set_vram_mb(2000u);
     * @endcode
     */
    SimulationConfig& set_domain_elongated_y(float32_t y_factor = 3.0f) {
        return set_domain_aspect_ratio(1.0f, y_factor, 1.0f);
    }

    /**
     * @brief Set how much of the domain the geometry occupies
     * @param occupancy Fraction of reference axis occupied (0.0 to 1.0)
     * @return Reference to this object for method chaining
     *
     * Semantic alias for set_geometry_scale(). An occupancy of 0.5 means
     * the geometry fills 50% of the reference axis dimension.
     *
     * @par Example:
     * @code
     * SimulationConfig("car.stl")
     *     .set_domain_aspect_ratio(1.0f, 2.0f, 0.5f)
     *     .set_geometry_occupancy(0.6f)  // Geometry fills 60% of ref axis
     *     .set_vram_mb(2000u);
     * @endcode
     */
    SimulationConfig& set_geometry_occupancy(float32_t occupancy) {
        return set_geometry_scale(occupancy);
    }

    /**
     * @brief Set angle of attack for aerodynamic simulations
     * @param pitch_deg Angle of attack in degrees (positive = nose up)
     * @return Reference to this object for method chaining
     *
     * This adds an additional pitch rotation on top of the main rotation
     * settings. It's applied as the final rotation to set the aerodynamic
     * angle of attack relative to the flow direction.
     *
     * @par Example:
     * @code
     * SimulationConfig("aircraft.stl")
     *     .set_rotation_deg(90.0f, 0.0f, 90.0f)  // Orient along Y axis
     *     .set_angle_of_attack_deg(-10.0f);      // 10 degree nose-down
     * @endcode
     */
    SimulationConfig& set_angle_of_attack_deg(float32_t pitch_deg) {
        angle_of_attack_deg_ = pitch_deg;
        return *this;
    }

    /**
     * @brief Set center offset as ratio of geometry reference length
     * @param x X offset as ratio of geometry length
     * @param y Y offset as ratio of geometry length
     * @param z Z offset as ratio of geometry length
     * @return Reference to this object for method chaining
     *
     * Offsets the geometry center relative to the domain center, where
     * offset values are expressed as a fraction of the geometry's reference
     * length. Positive values move in the positive axis direction.
     *
     * @par Example:
     * @code
     * SimulationConfig("plane.stl")
     *     .set_center_offset_ratio(0.0f, 0.02f, 0.03f);  // Shift forward and up
     * @endcode
     *
     * @note Only used in ASPECT_RATIO mode
     */
    SimulationConfig& set_center_offset_ratio(float32_t x, float32_t y, float32_t z) {
        center_offset_x_ = x;
        center_offset_y_ = y;
        center_offset_z_ = z;
        return *this;
    }
};