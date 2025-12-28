#pragma once
#include "core/types.hpp"
#include "units.hpp"
#include "core/fluids.hpp"
#include "boundaries/boundary_flags.hpp"
#include <stdexcept>
#include <vector>

#include "config/simulation_config.hpp"
#include "simulation/geometry_scaler.hpp"
#include "sdf/sdf_generator.hpp"
#include "simulation/mesh_loader.hpp"

extern Units units;  // Global units object from lbm.cpp

/**
 * @file simulation_setup.hpp
 * @brief STL geometry handling utilities for FluidX3D simulations
 *
 * This file provides classes for automatic STL loading, positioning, scaling,
 * and domain sizing based on VRAM budget. Uses the fluent interface pattern
 * for easy configuration.
 */

/**
 * @class SimulationSetup
 * @brief Main class for STL geometry setup in FluidX3D simulations
 *
 * This class handles:
 * - Loading STL files and extracting dimensions
 * - Calculating optimal domain size based on VRAM budget
 * - Positioning and rotating geometry
 * - Unit conversion between SI and LBM units
 * - Voxelizing geometry to the LBM grid
 *
 * @par Typical Workflow:
 * @code
 * // 1. Configure geometry (validates STL file exists in constructor)
 * SimulationSetup sim(SimulationConfig("mesh.stl").set_vram(4000u));
 *
 * // 2. Setup (calculates domain size)
 * auto results = sim.setup();
 *
 * // 3. Optionally generate SDF separately (decoupled)
 * SDFGenerator sdf;
 * std::string sdf_path = sdf.generate(sim.get_stl_path(),
 *     results.base_grid.x, results.base_grid.y, results.base_grid.z);
 * sim.use_sdf(sdf_path);
 *
 * // 4. Configure units (SI velocity and density)
 * sim.configure_units(15.0f, 1.225f);  // si_velocity, si_density
 *
 * // 5. Create LBM simulation with converted viscosity
 * LBM lbm(results.Nx, results.Ny, results.Nz, sim.to_lbm_viscosity(1.48e-5f));
 *
 * // 6. Voxelize geometry
 * sim.voxelize(lbm);
 * @endcode
 *
 * @note The constructor validates that the STL file exists and terminates if not found
 * @note The setup() method must be called before configure_units() or voxelize()
 */
class SimulationSetup {
public:
    /**
     * @struct Results
     * @brief Contains all results from geometry setup
     *
     * This structure is returned by setup() and contains all information
     * needed to create the LBM simulation and set up unit conversion.
     */
    struct Results {
        float3 stl_size_si{};           ///< STL dimensions in meters (X, Y, Z)
        uint32_t Nx{}, Ny{}, Nz{};      ///< Domain dimensions in lattice units
        uint3 base_grid{};              ///< Base mesh grid (without clearances) for SDF generation
        float3 center_lbm{};            ///< Geometry center in LBM coordinates
        float3x3 rotation_matrix{};     ///< Rotation matrix for STL
        float32_t lbm_reference_size{}; ///< Reference dimension in LBM units
        float32_t si_reference_size{};  ///< Reference dimension in meters
    };

private:
    SimulationConfig config;        ///< Configuration parameters
    Results results;                ///< Setup results
    string resolved_geometry_path;  ///< Resolved path to geometry file (for cached SDFs)
    string original_stl_path_;      ///< Original STL path (for external SDF generation)
    bool units_configured_ = false; ///< Whether configure_units() has been called
    float32_t lbm_u_ref_ = 0.1f;    ///< Default LBM reference velocity
    bool force_tracking_ = false;   ///< Whether to use TYPE_X flag for force tracking

    // Reynolds number based simulation (dimensionless)
    bool reynolds_configured_ = false;  ///< Whether configure_reynolds() has been called
    float32_t reynolds_number_ = 0.0f;  ///< Reynolds number for Re-based simulation


    void validate_geometry_file() {
        const string geometry_path = get_resource_path(config.geometry_filename);
        if(geometry_path.empty()) {
            const string exe_path = get_exe_path();
            const string file_type = config.use_sdf ? "SDF" : "STL";
            std::cerr << "\n";
            std::cerr << "================================================================================\n";
            std::cerr << "FATAL ERROR: " << file_type << " file not found\n";
            std::cerr << "================================================================================\n";
            std::cerr << "File: " << config.geometry_filename << "\n\n";
            std::cerr << "Searched in:\n";
#ifdef FLUIDX3D_RESOURCE_DIR
            std::cerr << "  1. " << string(FLUIDX3D_RESOURCE_DIR) << "/" << config.geometry_filename << "\n";
#endif
            std::cerr << "  2. " << exe_path << "resources/" << config.geometry_filename << "\n\n";
            std::cerr << "Please ensure the file exists in one of these directories.\n";
            std::cerr << "================================================================================\n";
            std::cerr << std::endl;
            exit(1);
        }
    }

    float3x3 create_rotation_matrix() {
        const float3x3 Rx = float3x3(float3(1, 0, 0), radians(config.rotation_x));
        const float3x3 Ry = float3x3(float3(0, 1, 0), radians(config.rotation_y));
        const float3x3 Rz = float3x3(float3(0, 0, 1), radians(config.rotation_z));
        float3x3 base_rotation = Rz * Ry * Rx;

        // Apply angle of attack (additional pitch) if specified
        if(config.angle_of_attack_deg_ != 0.0f) {
            const float3x3 R_aoa = float3x3(float3(1, 0, 0), radians(config.angle_of_attack_deg_));
            return R_aoa * base_rotation;
        }
        return base_rotation;
    }

    uint32_t get_reference_axis_dimension(const uint3& dims) {
        switch(config.reference_axis) {
            case SimulationConfig::ReferenceAxis::X: return dims.x;
            case SimulationConfig::ReferenceAxis::Y: return dims.y;
            case SimulationConfig::ReferenceAxis::Z: return dims.z;
            case SimulationConfig::ReferenceAxis::MAX: return max(max(dims.x, dims.y), dims.z);
            case SimulationConfig::ReferenceAxis::MIN: return min(min(dims.x, dims.y), dims.z);
            default: return dims.y;
        }
    }

    /**
     * @brief Setup for ASPECT_RATIO domain mode
     *
     * In this mode, domain is sized from aspect ratio + VRAM budget,
     * and geometry is scaled to fit within the domain.
     */
    Results setup_aspect_ratio_mode() {
        // Use resolution() to calculate domain size from aspect ratio and VRAM
        const float3 aspect(config.aspect_x_, config.aspect_y_, config.aspect_z_);
        const uint3 lbm_N = resolution(aspect, config.vram_mb);

        results.Nx = lbm_N.x;
        results.Ny = lbm_N.y;
        results.Nz = lbm_N.z;

        // Calculate geometry reference length based on reference axis and scale
        const uint32_t ref_axis_size = get_reference_axis_dimension(lbm_N);
        results.lbm_reference_size = config.geometry_scale_ * (float32_t)ref_axis_size;

        // For aspect ratio mode, we don't have real STL dimensions in meters
        // We'll set a nominal SI reference size (can be overridden by configure_units)
        results.si_reference_size = 1.0f;  // Will be set properly by configure_units

        // Calculate center position with offsets
        // Base center is domain center
        float3 center = float3(
            0.5f * (float)lbm_N.x,
            0.5f * (float)lbm_N.y,
            0.5f * (float)lbm_N.z
        );

        // Apply center offsets as ratio of geometry length
        center.x += config.center_offset_x_ * results.lbm_reference_size;
        center.y += config.center_offset_y_ * results.lbm_reference_size;
        center.z += config.center_offset_z_ * results.lbm_reference_size;

        results.center_lbm = center;

        // Base grid is the geometry size (for SDF generation)
        // In aspect ratio mode, we estimate this from the geometry scale
        results.base_grid = uint3(
            (uint32_t)(config.geometry_scale_ * (float)lbm_N.x),
            (uint32_t)(config.geometry_scale_ * (float)lbm_N.y),
            (uint32_t)(config.geometry_scale_ * (float)lbm_N.z)
        );

        // Create rotation matrix (includes angle of attack)
        results.rotation_matrix = create_rotation_matrix();

        // Resolve geometry path (STL mode only for aspect ratio mode)
        resolved_geometry_path = get_resource_path(config.geometry_filename);
        config.use_sdf = false;  // Aspect ratio mode uses STL voxelization

        if(config.verbose) {
            print_info("Geometry: " + config.geometry_filename + " (aspect ratio mode)");
            print_info("Domain: Nx=" + to_string(results.Nx) + ", Ny=" + to_string(results.Ny) + ", Nz=" + to_string(results.Nz));
            print_info("Geometry scale: " + to_string(config.geometry_scale_ * 100.0f, 1u) + "% of reference axis");
            print_info("LBM reference size: " + to_string(results.lbm_reference_size, 1u) + " cells");
            print_info("VRAM usage: ~" + to_string(config.vram_mb) + " MB");
            if(config.angle_of_attack_deg_ != 0.0f) {
                print_info("Angle of attack: " + to_string(config.angle_of_attack_deg_, 1u) + " deg");
            }
        }

        return results;
    }

    float32_t get_reference_dimension(const float3& size) {
        switch(config.reference_axis) {
            case SimulationConfig::ReferenceAxis::X: return size.x;
            case SimulationConfig::ReferenceAxis::Y: return size.y;
            case SimulationConfig::ReferenceAxis::Z: return size.z;
            case SimulationConfig::ReferenceAxis::MAX: return fmax(fmax(size.x, size.y), size.z);
            case SimulationConfig::ReferenceAxis::MIN: return fmin(fmin(size.x, size.y), size.z);
            default: return size.y;
        }
    }

    /**
     * @brief Setup for DOMAIN_ONLY mode (no geometry)
     *
     * In this mode, domain is sized from explicit SI dimensions + VRAM budget.
     */
    Results setup_domain_only_mode() {
        // Calculate aspect ratio from SI dimensions
        const float max_dim = fmax(fmax(config.domain_size_x_m_, config.domain_size_y_m_), config.domain_size_z_m_);
        const float3 aspect(
            config.domain_size_x_m_ / max_dim,
            config.domain_size_y_m_ / max_dim,
            config.domain_size_z_m_ / max_dim
        );

        // Use resolution() to calculate domain size from aspect ratio and VRAM
        const uint3 lbm_N = resolution(aspect, config.vram_mb);

        results.Nx = lbm_N.x;
        results.Ny = lbm_N.y;
        results.Nz = lbm_N.z;

        // Calculate SI reference size (use max dimension)
        results.si_reference_size = max_dim;

        // Calculate LBM reference size (corresponding lattice dimension)
        results.lbm_reference_size = (float32_t)max(max(lbm_N.x, lbm_N.y), lbm_N.z);

        // Store SI dimensions
        results.stl_size_si = float3(config.domain_size_x_m_, config.domain_size_y_m_, config.domain_size_z_m_);

        // Center is domain center (no geometry offset)
        results.center_lbm = float3(
            0.5f * (float32_t)lbm_N.x,
            0.5f * (float32_t)lbm_N.y,
            0.5f * (float32_t)lbm_N.z
        );

        // No base grid (no geometry)
        results.base_grid = uint3(0, 0, 0);

        // Identity rotation (no geometry)
        results.rotation_matrix = float3x3(1.0f);

        if(config.verbose) {
            print_info("Domain-only simulation (no geometry)");
            print_info("SI dimensions: " + to_string(config.domain_size_x_m_) + "m x " +
                      to_string(config.domain_size_y_m_) + "m x " + to_string(config.domain_size_z_m_) + "m");
            print_info("Domain: Nx=" + to_string(results.Nx) + ", Ny=" + to_string(results.Ny) + ", Nz=" + to_string(results.Nz));
            print_info("VRAM usage: ~" + to_string(config.vram_mb) + " MB");
        }

        return results;
    }

public:
    SimulationSetup(const SimulationConfig& cfg) : config(cfg) {
        std::cout.flush();
        // Only validate geometry file if not in DOMAIN_ONLY mode
        if(config.domain_mode_ != SimulationConfig::DomainMode::DOMAIN_ONLY) {
            validate_geometry_file();
        }
        std::cout.flush();
    }

    Results setup() {
        // Handle DOMAIN_ONLY mode (no geometry)
        if(config.domain_mode_ == SimulationConfig::DomainMode::DOMAIN_ONLY) {
            return setup_domain_only_mode();
        }

        original_stl_path_ = get_resource_path(config.geometry_filename);

        // Handle ASPECT_RATIO mode separately
        if(config.domain_mode_ == SimulationConfig::DomainMode::ASPECT_RATIO) {
            return setup_aspect_ratio_mode();
        }

        // GEOMETRY_BASED mode (default) - domain sized from STL + clearances
        GeometryScaler::ReferenceAxis scaler_axis;
        switch(config.reference_axis) {
            case SimulationConfig::ReferenceAxis::X: scaler_axis = GeometryScaler::ReferenceAxis::X; break;
            case SimulationConfig::ReferenceAxis::Y: scaler_axis = GeometryScaler::ReferenceAxis::Y; break;
            case SimulationConfig::ReferenceAxis::Z: scaler_axis = GeometryScaler::ReferenceAxis::Z; break;
            case SimulationConfig::ReferenceAxis::MAX: scaler_axis = GeometryScaler::ReferenceAxis::MAX; break;
            case SimulationConfig::ReferenceAxis::MIN: scaler_axis = GeometryScaler::ReferenceAxis::MIN; break;
            default: scaler_axis = GeometryScaler::ReferenceAxis::Y; break;
        }

        // Build clearances first - they're needed for VRAM calculation
        GeometryScaler::Clearances clearances;
        clearances.bottom_m = config.bottom_clearance_m;
        clearances.top_m = config.top_clearance_m;
        clearances.side_m = config.side_clearance_m;

        // Create scaler with clearances so VRAM budget applies to TOTAL domain
        GeometryScaler scaler = (config.resolution_mode_ == SimulationConfig::ResolutionMode::VOXEL_SIZE)
            ? GeometryScaler(original_stl_path_, config.voxel_size_m_, config.max_vram_mb_, scaler_axis)
            : GeometryScaler(original_stl_path_, config.vram_mb, clearances, scaler_axis);

        results.base_grid = scaler.get_stl_size_cells();

        // Voxelization mode determines how geometry is processed
        if(config.voxelization_mode_ == SimulationConfig::VoxelizationMode::SDF) {
            // SDF mode: convert STL to SDF for smoother voxelization
            if(!config.use_sdf && config.geometry_filename.find(".stl") != string::npos) {
                SDFGenerator sdf_gen;
                sdf_gen.set_cache_dir("resources/sdf_cache/")
                       .enable_cache(true)
                       .set_fix_mesh(config.fix_mesh_)
                       .set_verbose(config.verbose);

                // Generate or retrieve cached SDF at exact base grid dimensions
                std::string sdf_path = sdf_gen.generate(
                    original_stl_path_,
                    results.base_grid.x,
                    results.base_grid.y,
                    results.base_grid.z,
                    1  // padding
                );

                if(!sdf_path.empty()) {
                    // Successfully got cached SDF - use it instead of STL
                    resolved_geometry_path = sdf_path;
                    config.use_sdf = true;
                    if(config.verbose) {
                        print_info("Using SDF voxelization (cached: " + sdf_path + ")");
                    }
                } else {
                    // SDF generation failed - fall back to STL
                    resolved_geometry_path = get_resource_path(config.geometry_filename);
                    if(config.verbose) {
                        print_info("SDF caching failed, falling back to STL voxelization");
                    }
                }
            } else if(config.use_sdf && config.geometry_filename.find("sdf_cache") != string::npos) {
                resolved_geometry_path = config.geometry_filename;
            } else {
                resolved_geometry_path = get_resource_path(config.geometry_filename);
            }
        } else {
            // STL mode: direct ray-triangle voxelization
            resolved_geometry_path = get_resource_path(config.geometry_filename);
            config.use_sdf = false;
            if(config.verbose) {
                print_info("Using STL voxelization (direct ray-triangle)");
            }
        }

        results.stl_size_si = scaler.get_stl_size_meters();
        results.lbm_reference_size = scaler.get_reference_size_cells();
        results.si_reference_size = scaler.get_reference_size_meters();

        // Use clearances already defined above
        uint3 domain_size = scaler.calculate_domain_size(clearances);
        results.Nx = domain_size.x;
        results.Ny = domain_size.y;
        results.Nz = domain_size.z;

        float3 offset_m(config.offset_x, config.offset_y, config.offset_z);
        results.center_lbm = scaler.calculate_center(domain_size, clearances, offset_m);

        results.rotation_matrix = create_rotation_matrix();

        if(config.verbose) {
            print_info("Geometry: " + config.geometry_filename);
            print_info("Dimensions: X=" + to_string(results.stl_size_si.x) + "m, Y=" + to_string(results.stl_size_si.y) + "m, Z=" + to_string(results.stl_size_si.z) + "m");
            print_info("Scale: " + to_string(scaler.get_scale_factor()) + " m/cell");

            uint32_t side_cells = scaler.meters_to_cells(clearances.side_m);
            uint32_t bottom_cells = scaler.meters_to_cells(clearances.bottom_m);
            uint32_t top_cells = scaler.meters_to_cells(clearances.top_m);

            print_info("Base grid (mesh only): " + to_string(results.base_grid.x) + " x " + to_string(results.base_grid.y) + " x " + to_string(results.base_grid.z));
            print_info("Domain: Nx=" + to_string(results.Nx) + ", Ny=" + to_string(results.Ny) + ", Nz=" + to_string(results.Nz));
            print_info("VRAM usage: ~" + to_string(config.vram_mb) + " MB");
        }

        return results;
    }

    SimulationSetup& use_sdf(const std::string& sdf_path) {
        resolved_geometry_path = sdf_path;
        config.use_sdf = true;
        return *this;
    }

    const string& get_stl_path() const { return original_stl_path_; }

    /**
     * @brief Get the geometry filename from configuration
     * @return Geometry filename (as passed to SimulationConfig constructor)
     *
     * Use this to get the filename for loading additional parts that should
     * use the same geometry as the main body (e.g., for tumbling simulations).
     */
    const string& get_geometry_filename() const { return config.geometry_filename; }

    /**
     * @brief Get the scale factor (cells per meter) for mesh transformations
     *
     * This is the factor to convert from SI meters to LBM cells.
     * Use this when loading additional meshes (e.g., moving parts) that
     * should be scaled consistently with the main geometry.
     *
     * @return Scale factor in cells per meter
     */
    float32_t get_mesh_scale_factor() const {
        return results.lbm_reference_size / results.si_reference_size;
    }

    /**
     * @brief Enable force tracking for aerodynamic analysis
     *
     * When enabled, voxelize() will use TYPE_S|TYPE_X flags instead of just TYPE_S.
     * This allows ForceAnalyzer to calculate forces on the voxelized geometry.
     *
     * @return Reference for method chaining
     *
     * @note Must be called before voxelize()
     * @note Requires FORCE_FIELD extension to be enabled in defines.hpp
     */
    SimulationSetup& enable_force_tracking() {
        force_tracking_ = true;
        return *this;
    }

    /**
     * @brief Check if force tracking is enabled
     * @return True if enable_force_tracking() was called
     */
    bool is_force_tracking_enabled() const { return force_tracking_; }

    void voxelize(LBM& lbm) {
        // Use TYPE_S|TYPE_X (0x41) if force tracking enabled, otherwise TYPE_S (0x01)
        const uchar voxel_flag = force_tracking_ ? 0x41 : 0x01;

        // Handle mirrored geometry (for symmetric half-models)
        if(config.mirror_plane_ != SimulationConfig::MirrorPlane::NONE) {
            Mesh* mesh = load_mesh_mirrored();
            lbm.voxelize_mesh_on_device(mesh, voxel_flag);
            delete mesh;
            return;
        }

        // Determine scaling size based on domain mode
        float voxel_size;
        if(config.domain_mode_ == SimulationConfig::DomainMode::ASPECT_RATIO) {
            // In aspect ratio mode, use lbm_reference_size directly
            // This matches original behavior: size = geometry_scale * domain_ref_axis
            voxel_size = results.lbm_reference_size;
        } else {
            // In geometry-based mode, use max dimension of base_grid
            // Both STL and SDF voxelization expect the scale/size to be the LONGEST dimension
            voxel_size = fmax(fmax((float)results.base_grid.x, (float)results.base_grid.y), (float)results.base_grid.z);
        }

        if(config.use_sdf) {
            // SDF voxelization: scale parameter is divided by sdf_max_world_dim in kernel
            lbm.voxelize_sdf(resolved_geometry_path, results.center_lbm, results.rotation_matrix, voxel_size, voxel_flag);
        } else {
            // STL voxelization: size parameter is the LONGEST dimension (see read_stl in utilities.hpp)
            lbm.voxelize_stl(resolved_geometry_path, results.center_lbm, results.rotation_matrix, voxel_size, voxel_flag);
        }
    }

    const Results& get_results() const { return results; }

    // ========================================================================
    // Unit Conversion API
    // ========================================================================

    /**
     * @brief Configure unit conversion from SI to LBM units
     *
     * This method sets up the global `units` object for converting between
     * SI units (meters, m/s, kg/m³) and LBM simulation units.
     *
     * @param si_velocity Reference velocity in m/s (e.g., inlet velocity)
     * @param si_density Fluid density in kg/m³ (default: 1.225 for air at STP)
     * @param lbm_u Reference velocity in LBM units (default: 0.1, safe for stability)
     * @return Reference to this object for method chaining
     *
     * @note Must be called after setup()
     *
     * @par Example:
     * @code
     * sim.configure_units(15.0f, 1.225f);  // 15 m/s, air density
     * float lbm_nu = sim.to_lbm_viscosity(1.48e-5f);  // Convert SI viscosity to LBM
     * @endcode
     */
    SimulationSetup& configure_units(float32_t si_velocity, float32_t si_density = 1.225f, float32_t lbm_u = 0.1f) {
        lbm_u_ref_ = lbm_u;
        units.set_m_kg_s(
            results.lbm_reference_size,  // LBM reference length
            lbm_u,                       // LBM reference velocity
            1.0f,                        // LBM reference density (always 1)
            results.si_reference_size,   // SI reference length (meters)
            si_velocity,                 // SI reference velocity (m/s)
            si_density                   // SI reference density (kg/m³)
        );
        units_configured_ = true;
        return *this;
    }

    /**
     * @brief Configure unit conversion using predefined fluid properties
     *
     * @param si_velocity Reference velocity in m/s
     * @param fluid Fluid properties (e.g., Fluid::AIR, Fluid::WATER)
     * @param lbm_u Reference velocity in LBM units (default: 0.1)
     * @return Reference to this object for method chaining
     *
     * @par Example:
     * @code
     * sim.configure_units(15.0f, Fluid::AIR);
     * @endcode
     */
    SimulationSetup& configure_units(float32_t si_velocity, const FluidProperties& fluid, float32_t lbm_u = 0.1f) {
        return configure_units(si_velocity, fluid.density, lbm_u);
    }

    /**
     * @brief Configure unit conversion with explicit SI reference length
     *
     * Use this overload in ASPECT_RATIO mode where the SI reference length
     * is not automatically determined from STL geometry.
     *
     * @param si_reference_length Reference length in meters (e.g., aircraft length)
     * @param si_velocity Reference velocity in m/s
     * @param si_density Fluid density in kg/m³
     * @param lbm_u Reference velocity in LBM units (default: 0.1)
     * @return Reference to this object for method chaining
     *
     * @par Example:
     * @code
     * sim.configure_units_with_length(62.0f, 83.0f, Fluid::AIR.density);  // 62m Concorde
     * @endcode
     */
    SimulationSetup& configure_units_with_length(float32_t si_reference_length, float32_t si_velocity,
                                                  float32_t si_density = 1.225f, float32_t lbm_u = 0.1f) {
        results.si_reference_size = si_reference_length;
        return configure_units(si_velocity, si_density, lbm_u);
    }

    /**
     * @brief Configure unit conversion with explicit SI reference length using fluid properties
     *
     * @param si_reference_length Reference length in meters
     * @param si_velocity Reference velocity in m/s
     * @param fluid Fluid properties (e.g., Fluid::AIR, Fluid::WATER)
     * @param lbm_u Reference velocity in LBM units (default: 0.1)
     * @return Reference to this object for method chaining
     */
    SimulationSetup& configure_units_with_length(float32_t si_reference_length, float32_t si_velocity,
                                                  const FluidProperties& fluid, float32_t lbm_u = 0.1f) {
        return configure_units_with_length(si_reference_length, si_velocity, fluid.density, lbm_u);
    }

    /**
     * @brief Get reference to the global units object
     * @return Reference to the global Units object
     * @note configure_units() should be called first
     */
    Units& get_units() { return units; }
    const Units& get_units() const { return units; }

    /**
     * @brief Check if units have been configured
     * @return True if configure_units() has been called
     */
    bool is_units_configured() const { return units_configured_; }

    // ========================================================================
    // SI to LBM Conversion Helpers
    // ========================================================================

    /**
     * @brief Convert SI kinematic viscosity to LBM units
     * @param si_viscosity Kinematic viscosity in m²/s (e.g., 1.48e-5 for air)
     * @return Viscosity in LBM units for LBM constructor
     */
    float32_t to_lbm_viscosity(float32_t si_viscosity) const { return units.nu(si_viscosity); }

    /**
     * @brief Convert SI velocity to LBM units
     * @param si_velocity Velocity in m/s
     * @return Velocity in LBM units
     */
    float32_t to_lbm_velocity(float32_t si_velocity) const { return units.u(si_velocity); }

    /**
     * @brief Convert SI length to LBM units
     * @param si_length Length in meters
     * @return Length in LBM units (cells)
     */
    float32_t to_lbm_length(float32_t si_length) const { return units.x(si_length); }

    /**
     * @brief Convert SI force per volume to LBM units
     * @param si_force_per_volume Force per volume in kg/(m·s²)
     * @return Force per volume in LBM units
     */
    float32_t to_lbm_force_per_volume(float32_t si_force_per_volume) const { return units.f(si_force_per_volume); }

    /**
     * @brief Convert SI density and gravitational acceleration to LBM force per volume
     * @param si_density Density in kg/m³
     * @param si_gravity Gravitational acceleration in m/s²
     * @return Force per volume in LBM units
     */
    float32_t to_lbm_gravity_force(float32_t si_density, float32_t si_gravity) const { return units.f(si_density, si_gravity); }

    /**
     * @brief Convert time in seconds to LBM timesteps
     * @param si_seconds Time in seconds
     * @return Number of LBM timesteps
     */
    uint64_t to_lbm_timesteps(float32_t si_seconds) const { return units.t(si_seconds); }

    // ========================================================================
    // LBM to SI Conversion Helpers
    // ========================================================================

    /**
     * @brief Convert LBM velocity to SI units
     * @param lbm_velocity Velocity in LBM units
     * @return Velocity in m/s
     */
    float32_t to_si_velocity(float32_t lbm_velocity) const { return units.si_u(lbm_velocity); }

    /**
     * @brief Convert LBM force to SI units
     * @param lbm_force Force in LBM units
     * @return Force in Newtons (kg·m/s²)
     */
    float32_t to_si_force(float32_t lbm_force) const { return units.si_F(lbm_force); }

    /**
     * @brief Convert LBM pressure to SI units
     * @param lbm_pressure Pressure in LBM units
     * @return Pressure in Pascals (kg/(m·s²))
     */
    float32_t to_si_pressure(float32_t lbm_pressure) const { return units.si_p(lbm_pressure); }

    /**
     * @brief Get the LBM reference velocity used in configure_units
     * @return LBM reference velocity
     */
    float32_t get_lbm_reference_velocity() const { return lbm_u_ref_; }

    // ========================================================================
    // LBM Creation Helpers
    // ========================================================================

    /**
     * @brief Create an LBM simulation with the configured domain size
     *
     * Creates an LBM object with dimensions from setup() and viscosity
     * converted from SI units.
     *
     * @param si_kinematic_viscosity Kinematic viscosity in m²/s
     * @return LBM simulation object
     *
     * @note Must be called after setup() and configure_units()
     *
     * @par Example:
     * @code
     * LBM lbm = sim.create_lbm(1.48e-5f);  // Air viscosity
     * @endcode
     */
    LBM create_lbm(float32_t si_kinematic_viscosity) {
        return LBM(results.Nx, results.Ny, results.Nz, to_lbm_viscosity(si_kinematic_viscosity));
    }

    /**
     * @brief Create an LBM simulation using predefined fluid properties
     *
     * @param fluid Fluid properties (e.g., Fluid::AIR, Fluid::WATER)
     * @return LBM simulation object
     *
     * @par Example:
     * @code
     * LBM lbm = sim.create_lbm(Fluid::AIR);
     * @endcode
     */
    LBM create_lbm(const FluidProperties& fluid) {
        return create_lbm(fluid.kinematic_viscosity);
    }

    // ========================================================================
    // Thermal Simulation (TEMPERATURE extension)
    // ========================================================================

    /**
     * @brief Create an LBM simulation with thermal parameters from SI units
     *
     * Creates an LBM object configured for thermal simulations with the
     * TEMPERATURE extension. All parameters are specified in SI units.
     *
     * @param si_kinematic_viscosity Kinematic viscosity in m²/s
     * @param si_thermal_diffusivity Thermal diffusivity in m²/s
     * @param si_thermal_expansion Thermal expansion coefficient in 1/K
     * @param si_gravity Gravitational acceleration in m/s² (positive = downward)
     * @param gravity_axis Axis for gravity direction (default: Z)
     * @return LBM simulation object
     *
     * @note Requires TEMPERATURE and VOLUME_FORCE extensions in defines.hpp
     *
     * @par Example:
     * @code
     * // Air at 300K: nu=1.5e-5 m²/s, alpha=2.2e-5 m²/s, beta=1/300 K⁻¹
     * LBM lbm = sim.create_lbm_thermal(1.5e-5f, 2.2e-5f, 0.00333f, 9.81f, Axis::Z);
     * @endcode
     */
    LBM create_lbm_thermal(float32_t si_kinematic_viscosity,
                           float32_t si_thermal_diffusivity,
                           float32_t si_thermal_expansion,
                           float32_t si_gravity,
                           Axis gravity_axis = Axis::Z) {
        // Convert SI parameters to LBM units
        const float32_t lbm_nu = to_lbm_viscosity(si_kinematic_viscosity);
        const float32_t lbm_alpha = to_lbm_viscosity(si_thermal_diffusivity);  // Same conversion

        // Thermal expansion: beta_lbm = beta_si * T_si / T_lbm
        // For dimensionless temperature (T_lbm ~ 1), and typical T_si ~ 300K
        // We use a reference temperature ratio to convert
        const float32_t T_ref_si = 300.0f;  // Reference temperature in Kelvin
        const float32_t T_ref_lbm = 1.0f;   // Reference temperature in LBM units
        const float32_t lbm_beta = si_thermal_expansion * T_ref_si / T_ref_lbm;

        // Gravity: convert m/s² to LBM force per unit mass
        // g_lbm = g_si * (dt²/dx) = g_si * (t_si/t_lbm)² / (x_si/x_lbm)
        const float32_t lbm_gravity = to_lbm_force_per_volume(si_gravity);

        // Set gravity components based on axis
        float32_t fx = 0.0f, fy = 0.0f, fz = 0.0f;
        switch (gravity_axis) {
            case Axis::X: fx = -lbm_gravity; break;
            case Axis::Y: fy = -lbm_gravity; break;
            case Axis::Z: fz = -lbm_gravity; break;
        }

        // Create LBM with thermal parameters (single GPU)
        return LBM(results.Nx, results.Ny, results.Nz, 1u, 1u, 1u,
                   lbm_nu, fx, fy, fz, 0.0f, lbm_alpha, lbm_beta);
    }

    /**
     * @brief Create thermal LBM using predefined fluid properties
     *
     * @param fluid Fluid properties (e.g., Fluid::AIR, Fluid::WATER)
     * @param si_gravity Gravitational acceleration in m/s²
     * @param gravity_axis Gravity direction axis
     * @return LBM simulation object
     *
     * @par Example:
     * @code
     * LBM lbm = sim.create_lbm_thermal(Fluid::AIR, 9.81f, Axis::Z);
     * @endcode
     */
    LBM create_lbm_thermal(const FluidProperties& fluid,
                           float32_t si_gravity = 9.81f,
                           Axis gravity_axis = Axis::Z) {
        return create_lbm_thermal(
            fluid.kinematic_viscosity,
            fluid.thermal_diffusivity,
            fluid.thermal_expansion,
            si_gravity,
            gravity_axis
        );
    }

    // ========================================================================
    // Free Surface Simulation (SURFACE extension)
    // ========================================================================

    /**
     * @brief Create an LBM simulation for free surface flows
     *
     * Creates an LBM object configured for free surface simulations with the
     * SURFACE extension. Includes gravity for multiphase flows.
     *
     * @param si_kinematic_viscosity Kinematic viscosity in m²/s
     * @param si_gravity Gravitational acceleration in m/s² (positive = downward)
     * @param si_surface_tension Surface tension coefficient in N/m (0 to disable)
     * @param gravity_axis Axis for gravity direction (default: Z)
     * @return LBM simulation object
     *
     * @note Requires SURFACE and VOLUME_FORCE extensions in defines.hpp
     *
     * @par Example:
     * @code
     * LBM lbm = sim.create_lbm_surface(Fluid::WATER.kinematic_viscosity, 9.81f);
     * @endcode
     */
    LBM create_lbm_surface(float32_t si_kinematic_viscosity,
                           float32_t si_gravity = 9.81f,
                           float32_t si_surface_tension = 0.0f,
                           Axis gravity_axis = Axis::Z) {
        const float32_t lbm_nu = to_lbm_viscosity(si_kinematic_viscosity);
        const float32_t lbm_gravity = to_lbm_force_per_volume(si_gravity);

        // Convert surface tension: sigma_lbm = sigma_si / (rho * u² * L)
        // For simplicity, use a typical small value if non-zero
        float32_t lbm_sigma = 0.0f;
        if (si_surface_tension > 0.0f) {
            lbm_sigma = 0.0001f;  // Typical stable value for LBM
        }

        float32_t fx = 0.0f, fy = 0.0f, fz = 0.0f;
        switch (gravity_axis) {
            case Axis::X: fx = -lbm_gravity; break;
            case Axis::Y: fy = -lbm_gravity; break;
            case Axis::Z: fz = -lbm_gravity; break;
        }

        return LBM(results.Nx, results.Ny, results.Nz, lbm_nu, fx, fy, fz, lbm_sigma);
    }

    /**
     * @brief Create free surface LBM using predefined fluid properties
     *
     * @param fluid Fluid properties (e.g., Fluid::WATER)
     * @param si_gravity Gravitational acceleration in m/s²
     * @param si_surface_tension Surface tension in N/m (default: 0)
     * @param gravity_axis Gravity direction axis
     * @return LBM simulation object
     *
     * @par Example:
     * @code
     * LBM lbm = sim.create_lbm_surface(Fluid::WATER, 9.81f);
     * @endcode
     */
    LBM create_lbm_surface(const FluidProperties& fluid,
                           float32_t si_gravity = 9.81f,
                           float32_t si_surface_tension = 0.0f,
                           Axis gravity_axis = Axis::Z) {
        return create_lbm_surface(fluid.kinematic_viscosity, si_gravity,
                                  si_surface_tension, gravity_axis);
    }

    // ========================================================================
    // Particle Simulation (PARTICLES extension)
    // ========================================================================

    /**
     * @brief Create an LBM simulation with particle tracking
     *
     * Creates an LBM object configured for particle simulations with the
     * PARTICLES extension. Includes optional gravity for particle settling.
     *
     * @param si_kinematic_viscosity Kinematic viscosity in m²/s
     * @param particle_count Number of particles to allocate
     * @param particle_density Relative density of particles (1.0 = neutral, >1 = sink, <1 = float)
     * @param si_gravity Gravitational acceleration in m/s² (0 to disable)
     * @param gravity_axis Axis for gravity direction (default: Z)
     * @return LBM simulation object
     *
     * @note Requires PARTICLES extension in defines.hpp
     * @note For force coupling, also requires VOLUME_FORCE and FORCE_FIELD
     *
     * @par Example:
     * @code
     * LBM lbm = sim.create_lbm_particles(Fluid::WATER.kinematic_viscosity, 10000, 2.0f);
     * @endcode
     */
    LBM create_lbm_particles(float32_t si_kinematic_viscosity,
                              uint32_t particle_count,
                              float32_t particle_density = 1.0f,
                              float32_t si_gravity = 0.0f,
                              Axis gravity_axis = Axis::Z) {
        const float32_t lbm_nu = to_lbm_viscosity(si_kinematic_viscosity);

        float32_t fx = 0.0f, fy = 0.0f, fz = 0.0f;
        if (si_gravity > 0.0f) {
            const float32_t lbm_gravity = to_lbm_force_per_volume(si_gravity);
            switch (gravity_axis) {
                case Axis::X: fx = -lbm_gravity; break;
                case Axis::Y: fy = -lbm_gravity; break;
                case Axis::Z: fz = -lbm_gravity; break;
            }
        }

        return LBM(results.Nx, results.Ny, results.Nz, lbm_nu, fx, fy, fz,
                   particle_count, particle_density);
    }

    /**
     * @brief Create particle LBM using predefined fluid properties
     *
     * @param fluid Fluid properties (e.g., Fluid::WATER)
     * @param particle_count Number of particles to allocate
     * @param particle_density Relative density of particles
     * @param si_gravity Gravitational acceleration in m/s²
     * @param gravity_axis Gravity direction axis
     * @return LBM simulation object
     *
     * @par Example:
     * @code
     * LBM lbm = sim.create_lbm_particles(Fluid::WATER, 10000, 2.0f, 9.81f);
     * @endcode
     */
    LBM create_lbm_particles(const FluidProperties& fluid,
                              uint32_t particle_count,
                              float32_t particle_density = 1.0f,
                              float32_t si_gravity = 0.0f,
                              Axis gravity_axis = Axis::Z) {
        return create_lbm_particles(fluid.kinematic_viscosity, particle_count,
                                    particle_density, si_gravity, gravity_axis);
    }

    /**
     * @brief Create particle LBM for Reynolds number based simulation
     *
     * @param reynolds Reynolds number
     * @param particle_count Number of particles to allocate
     * @param particle_density Relative density of particles
     * @param si_gravity Gravitational acceleration in m/s²
     * @param gravity_axis Gravity direction axis
     * @param lbm_u Reference velocity in LBM units
     * @return LBM simulation object
     *
     * @par Example:
     * @code
     * LBM lbm = sim.create_lbm_particles_reynolds(1000.0f, 10000, 2.0f);
     * @endcode
     */
    LBM create_lbm_particles_reynolds(float32_t reynolds,
                                       uint32_t particle_count,
                                       float32_t particle_density = 1.0f,
                                       float32_t si_gravity = 0.0f,
                                       Axis gravity_axis = Axis::Z,
                                       float32_t lbm_u = 0.1f) {
        const float32_t lbm_nu = units.nu_from_Re(reynolds, (float32_t)results.Nx, lbm_u);
        lbm_u_ref_ = lbm_u;

        float32_t fx = 0.0f, fy = 0.0f, fz = 0.0f;
        if (si_gravity > 0.0f) {
            const float32_t lbm_gravity = to_lbm_force_per_volume(si_gravity);
            switch (gravity_axis) {
                case Axis::X: fx = -lbm_gravity; break;
                case Axis::Y: fy = -lbm_gravity; break;
                case Axis::Z: fz = -lbm_gravity; break;
            }
        }

        return LBM(results.Nx, results.Ny, results.Nz, lbm_nu, fx, fy, fz,
                   particle_count, particle_density);
    }

    // ========================================================================
    // Reynolds Number Based Simulation (Dimensionless)
    // ========================================================================

    /**
     * @brief Configure for Reynolds number based simulation (dimensionless)
     *
     * Use this for simulations where physical SI units are not needed and
     * you want to work directly with Reynolds number and LBM units.
     *
     * @param reynolds Reynolds number
     * @param lbm_u Reference velocity in LBM units (default: 0.1)
     * @return Reference for method chaining
     *
     * @note Must be called after setup()
     * @note This is an alternative to configure_units() for dimensionless simulations
     *
     * @par Example:
     * @code
     * sim.setup();
     * sim.configure_reynolds(100000.0f, 0.075f);  // Re = 100,000
     * LBM lbm = sim.create_lbm_reynolds();
     * @endcode
     */
    SimulationSetup& configure_reynolds(float32_t reynolds, float32_t lbm_u = 0.1f) {
        reynolds_number_ = reynolds;
        lbm_u_ref_ = lbm_u;
        reynolds_configured_ = true;
        return *this;
    }

    /**
     * @brief Create LBM for Reynolds number based simulation
     *
     * Creates an LBM object with viscosity calculated from the configured
     * Reynolds number: nu = u * L / Re
     *
     * @return LBM simulation object
     *
     * @note Must be called after configure_reynolds()
     *
     * @par Example:
     * @code
     * sim.configure_reynolds(100000.0f, 0.075f);
     * LBM lbm = sim.create_lbm_reynolds();
     * @endcode
     */
    LBM create_lbm_reynolds() {
        const float32_t lbm_nu = units.nu_from_Re(reynolds_number_,
            (float32_t)results.Nx, lbm_u_ref_);
        return LBM(results.Nx, results.Ny, results.Nz, lbm_nu);
    }

    /**
     * @brief Check if Reynolds-based simulation is configured
     * @return True if configure_reynolds() has been called
     */
    bool is_reynolds_configured() const { return reynolds_configured_; }

    /**
     * @brief Get the configured Reynolds number
     * @return Reynolds number (0 if not configured)
     */
    float32_t get_reynolds_number() const { return reynolds_number_; }

    // ========================================================================
    // Mesh Loading Helper
    // ========================================================================

    /**
     * @brief Get mesh parameters for loading meshes with matching configuration
     *
     * Returns parameters that can be used with MeshLoader functions to load
     * meshes that match this simulation's scale and positioning.
     *
     * @return MeshLoader::MeshParams with current configuration
     */
    MeshLoader::MeshParams get_mesh_params() const {
        MeshLoader::MeshParams params;
        params.stl_path = original_stl_path_;
        params.center_lbm = results.center_lbm;
        params.rotation_matrix = results.rotation_matrix;
        params.lbm_reference_size = results.lbm_reference_size;
        return params;
    }

    /**
     * @brief Load and transform mesh matching SimulationSetup configuration
     *
     * Returns a Mesh* with scaling, rotation, and positioning applied
     * consistent with the simulation setup. Useful for tumbling objects
     * or parts that need Mesh* for direct rotation operations.
     *
     * @return Pointer to loaded and transformed Mesh (caller owns memory)
     *
     * @note Must be called after setup()
     * @note Caller is responsible for deleting the returned Mesh*
     *
     * @par Example (tumbling object):
     * @code
     * sim.setup();
     * sim.configure_reynolds(100000.0f, 0.075f);
     * LBM lbm = sim.create_lbm_reynolds();
     * Mesh* mesh = sim.load_mesh();
     *
     * // Tumbling rotation
     * const float3x3 spin = float3x3(float3(0.2f, 1.0f, 0.1f), radians(0.4032f));
     * while(lbm.get_t() < lbm_T) {
     *     lbm.unvoxelize_mesh_on_device(mesh);
     *     mesh->rotate(spin);
     *     lbm.voxelize_mesh_on_device(mesh);
     *     lbm.run(update_interval, lbm_T);
     * }
     * delete mesh;
     * @endcode
     */
    Mesh* load_mesh() {
        return MeshLoader::load(get_mesh_params());
    }

    /**
     * @brief Load mesh and mirror across specified plane for symmetric geometry
     *
     * Creates a full mesh from a half-model STL by mirroring across the
     * configured mirror plane. Used for symmetric aircraft and other geometry.
     *
     * @return Pointer to mirrored Mesh (caller owns memory)
     *
     * @note Must be called after setup()
     * @note Caller is responsible for deleting the returned Mesh*
     * @note Returns regular load_mesh() result if no mirroring configured
     *
     * @par Example (symmetric aircraft):
     * @code
     * SimulationSetup sim(SimulationConfig("aircraft_half.stl")
     *     .set_domain_aspect_ratio(1.0f, 1.5f, 0.33f)
     *     .set_vram_mb(2000u)
     *     .set_mirror_plane(SimulationConfig::MirrorPlane::X)
     *     .set_angle_of_attack_deg(-10.0f));
     *
     * sim.setup();
     * sim.configure_units_with_length(62.0f, 80.0f, Fluid::AIR);
     * LBM lbm = sim.create_lbm(Fluid::AIR);
     * sim.voxelize(lbm);  // Uses mirrored mesh automatically
     * @endcode
     */
    Mesh* load_mesh_mirrored() {
        return MeshLoader::load_mirrored(get_mesh_params(), config.mirror_plane_);
    }

    // ========================================================================
    // Reynolds Number Helpers (for SI-based simulations)
    // ========================================================================

    /**
     * @brief Calculate Reynolds number for the simulation
     *
     * Uses the reference size from setup() and velocity from configure_units().
     *
     * @param si_kinematic_viscosity Kinematic viscosity in m²/s
     * @return Reynolds number (dimensionless)
     *
     * @note Must be called after setup() and configure_units()
     */
    float32_t reynolds_number(float32_t si_kinematic_viscosity) const {
        return units.si_Re(results.si_reference_size, units.si_u(lbm_u_ref_), si_kinematic_viscosity);
    }

    /**
     * @brief Calculate Reynolds number using predefined fluid properties
     *
     * @param fluid Fluid properties (e.g., Fluid::AIR, Fluid::WATER)
     * @return Reynolds number (dimensionless)
     */
    float32_t reynolds_number(const FluidProperties& fluid) const {
        return reynolds_number(fluid.kinematic_viscosity);
    }

    /**
     * @brief Print Reynolds number to console
     *
     * @param si_kinematic_viscosity Kinematic viscosity in m²/s
     */
    void print_reynolds_number(float32_t si_kinematic_viscosity) const {
        print_info("Re = " + to_string(static_cast<uint32_t>(reynolds_number(si_kinematic_viscosity))));
    }

    /**
     * @brief Print Reynolds number using predefined fluid properties
     *
     * @param fluid Fluid properties (e.g., Fluid::AIR, Fluid::WATER)
     */
    void print_reynolds_number(const FluidProperties& fluid) const {
        print_reynolds_number(fluid.kinematic_viscosity);
    }

    // ========================================================================
    // Domain and Position Helpers
    // ========================================================================

    /**
     * @brief Get domain dimensions in SI units (meters)
     * @return Domain dimensions (Nx, Ny, Nz) converted to meters
     *
     * @note Must be called after setup() and configure_units()
     */
    float3 get_domain_size_si() const {
        const float32_t scale = results.si_reference_size / results.lbm_reference_size;
        return float3(
            (float32_t)results.Nx * scale,
            (float32_t)results.Ny * scale,
            (float32_t)results.Nz * scale
        );
    }

    /**
     * @brief Convert SI position (meters) to LBM position (cells)
     * @param position_m Position in meters
     * @return Position in LBM units (cells)
     *
     * @note Must be called after setup() and configure_units()
     *
     * @par Example:
     * @code
     * float3 probe_si(10.0f, 20.0f, 5.0f);  // 10m, 20m, 5m
     * float3 probe_lbm = sim.position_to_lbm(probe_si);
     * @endcode
     */
    float3 position_to_lbm(float3 position_m) const {
        const float32_t scale = results.lbm_reference_size / results.si_reference_size;
        return float3(
            position_m.x * scale,
            position_m.y * scale,
            position_m.z * scale
        );
    }

    /**
     * @brief Convert LBM position (cells) to SI position (meters)
     * @param position_lbm Position in LBM units (cells)
     * @return Position in meters
     *
     * @note Must be called after setup() and configure_units()
     *
     * @par Example:
     * @code
     * float3 center_lbm = sim.get_results().center_lbm;
     * float3 center_si = sim.position_to_si(center_lbm);
     * @endcode
     */
    float3 position_to_si(float3 position_lbm) const {
        const float32_t scale = results.si_reference_size / results.lbm_reference_size;
        return float3(
            position_lbm.x * scale,
            position_lbm.y * scale,
            position_lbm.z * scale
        );
    }

    /**
     * @brief Load an STL mesh and fit it to the domain with specified occupancy
     *
     * Loads an STL file and scales it to occupy a specified fraction of the
     * domain's reference dimension. Useful for loading additional geometry
     * that should fit within the simulation domain.
     *
     * @param stl_path Path to STL file (relative to resources/ or absolute)
     * @param occupancy Fraction of domain reference dimension to occupy (0.0-1.0, default: 0.8)
     * @return Pointer to loaded and scaled Mesh (caller owns memory)
     *
     * @note Must be called after setup()
     * @note Caller is responsible for deleting the returned Mesh*
     *
     * @par Example:
     * @code
     * // Load obstacle at 50% of domain size, centered
     * Mesh* obstacle = sim.load_mesh_fitted("obstacle.stl", 0.5f);
     * lbm.voxelize_mesh_on_device(obstacle, TYPE_S);
     * delete obstacle;
     * @endcode
     */
    Mesh* load_mesh_fitted(const std::string& stl_path, float32_t occupancy = 0.8f) {
        const std::string resolved_path = get_resource_path(stl_path);
        if(resolved_path.empty()) {
            print_error("SimulationSetup::load_mesh_fitted: STL file not found: " + stl_path);
            return nullptr;
        }

        // Load mesh at unit scale
        Mesh* mesh = read_stl(resolved_path, 1.0f);
        if(mesh == nullptr) {
            print_error("SimulationSetup::load_mesh_fitted: Failed to load STL: " + stl_path);
            return nullptr;
        }

        // Calculate target size based on domain reference dimension and occupancy
        const float32_t target_size = occupancy * results.lbm_reference_size;

        // Scale mesh to target size
        mesh->scale(target_size / mesh->get_max_size());

        // Center mesh in domain
        const float3 domain_center(
            0.5f * (float32_t)results.Nx,
            0.5f * (float32_t)results.Ny,
            0.5f * (float32_t)results.Nz
        );
        mesh->translate(domain_center - mesh->get_bounding_box_center());
        mesh->set_center(mesh->get_center_of_mass());

        return mesh;
    }

    /**
     * @brief Load multiple STL meshes with consistent scaling
     *
     * Loads multiple STL files and scales them all using the same scale factor
     * (based on the first mesh). Useful for multi-part assemblies where parts
     * should maintain their relative sizes.
     *
     * @param stl_paths Vector of STL file paths
     * @param occupancy Fraction of domain for the first mesh (default: 0.8)
     * @return Vector of pointers to loaded Meshes (caller owns memory)
     *
     * @note Must be called after setup()
     * @note Caller is responsible for deleting all returned Mesh* pointers
     *
     * @par Example:
     * @code
     * std::vector<std::string> parts = {"body.stl", "wing_left.stl", "wing_right.stl"};
     * auto meshes = sim.load_mesh_group(parts, 0.7f);
     * for(Mesh* m : meshes) {
     *     lbm.voxelize_mesh_on_device(m, TYPE_S);
     *     delete m;
     * }
     * @endcode
     */
    std::vector<Mesh*> load_mesh_group(const std::vector<std::string>& stl_paths, float32_t occupancy = 0.8f) {
        std::vector<Mesh*> meshes;
        if(stl_paths.empty()) return meshes;

        // Load first mesh to determine scale factor
        Mesh* first = load_mesh_fitted(stl_paths[0], occupancy);
        if(first == nullptr) return meshes;

        const float32_t scale_factor = occupancy * results.lbm_reference_size / first->get_max_size();
        meshes.push_back(first);

        // Load remaining meshes with same scale
        for(size_t i = 1; i < stl_paths.size(); i++) {
            const std::string resolved_path = get_resource_path(stl_paths[i]);
            if(resolved_path.empty()) {
                print_error("SimulationSetup::load_mesh_group: STL file not found: " + stl_paths[i]);
                continue;
            }

            Mesh* mesh = read_stl(resolved_path, 1.0f);
            if(mesh == nullptr) {
                print_error("SimulationSetup::load_mesh_group: Failed to load STL: " + stl_paths[i]);
                continue;
            }

            // Apply same scale factor as first mesh
            mesh->scale(scale_factor);

            // Center in domain
            const float3 domain_center(
                0.5f * (float32_t)results.Nx,
                0.5f * (float32_t)results.Ny,
                0.5f * (float32_t)results.Nz
            );
            mesh->translate(domain_center - mesh->get_bounding_box_center());
            mesh->set_center(mesh->get_center_of_mass());

            meshes.push_back(mesh);
        }

        return meshes;
    }

    // ========================================================================
    // Simulation Execution Helpers
    // ========================================================================

    /**
     * @brief Run simulation for specified time
     *
     * @param lbm LBM simulation object
     * @param simulation_time_s Total simulation time in seconds
     */
    void run(LBM& lbm, float32_t simulation_time_s) {
        lbm.run(to_lbm_timesteps(simulation_time_s));
    }
};
