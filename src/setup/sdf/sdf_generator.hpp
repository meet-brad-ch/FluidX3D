#pragma once

#include "core/types.hpp"
#include <string>

/**
 * @file sdf_generator.hpp
 * @brief Standalone SDF generation for FluidX3D simulations
 *
 * Provides decoupled SDF generation with caching support.
 * Use this class when you need explicit control over SDF generation timing
 * or want to generate SDFs at custom resolutions.
 */

/**
 * @class SDFGenerator
 * @brief Generates and caches Signed Distance Fields from STL files
 *
 * This class provides explicit control over SDF generation, decoupled from
 * SimulationSetup. It wraps SDFCacheManager with a simpler fluent interface.
 *
 * @par Basic Usage:
 * @code
 * SDFGenerator sdf;
 * std::string sdf_path = sdf.generate("mesh.stl", 256, 256, 128);
 * @endcode
 *
 * @par With Configuration:
 * @code
 * SDFGenerator sdf;
 * sdf.set_cache_dir("resources/sdf_cache/")
 *    .enable_cache(true)
 *    .set_verbose(true);
 *
 * if (!sdf.has_cached("mesh.stl", 256, 256, 128)) {
 *     std::string path = sdf.generate("mesh.stl", 256, 256, 128);
 * }
 * @endcode
 */
class SDFGenerator {
public:
    /**
     * @brief Construct SDFGenerator with default settings
     *
     * Default settings:
     * - Cache directory: "resources/sdf_cache/"
     * - Caching enabled: true
     * - Verbose output: false
     */
    SDFGenerator();

    /**
     * @brief Set the cache directory
     * @param dir Path to cache directory (will be created if it doesn't exist)
     * @return Reference to this object for method chaining
     */
    SDFGenerator& set_cache_dir(const std::string& dir);

    /**
     * @brief Enable or disable caching
     * @param enable True to enable caching, false to always regenerate
     * @return Reference to this object for method chaining
     */
    SDFGenerator& enable_cache(bool enable);

    /**
     * @brief Enable or disable verbose output
     * @param verbose True to print progress messages
     * @return Reference to this object for method chaining
     */
    SDFGenerator& set_verbose(bool verbose);

    /**
     * @brief Enable or disable mesh repair for non-watertight meshes
     * @param fix True to attempt hole-filling on non-watertight meshes
     * @return Reference to this object for method chaining
     */
    SDFGenerator& set_fix_mesh(bool fix);

    /**
     * @brief Generate SDF from STL file at specified dimensions
     *
     * If caching is enabled and a matching SDF exists, returns the cached path.
     * Otherwise, generates a new SDF using GPU-accelerated algorithm.
     *
     * @param stl_path Path to STL file (absolute or relative to resources/)
     * @param nx Target X dimension in voxels
     * @param ny Target Y dimension in voxels
     * @param nz Target Z dimension in voxels
     * @param padding Padding cells around geometry (default: 1)
     * @return Path to generated SDF file, or empty string on failure
     */
    std::string generate(const std::string& stl_path,
                        uint32_t nx, uint32_t ny, uint32_t nz,
                        int32_t padding = 1);

    /**
     * @brief Check if a cached SDF exists for the given parameters
     *
     * Uses hash-based lookup considering STL content and dimensions.
     *
     * @param stl_path Path to STL file
     * @param nx Target X dimension
     * @param ny Target Y dimension
     * @param nz Target Z dimension
     * @return True if a matching cached SDF exists
     */
    bool has_cached(const std::string& stl_path,
                   uint32_t nx, uint32_t ny, uint32_t nz);

    /**
     * @brief Get path to cached SDF without generating
     *
     * @param stl_path Path to STL file
     * @param nx Target X dimension
     * @param ny Target Y dimension
     * @param nz Target Z dimension
     * @return Path to cached SDF, or empty string if not cached
     */
    std::string get_cached_path(const std::string& stl_path,
                               uint32_t nx, uint32_t ny, uint32_t nz);

    /**
     * @brief Clear all cached SDFs
     * @return Number of files deleted
     */
    int32_t clear_cache();

    /**
     * @brief Clear cached SDFs for a specific STL file
     * @param stl_basename Base name of STL file (e.g., "mesh" from "mesh.stl")
     * @return Number of files deleted
     */
    int32_t clear_cache(const std::string& stl_basename);

private:
    std::string cache_dir_;
    bool cache_enabled_;
    bool verbose_;
    bool fix_mesh_;
};
