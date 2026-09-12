#pragma once

#include "setup/core/types.hpp"
#include <string>

// Generates signed distance fields from STL files, with a hash-based cache (wraps SDFCacheManager).
class SDFGenerator {
public:
    SDFGenerator(); // cache in resources/sdf_cache/, caching on, quiet

    SDFGenerator& set_cache_dir(const std::string& dir);
    SDFGenerator& enable_cache(bool enable); // false: always regenerate
    SDFGenerator& set_verbose(bool verbose);
    SDFGenerator& set_fix_mesh(bool fix); // repair holes in non-watertight meshes

    // SDF of nx*ny*nz voxels (stl_path absolute or in resources/); returns the SDF path, or "" on failure
    std::string generate(const std::string& stl_path,
                        uint32_t nx, uint32_t ny, uint32_t nz,
                        int32_t padding = 1);

private:
    std::string cache_dir_;
    bool cache_enabled_;
    bool verbose_;
    bool fix_mesh_;
};
