#include "sdf/sdf_generator.hpp"
#include "sdf_cache/sdf_cache.hpp"
#include "utilities.hpp"

SDFGenerator::SDFGenerator()
    : cache_dir_("resources/sdf_cache/")
    , cache_enabled_(true)
    , verbose_(false)
    , fix_mesh_(false) {
}

SDFGenerator& SDFGenerator::set_cache_dir(const std::string& dir) {
    cache_dir_ = dir;
    // Ensure trailing slash
    if (!cache_dir_.empty() && cache_dir_.back() != '/' && cache_dir_.back() != '\\') {
        cache_dir_ += '/';
    }
    return *this;
}

SDFGenerator& SDFGenerator::enable_cache(bool enable) {
    cache_enabled_ = enable;
    return *this;
}

SDFGenerator& SDFGenerator::set_verbose(bool verbose) {
    verbose_ = verbose;
    return *this;
}

SDFGenerator& SDFGenerator::set_fix_mesh(bool fix) {
    fix_mesh_ = fix;
    return *this;
}

std::string SDFGenerator::generate(const std::string& stl_path,
                                   uint32_t nx, uint32_t ny, uint32_t nz,
                                   int32_t padding) {
    // Resolve STL path
    std::string resolved_stl = stl_path;
    if (stl_path.find('/') == std::string::npos &&
        stl_path.find('\\') == std::string::npos) {
        // Relative path - try resource lookup
        resolved_stl = get_resource_path(stl_path);
        if (resolved_stl.empty()) {
            if (verbose_) {
                print_error("[SDFGenerator] STL file not found: " + stl_path);
            }
            return "";
        }
    }

    // Configure cache manager
    SDFCacheConfig cache_config;
    cache_config.cache_directory = cache_dir_;
    cache_config.enable_cache = cache_enabled_;
    cache_config.force_regenerate = !cache_enabled_;
    cache_config.fix_mesh = fix_mesh_;
    cache_config.verbose = verbose_;

    SDFCacheManager cache(cache_config);

    // Generate or retrieve cached SDF
    std::string sdf_path = cache.get_or_generate(
        resolved_stl,
        nx, ny, nz,
        padding
    );

    return sdf_path;
}

bool SDFGenerator::has_cached(const std::string& stl_path,
                             uint32_t nx, uint32_t ny, uint32_t nz) {
    return !get_cached_path(stl_path, nx, ny, nz).empty();
}

std::string SDFGenerator::get_cached_path(const std::string& stl_path,
                                          uint32_t nx, uint32_t ny, uint32_t nz) {
    // Resolve STL path
    std::string resolved_stl = stl_path;
    if (stl_path.find('/') == std::string::npos &&
        stl_path.find('\\') == std::string::npos) {
        resolved_stl = get_resource_path(stl_path);
        if (resolved_stl.empty()) {
            return "";
        }
    }

    // Configure cache manager for lookup only
    SDFCacheConfig cache_config;
    cache_config.cache_directory = cache_dir_;
    cache_config.enable_cache = true;
    cache_config.verbose = false;

    SDFCacheManager cache(cache_config);
    return cache.find_cached(resolved_stl, nx, ny, nz, 1);
}

int32_t SDFGenerator::clear_cache() {
    SDFCacheConfig cache_config;
    cache_config.cache_directory = cache_dir_;
    cache_config.verbose = verbose_;

    SDFCacheManager cache(cache_config);
    return cache.clear_all_cache();
}

int32_t SDFGenerator::clear_cache(const std::string& stl_basename) {
    SDFCacheConfig cache_config;
    cache_config.cache_directory = cache_dir_;
    cache_config.verbose = verbose_;

    SDFCacheManager cache(cache_config);
    return cache.clear_cache(stl_basename);
}
