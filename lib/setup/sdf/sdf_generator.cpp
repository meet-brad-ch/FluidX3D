#include "setup/sdf/sdf_generator.hpp"
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
    std::string resolved_stl = stl_path;
    if (stl_path.find('/') == std::string::npos &&
        stl_path.find('\\') == std::string::npos) { // bare file name: look it up in resources/
        resolved_stl = get_resource_path(stl_path);
        if (resolved_stl.empty()) {
            if (verbose_) {
                print_error("[SDFGenerator] STL file not found: " + stl_path);
            }
            return "";
        }
    }

    SDFCacheConfig cache_config;
    cache_config.cache_directory = cache_dir_;
    cache_config.enable_cache = cache_enabled_;
    cache_config.force_regenerate = !cache_enabled_;
    cache_config.fix_mesh = fix_mesh_;
    cache_config.verbose = verbose_;

    SDFCacheManager cache(cache_config);
    return cache.get_or_generate(resolved_stl, nx, ny, nz, padding);
}
