#include "sdf_cache.hpp"
#include "hash_utils.hpp"
#include "sdfgen_unified.h"
#include "sdf_io.h"
#include "mesh_repair.h"
#include "vec.h"
#include <iostream>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <limits>
#include <cmath>

namespace fs = std::filesystem;

// Note: SDF Cache is now dimension-driven instead of VRAM-driven
// The caller (GeometrySetup) calculates target dimensions and passes them directly

// Load binary STL file (simple format: 80-byte header + triangles)
static bool load_binary_stl(const char* filename, std::vector<Vec3f>& vertList, std::vector<Vec3ui>& faceList,
                            Vec3f& min_box, Vec3f& max_box) {
    std::ifstream file(filename, std::ios::binary);
    if(!file) {
        std::cerr << "[SDF Cache] Failed to open STL file: " << filename << std::endl;
        return false;
    }

    // Skip 80-byte header
    file.seekg(80, std::ios::beg);

    // Read triangle count
    uint32_t num_triangles;
    file.read(reinterpret_cast<char*>(&num_triangles), sizeof(uint32_t));

    // Initialize bounding box
    min_box = Vec3f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    max_box = Vec3f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());

    // Reserve space for vertices
    std::vector<Vec3f> temp_verts;
    temp_verts.reserve(num_triangles * 3);

    for(uint32_t i = 0; i < num_triangles; i++) {
        // Skip normal (12 bytes)
        file.seekg(12, std::ios::cur);

        // Read 3 vertices (9 floats = 36 bytes)
        float v[9];
        file.read(reinterpret_cast<char*>(v), 36);

        // Skip attribute bytes (2 bytes)
        file.seekg(2, std::ios::cur);

        // Store vertices and update bounds
        Vec3f vertices[3];
        for(int j = 0; j < 3; j++) {
            vertices[j] = Vec3f(v[j*3], v[j*3+1], v[j*3+2]);
            update_minmax(vertices[j], min_box, max_box);
            temp_verts.push_back(vertices[j]);
        }

        // Add face (indices for this triangle)
        unsigned int idx0 = i * 3;
        unsigned int idx1 = i * 3 + 1;
        unsigned int idx2 = i * 3 + 2;
        faceList.push_back(Vec3ui(idx0, idx1, idx2));
    }

    file.close();

    // Copy all vertices
    vertList = temp_verts;

    return true;
}

SDFCacheManager::SDFCacheManager(const SDFCacheConfig& config)
    : config_(config) {
    // Create cache directory if it doesn't exist
    if (config_.enable_cache) {
        fs::create_directories(config_.cache_directory);
    }
}

std::string SDFCacheManager::get_or_generate(const std::string& stl_path, uint32_t target_nx, uint32_t target_ny, uint32_t target_nz, int32_t padding) {
    // If caching disabled or force regenerate, generate directly
    if (!config_.enable_cache || config_.force_regenerate) {
        if (config_.verbose) {
            std::cout << "[SDF Cache] Cache disabled or force regenerate" << std::endl;
        }
        return generate_sdf(stl_path, target_nx, target_ny, target_nz, padding);
    }

    // Compute cache key
    uint64_t cache_key = compute_sdf_cache_key(stl_path, target_nx, target_ny, target_nz, padding, config_.fix_mesh);

    // Get STL basename
    fs::path stl_file(stl_path);
    std::string stl_basename = stl_file.stem().string();

    // Check for cached SDF
    std::string cached_path = find_cached_sdf(stl_basename, cache_key, target_nx, target_ny, target_nz);

    if (!cached_path.empty()) {
        if (config_.verbose) {
            std::cout << "[SDF Cache] Cache HIT: " << cached_path << std::endl;
        }
        return cached_path;
    }

    // Cache miss - generate new SDF
    if (config_.verbose) {
        std::cout << "[SDF Cache] Cache MISS - generating..." << std::endl;
    }

    return generate_sdf(stl_path, target_nx, target_ny, target_nz, padding);
}

int SDFCacheManager::clear_cache(const std::string& stl_basename) {
    int deleted_count = 0;

    try {
        for (const auto& entry : fs::directory_iterator(config_.cache_directory)) {
            if (entry.is_regular_file()) {
                std::string filename = entry.path().filename().string();
                // Check if filename starts with basename
                if (filename.find(stl_basename) == 0 && filename.substr(filename.size() - 4) == ".sdf") {
                    fs::remove(entry.path());
                    deleted_count++;
                    if (config_.verbose) {
                        std::cout << "[SDF Cache] Deleted: " << filename << std::endl;
                    }
                }
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "[SDF Cache] ERROR clearing cache: " << e.what() << std::endl;
    }

    return deleted_count;
}

int SDFCacheManager::clear_all_cache() {
    int deleted_count = 0;

    try {
        for (const auto& entry : fs::directory_iterator(config_.cache_directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".sdf") {
                fs::remove(entry.path());
                deleted_count++;
                if (config_.verbose) {
                    std::cout << "[SDF Cache] Deleted: " << entry.path().filename().string() << std::endl;
                }
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "[SDF Cache] ERROR clearing cache: " << e.what() << std::endl;
    }

    return deleted_count;
}

void SDFCacheManager::get_cache_stats(int& total_files, float& total_size_mb) {
    total_files = 0;
    total_size_mb = 0.0f;

    try {
        for (const auto& entry : fs::directory_iterator(config_.cache_directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".sdf") {
                total_files++;
                total_size_mb += static_cast<float>(fs::file_size(entry.path())) / (1024.0f * 1024.0f);
            }
        }
    } catch (const fs::filesystem_error&) {
        // Directory doesn't exist or can't be read
    }
}

void SDFCacheManager::set_verbose(bool verbose) {
    config_.verbose = verbose;
}

std::string SDFCacheManager::find_cached(const std::string& stl_path, uint32_t target_nx, uint32_t target_ny, uint32_t target_nz, int32_t padding) {
    if (!config_.enable_cache) {
        return "";
    }

    // Compute cache key
    uint64_t cache_key = compute_sdf_cache_key(stl_path, target_nx, target_ny, target_nz, padding, config_.fix_mesh);

    // Get STL basename
    fs::path stl_file(stl_path);
    std::string stl_basename = stl_file.stem().string();

    // Look for cached SDF
    return find_cached_sdf(stl_basename, cache_key, target_nx, target_ny, target_nz);
}

std::string SDFCacheManager::find_cached_sdf(const std::string& stl_basename, uint64_t expected_hash, uint32_t nx, uint32_t ny, uint32_t nz) {
    // Look for files matching: {basename}_sdf_{nx}x{ny}x{nz}_{hash8}.sdf
    std::string hash_str = format_hash(expected_hash);
    std::string dim_pattern = "_sdf_" + std::to_string(nx + 2) + "x" + std::to_string(ny + 2) + "x" + std::to_string(nz + 2);
    std::string pattern_suffix = "_" + hash_str + ".sdf";

    try {
        for (const auto& entry : fs::directory_iterator(config_.cache_directory)) {
            if (entry.is_regular_file()) {
                std::string filename = entry.path().filename().string();

                // Check if filename:
                // 1. Starts with basename
                // 2. Contains the dimension pattern (e.g., "_sdf_479x479x85")
                // 3. Ends with hash pattern
                if (filename.find(stl_basename) == 0 &&
                    filename.find(dim_pattern) != std::string::npos &&
                    filename.size() > pattern_suffix.size() &&
                    filename.substr(filename.size() - pattern_suffix.size()) == pattern_suffix) {
                    return entry.path().string();
                }
            }
        }
    } catch (const fs::filesystem_error&) {
        // Directory doesn't exist or can't be read
        return "";
    }

    return "";
}

std::string SDFCacheManager::generate_sdf(const std::string& stl_path, uint32_t target_nx, uint32_t target_ny, uint32_t target_nz, int32_t padding) {
    // Validate inputs
    if (target_nx == 0 || target_ny == 0 || target_nz == 0) {
        std::cerr << "[SDF Cache] ERROR: Target dimensions must be positive" << std::endl;
        return "";
    }
    if (padding < 1) {
        padding = 1;
    }

    // Load STL file to get mesh dimensions
    std::vector<Vec3f> vertList;
    std::vector<Vec3ui> faceList;
    Vec3f min_box, max_box;

    if (!load_binary_stl(stl_path.c_str(), vertList, faceList, min_box, max_box)) {
        std::cerr << "[SDF Cache] ERROR: Failed to load STL file" << std::endl;
        return "";
    }

    // Weld duplicate vertices (STL files have separate vertices per triangle)
    int welded = meshio::weld_vertices(vertList, faceList, 1e-5f);
    if (config_.verbose && welded > 0) {
        std::cout << "[SDF Cache] Welded " << welded << " duplicate vertices" << std::endl;
    }

    // Analyze mesh watertightness
    meshio::MeshAnalysis analysis = meshio::analyze_mesh(vertList, faceList);
    if (config_.verbose) {
        meshio::print_mesh_analysis(analysis, false);
    }

    // Optionally repair non-watertight meshes
    if (config_.fix_mesh && !analysis.is_watertight) {
        if (config_.verbose) {
            std::cout << "[SDF Cache] Attempting mesh repair..." << std::endl;
        }
        int holes_filled = meshio::repair_mesh(vertList, faceList, 0.0f);
        if (holes_filled > 0) {
            // Recalculate bounding box after repair
            min_box = Vec3f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
            max_box = Vec3f(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
            for (const auto& v : vertList) {
                update_minmax(v, min_box, max_box);
            }
            if (config_.verbose) {
                std::cout << "[SDF Cache] Filled " << holes_filled << " holes" << std::endl;
            }
        }
    } else if (!analysis.is_watertight && config_.verbose) {
        std::cout << "[SDF Cache] WARNING: Mesh is not watertight. SDF sign may be incorrect." << std::endl;
        std::cout << "[SDF Cache] Use fix_mesh=true in config to attempt automatic repair." << std::endl;
    }

    if (config_.verbose) {
        std::cout << "[SDF Cache] Generating SDF: " << faceList.size() << " triangles, grid spacing: ";
    }

    Vec3f mesh_size = max_box - min_box;

    // SDF grid = target dimensions + padding
    int sdf_nx = target_nx + 2 * padding;
    int sdf_ny = target_ny + 2 * padding;
    int sdf_nz = target_nz + 2 * padding;

    // Calculate dx based on X dimension
    float dx = mesh_size[0] / target_nx;

    // Recalculate bounds to exactly fit the SDF grid with calculated dx
    // Center the mesh in the grid with padding on all sides
    Vec3f grid_size = Vec3f(sdf_nx * dx, sdf_ny * dx, sdf_nz * dx);
    Vec3f mesh_center = (min_box + max_box) * 0.5f;

    min_box = mesh_center - grid_size * 0.5f;
    max_box = mesh_center + grid_size * 0.5f;

    if (config_.verbose) {
        std::cout << dx << " m" << std::endl;
    }

    // Generate the SDF using SDFGen's unified interface (GPU, falls back to CPU if the GPU fails)
    Array3f phi_grid;
    sdfgen::make_level_set3(faceList, vertList, min_box, dx, sdf_nx, sdf_ny, sdf_nz, phi_grid, 1, sdfgen::HardwareBackend::Auto);

    // Generate output filename
    fs::path stl_file(stl_path);
    std::string stl_basename = stl_file.stem().string();

    std::ostringstream dims;
    dims << "_sdf_" << phi_grid.ni << "x" << phi_grid.nj << "x" << phi_grid.nk;
    std::string filename = stl_basename + dims.str();

    if (config_.enable_cache) {
        // Add hash to filename
        uint64_t cache_key = compute_sdf_cache_key(stl_path, target_nx, target_ny, target_nz, padding, config_.fix_mesh);
        std::string hash_str = format_hash(cache_key);
        filename += "_" + hash_str;
    }

    filename += ".sdf";

    // Determine output path
    std::string output_path;
    if (config_.enable_cache) {
        fs::path cached_path = fs::path(config_.cache_directory) / filename;
        output_path = cached_path.string();
    } else {
        // Output to same directory as STL
        fs::path output_dir = stl_file.parent_path();
        output_path = (output_dir / filename).string();
    }

    // DEBUG: Print first few values before writing
    if (config_.verbose) {
        std::cout << "[SDF Cache] First 5 values from phi_grid: ";
        for (int i = 0; i < 5 && i < phi_grid.ni; ++i) {
            std::cout << "[" << i << ",0,0]=" << phi_grid(i,0,0) << " ";
        }
        std::cout << std::endl;
    }

    // Use shared SDF file writing function
    int inside_count = 0;
    int total_count = phi_grid.ni * phi_grid.nj * phi_grid.nk;

    if (!write_sdf_binary(output_path, phi_grid, min_box, dx, &inside_count)) {
        std::cerr << "[SDF Cache] ERROR: Failed to write SDF file" << std::endl;
        return "";
    }

    if (config_.verbose) {
        long long file_size_bytes = 36 + (long long)total_count * sizeof(float);
        float file_size_mb = file_size_bytes / (1024.0f * 1024.0f);
        std::cout << "[SDF Cache] Saved: " << output_path
                  << " (" << file_size_mb << " MB, "
                  << (100.0f * inside_count / total_count) << "% solid)" << std::endl;
    }

    return output_path;
}
