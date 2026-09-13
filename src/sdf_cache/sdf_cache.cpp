/**
 * @file sdf_cache.cpp
 * @brief The SDF cache: lookup by file name, generation with SDFGen
 */

#include "sdf_cache.hpp"
#include "hash_utils.hpp"
#include "sdfgen_unified.h"
#include "sdf_io.h"
#include "mesh_repair.h"
#include "vec.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

namespace fs = std::filesystem;

/**
 * @brief A binary STL file's triangles as SDFGen's vertex and face lists
 *
 * The binary STL format is an 80-byte header, a 4-byte triangle count, then 50 bytes per triangle: the normal
 * (12 bytes), three vertices (36 bytes) and two attribute bytes.
 */
class BinaryStl {
public:
    /**
     * @brief Read a binary STL file
     *
     * Every triangle gets its own three vertices (STL files repeat shared vertices); weld them afterwards.
     *
     * @param path Path to the STL file
     * @param vertices Output: three vertices per triangle
     * @param faces Output: one face per triangle, indexing its vertices
     * @return true if the file is a binary STL and was read; false, with a message on stderr, if it cannot be opened
     *         or is not a binary STL (an ASCII STL must be converted to binary first)
     */
    static bool load(const std::string& path, std::vector<Vec3f>& vertices, std::vector<Vec3ui>& faces) {
        std::ifstream file(path, std::ios::binary);
        if(!file) {
            std::cerr << "[SDF Cache] Cannot open the STL file " << path << std::endl;
            return false;
        }
        file.seekg(0, std::ios::end);
        const std::streamoff file_size = file.tellg();
        file.seekg(80, std::ios::beg); // past the header
        uint32_t triangle_count = 0u;
        file.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
        const std::streamoff expected_size = 84 + static_cast<std::streamoff>(triangle_count) * 50; // the size a binary STL of this count has
        if(file_size < 84 || file_size != expected_size || triangle_count == 0u) {
            std::cerr << "[SDF Cache] " << path << " is not a binary STL (an ASCII STL must be converted to binary)" << std::endl;
            return false;
        }

        vertices.clear();
        faces.clear();
        vertices.reserve(static_cast<size_t>(triangle_count) * 3u);
        faces.reserve(triangle_count);
        for(uint32_t i = 0u; i < triangle_count; i++) {
            file.seekg(12, std::ios::cur); // the normal
            float v[9];
            file.read(reinterpret_cast<char*>(v), sizeof(v));
            file.seekg(2, std::ios::cur); // the attribute bytes
            for(int j = 0; j < 3; j++) vertices.push_back(Vec3f(v[j * 3], v[j * 3 + 1], v[j * 3 + 2]));
            faces.push_back(Vec3ui(i * 3u, i * 3u + 1u, i * 3u + 2u));
        }
        return true;
    }

    /**
     * @brief The bounding box of a vertex list
     *
     * @param vertices The vertices
     * @param min_box Output: the box's minimum corner
     * @param max_box Output: the box's maximum corner
     */
    static void bounds(const std::vector<Vec3f>& vertices, Vec3f& min_box, Vec3f& max_box) {
        min_box = Vec3f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        max_box = -min_box;
        for(const Vec3f& v : vertices) update_minmax(v, min_box, max_box);
    }
};

SDFCacheManager::SDFCacheManager(const SDFCacheConfig& config) : config_(config) {
    fs::create_directories(config_.cache_directory);
}

std::string SDFCacheManager::get_or_generate(const std::string& stl_path, uint32_t nx, uint32_t ny, uint32_t nz, int32_t padding) {
    if(nx == 0u || ny == 0u || nz == 0u) {
        std::cerr << "[SDF Cache] The SDF grid must have at least one cell along each axis" << std::endl;
        return "";
    }
    if(padding < 1) padding = 1;

    const uint64_t key = SDFCacheKey::compute(stl_path, nx, ny, nz, padding, config_.fix_mesh);
    const std::string stl_basename = fs::path(stl_path).stem().string();

    const std::string cached = find_cached(stl_basename, key, nx, ny, nz, padding);
    if(!cached.empty()) {
        if(config_.verbose) std::cout << "[SDF Cache] Cache hit: " << cached << std::endl;
        return cached;
    }
    if(config_.verbose) std::cout << "[SDF Cache] Cache miss: generating" << std::endl;
    return generate(stl_path, key, nx, ny, nz, padding);
}

std::string SDFCacheManager::find_cached(const std::string& stl_basename, uint64_t key, uint32_t nx, uint32_t ny, uint32_t nz, int32_t padding) const {
    // <basename>_sdf_<nx>x<ny>x<nz>_<key>.sdf, with the grid as generated: the padding on each side
    const uint32_t pad = 2u * static_cast<uint32_t>(padding);
    const std::string name = stl_basename + "_sdf_" + std::to_string(nx + pad) + "x" + std::to_string(ny + pad) + "x" + std::to_string(nz + pad) +
                             "_" + SDFCacheKey::format(key) + ".sdf";
    const fs::path path = fs::path(config_.cache_directory) / name;
    std::error_code error; // a missing directory is a miss, not an exception
    return fs::is_regular_file(path, error) ? path.string() : "";
}

std::string SDFCacheManager::generate(const std::string& stl_path, uint64_t key, uint32_t nx, uint32_t ny, uint32_t nz, int32_t padding) const {
    std::vector<Vec3f> vertices;
    std::vector<Vec3ui> faces;
    if(!BinaryStl::load(stl_path, vertices, faces)) return "";

    // STL files repeat the vertices of every triangle: weld them, then look at whether the mesh is closed
    const int welded = meshio::weld_vertices(vertices, faces, 1e-5f);
    if(config_.verbose && welded > 0) std::cout << "[SDF Cache] Welded " << welded << " duplicate vertices" << std::endl;
    const meshio::MeshAnalysis analysis = meshio::analyze_mesh(vertices, faces);
    if(config_.verbose) meshio::print_mesh_analysis(analysis, false);
    if(!analysis.is_watertight) {
        if(config_.fix_mesh) {
            const int holes_filled = meshio::repair_mesh(vertices, faces, 0.0f);
            if(config_.verbose) std::cout << "[SDF Cache] Mesh repair filled " << holes_filled << " holes" << std::endl;
        } else if(config_.verbose) {
            std::cout << "[SDF Cache] The mesh is not watertight, so the SDF's sign may be wrong in places; fix_mesh repairs it" << std::endl;
        }
    }

    // the grid: the model's extent along X is nx cells; padding cells around it on every side; the model centered
    Vec3f min_box, max_box;
    BinaryStl::bounds(vertices, min_box, max_box);
    const float dx = (max_box - min_box)[0] / static_cast<float>(nx);
    const int grid_nx = static_cast<int>(nx) + 2 * padding;
    const int grid_ny = static_cast<int>(ny) + 2 * padding;
    const int grid_nz = static_cast<int>(nz) + 2 * padding;
    const Vec3f grid_size(static_cast<float>(grid_nx) * dx, static_cast<float>(grid_ny) * dx, static_cast<float>(grid_nz) * dx);
    const Vec3f center = (min_box + max_box) * 0.5f;
    const Vec3f origin = center - grid_size * 0.5f;
    if(config_.verbose) std::cout << "[SDF Cache] Generating the SDF: " << faces.size() << " triangles, cell size " << dx << std::endl;

    Array3f phi; // SDFGen on the GPU, or on the CPU if the GPU fails
    sdfgen::make_level_set3(faces, vertices, origin, dx, grid_nx, grid_ny, grid_nz, phi, 1, sdfgen::HardwareBackend::Auto);

    std::ostringstream name;
    name << fs::path(stl_path).stem().string() << "_sdf_" << phi.ni << "x" << phi.nj << "x" << phi.nk << "_" << SDFCacheKey::format(key) << ".sdf";
    const std::string output_path = (fs::path(config_.cache_directory) / name.str()).string();

    int inside_count = 0;
    if(!write_sdf_binary(output_path, phi, origin, dx, &inside_count)) {
        std::cerr << "[SDF Cache] Cannot write " << output_path << std::endl;
        return "";
    }
    if(config_.verbose) {
        const long long cells = static_cast<long long>(phi.ni) * phi.nj * phi.nk;
        const float size_mb = static_cast<float>(36 + cells * static_cast<long long>(sizeof(float))) / (1024.0f * 1024.0f);
        std::cout << "[SDF Cache] Saved " << output_path << " (" << size_mb << " MB, " << (100.0f * static_cast<float>(inside_count) / static_cast<float>(cells)) << " % solid)" << std::endl;
    }
    return output_path;
}
