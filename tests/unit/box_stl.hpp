#pragma once

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

// Test fixture: a binary STL (the only format read_stl() reads) of an axis-aligned box from the origin to (sx, sy, sz),
// removed again at the end of the test.
class BoxStl {
public:
    BoxStl(const char* temp_name, float sx, float sy, float sz) // in the system's temp directory
        : BoxStl(std::filesystem::temp_directory_path() / temp_name, sx, sy, sz) {}

    BoxStl(const std::filesystem::path& path, float sx, float sy, float sz) : path_(path) {
        const float corners[8][3] = { {0, 0, 0}, {sx, 0, 0}, {sx, sy, 0}, {0, sy, 0}, {0, 0, sz}, {sx, 0, sz}, {sx, sy, sz}, {0, sy, sz} };
        const int triangles[12][3] = { {0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                                       {2, 3, 7}, {2, 7, 6}, {1, 2, 6}, {1, 6, 5}, {0, 4, 7}, {0, 7, 3} };
        std::ofstream file(path_, std::ios::binary);
        const char header[80] = {};
        file.write(header, sizeof(header));
        const std::uint32_t count = 12u;
        file.write(reinterpret_cast<const char*>(&count), sizeof(count));
        for(const auto& triangle : triangles) {
            const float normal[3] = { 0.0f, 0.0f, 0.0f };
            file.write(reinterpret_cast<const char*>(normal), sizeof(normal));
            for(const int corner : triangle) file.write(reinterpret_cast<const char*>(corners[corner]), 3 * sizeof(float));
            const std::uint16_t attributes = 0u;
            file.write(reinterpret_cast<const char*>(&attributes), sizeof(attributes));
        }
    }

    ~BoxStl() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    BoxStl(const BoxStl&) = delete;
    BoxStl& operator=(const BoxStl&) = delete;

    std::string path() const { return path_.string(); }

private:
    std::filesystem::path path_;
};

#ifdef FLUIDX3D_TEST_DIR
// SimulationConfig and Model look geometry files up in resources/ (and exit if they are missing), so geometry for
// them is written to the test build directory and named relative to resources/.
inline std::filesystem::path test_file(const char* name) { return std::filesystem::path(FLUIDX3D_TEST_DIR) / name; }
inline std::string resource_name(const std::string& path) {
    return std::filesystem::relative(path, FLUIDX3D_RESOURCE_DIR).generic_string();
}
#endif // FLUIDX3D_TEST_DIR
