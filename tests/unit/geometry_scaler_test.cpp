#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/geometry_scaler.hpp"
#include "box_stl.hpp"

#include <cmath>
#include <filesystem>
#include <string>

namespace {

using namespace si::symbols;

constexpr LatticeMemory d3q19_fp16 { au::bytes(55.0) };
const GeometryScaler::Clearances no_clearances{};
const GeometryScaler::Clearances wide_clearances{ .bottom = 0.0 * m, .top = 10.0 * m, .sides = 10.0 * m };
const si::Length cell_10cm{ 0.1 * m };

TEST(GeometryScaler, ReadsTheGeometrySizeInMetres) {
    const BoxStl box("fluidx3d_test_box_size.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler scaler(box.path(), cell_10cm, 20000.0 * MiB, no_clearances, d3q19_fp16);
    const si::Position size{ scaler.stl_size() };
    EXPECT_NEAR(size.x.in(m), 4.0, 1e-5);
    EXPECT_NEAR(size.y.in(m), 2.0, 1e-5);
    EXPECT_NEAR(size.z.in(m), 1.0, 1e-5);
    EXPECT_NEAR(scaler.reference_length().in(m), 2.0, 1e-5); // reference axis Y
}

// Measured as the core voxelizes it, after the rotation (ahmed_body's domain had its X and Y swapped).
TEST(GeometryScaler, MeasuresTheRotatedGeometry) {
    const BoxStl box("fluidx3d_test_box_rotated.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler scaler(box.path(), cell_10cm, 20000.0 * MiB, no_clearances, d3q19_fp16, float3x3(float3(0, 0, 1), radians(90.0f)));
    const si::Position size{ scaler.stl_size() };
    EXPECT_NEAR(size.x.in(m), 2.0, 1e-5);
    EXPECT_NEAR(size.y.in(m), 4.0, 1e-5);
    EXPECT_NEAR(size.z.in(m), 1.0, 1e-5);
}

TEST(GeometryScaler, CellSizeModeCountsCellsFromTheCellSize) {
    const BoxStl box("fluidx3d_test_box_cells.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler scaler(box.path(), cell_10cm, 20000.0 * MiB, no_clearances, d3q19_fp16);
    const uint3 cells = scaler.stl_cells();
    EXPECT_EQ(cells.x, 40u);
    EXPECT_EQ(cells.y, 20u);
    EXPECT_EQ(cells.z, 10u);
    EXPECT_FLOAT_EQ(scaler.reference_cells(), 20.0f);
    EXPECT_EQ(scaler.cell_size().in(m), static_cast<double>(0.1f)); // the scaler's float metres
}

TEST(GeometryScaler, ClearancesAddCellsAroundTheGeometry) {
    const BoxStl box("fluidx3d_test_box_clearance.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler scaler(box.path(), cell_10cm, 20000.0 * MiB, no_clearances, d3q19_fp16);
    const GeometryScaler::Clearances clearances{ .bottom = 0.5 * m, .top = 1.0 * m, .sides = 0.3 * m };
    const uint3 domain = scaler.calculate_domain_size(clearances);
    EXPECT_EQ(domain.x, 40u + 2u * 3u);
    EXPECT_EQ(domain.y, 20u + 2u * 3u);
    EXPECT_EQ(domain.z, 10u + 5u + 10u);
    const float3 center = scaler.calculate_center(domain, clearances); // centered in x/y, resting on the bottom clearance
    EXPECT_FLOAT_EQ(center.x, 22.5f); // the core's lbm.center(): cell i's center is at i
    EXPECT_FLOAT_EQ(center.y, 12.5f);
    EXPECT_FLOAT_EQ(center.z, 5.0f + 5.0f);
}

TEST(GeometryScaler, VramModeFitsGeometryAndClearancesIntoTheBudget) {
    const BoxStl box("fluidx3d_test_box_vram.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler::Clearances clearances{ .bottom = 0.5 * m, .top = 1.0 * m, .sides = 0.3 * m };
    const si::MemoryAmount budget{ 100.0 * MiB };
    const GeometryScaler scaler(box.path(), budget, clearances, d3q19_fp16);
    const uint3 domain = scaler.calculate_domain_size(clearances);
    const si::MemoryAmount used{ d3q19_fp16.required_memory({ domain.x, domain.y, domain.z }) };
    EXPECT_LE(used, 102.0 * MiB); // whole cells: rounding the clearances can add a little
    EXPECT_GE(used, 90.0 * MiB);
    const double cell_m{ scaler.cell_size().in(m) };
    const double cells_y{ static_cast<double>(scaler.stl_cells().y) };
    EXPECT_NEAR(cell_m * cells_y, 2.0, cell_m);
}

// Regression for review A8: the cell-size mode checked only the geometry's own cells against the memory limit.
TEST(GeometryScaler, CellSizeModeCountsTheClearancesInTheMemoryLimit) {
    const BoxStl box("fluidx3d_test_box_limit.stl", 4.0f, 2.0f, 1.0f);
    // 0.1 m cells: 40 x 20 x 10 cells alone (0.4 MB); with the clearances 240 x 220 x 110 cells (304 MB)
    EXPECT_NO_THROW(GeometryScaler(box.path(), cell_10cm, 100.0 * MiB, no_clearances, d3q19_fp16));
    EXPECT_THROW(GeometryScaler(box.path(), cell_10cm, 100.0 * MiB, wide_clearances, d3q19_fp16), SetupError);
}

TEST(GeometryScaler, MemoryLimitErrorNamesTheNumbersAndACellSizeThatFits) {
    const BoxStl box("fluidx3d_test_box_message.stl", 4.0f, 2.0f, 1.0f);
    try {
        GeometryScaler(box.path(), cell_10cm, 100.0 * MiB, wide_clearances, d3q19_fp16);
        FAIL() << "expected a SetupError";
    } catch(const SetupError& error) {
        const std::string message = error.what();
        EXPECT_NE(message.find("240 x 220 x 110"), std::string::npos) << message;
        EXPECT_NE(message.find("304 MB"), std::string::npos) << message;
        EXPECT_NE(message.find("100 MB"), std::string::npos) << message;
    }
    // the suggestion: (24 x 22 x 11 m³ / cells in 100 MB)^(1/3) = 0.145 m; a little larger fits
    const float min_cell_size = std::cbrt(24.0f * 22.0f * 11.0f / (100.0f * 1048576.0f / 55.0f));
    const double larger_cell_m{ static_cast<double>(1.05f * min_cell_size) };
    EXPECT_NO_THROW(GeometryScaler(box.path(), larger_cell_m * m, 100.0 * MiB, wide_clearances, d3q19_fp16));
}

TEST(GeometryScaler, MissingStlThrows) {
    const std::string missing = (std::filesystem::temp_directory_path() / "fluidx3d_test_missing.stl").string();
    EXPECT_THROW(GeometryScaler(missing, cell_10cm, 100.0 * MiB, no_clearances, d3q19_fp16), SetupError);
    EXPECT_THROW(GeometryScaler(missing, 100.0 * MiB, no_clearances, d3q19_fp16), SetupError);
}

} // namespace
