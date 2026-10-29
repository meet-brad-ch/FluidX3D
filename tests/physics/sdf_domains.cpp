// SDF voxelization in a lattice split into domains: a model voxelized through its SDF (Domain::around(...).clearances(),
// which converts the STL to a cached SDF) is the same solid cells whether the lattice is one domain or two along X.
// Passes when both halves hold the same solid cells either way (no difference at all). The kernel once placed every
// domain's cells as if each were the first domain, so the second domain held no model.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

namespace {

/// A binary STL of an axis-aligned box from the origin to (sx, sy, sz), in the resources folder, removed at the end.
class ResourceBox {
public:
	ResourceBox(const std::string& name, float sx, float sy, float sz) : name_(name), path_(std::filesystem::path(FLUIDX3D_RESOURCE_DIR) / name) {
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
			for(const int corner : triangle) {
				file.write(reinterpret_cast<const char*>(corners[corner]), 3 * sizeof(float));
			}
			const std::uint16_t attributes = 0u;
			file.write(reinterpret_cast<const char*>(&attributes), sizeof(attributes));
		}
	}
	~ResourceBox() {
		std::error_code ignored;
		std::filesystem::remove(path_, ignored);
	}
	ResourceBox(const ResourceBox&) = delete;
	ResourceBox& operator=(const ResourceBox&) = delete;
	const std::string& name() const { return name_; }

private:
	std::string name_;
	std::filesystem::path path_;
};

/// The solid cells west and east of the lattice's middle.
struct SolidHalves {
	std::uint64_t west{ 0u }; ///< the solid cells west of the middle
	std::uint64_t east{ 0u }; ///< the solid cells east of the middle
};

/// Voxelizes the box in a lattice of `domains` along X and counts its solid cells in each half.
SolidHalves voxelized(const std::string& model, std::uint32_t domains) {
	using namespace si::symbols;
	const si::Length cell = 0.1 * m;
	Domain domain = Domain::around(Model(model));
	const si::Length clearance_below = 0.5 * m;
	const si::Length clearance_above = 1.0 * m;
	const si::Length clearance_sides = 0.3 * m;
	const si::MemoryAmount memory_limit = 2000.0 * MiB;
	domain.clearances(clearance_below, clearance_above, clearance_sides).cell_size(cell).vram_limit(memory_limit).gpus(domains, 1u, 1u);
	const si::Speed reference_speed = 1.0 * m / s;
	Simulation sim(domain, Fluid::AIR, reference_speed);
	const FieldReader fields = sim.fields();
	const double nx = static_cast<double>(sim.lbm().get_Nx());
	const si::Length middle = 0.5 * nx * cell;
	SolidHalves halves;
	const std::uint64_t count{ fields.cell_count() };
	for(std::uint64_t n{ 0u }; n < count; ++n) {
		const FieldReader::Cell c{ fields.cell(n) };
		if(!c.solid) {
			continue;
		}
		if(c.center.x < middle) {
			halves.west++;
		} else {
			halves.east++;
		}
	}
	return halves;
}

} // namespace

void main_setup() { // extensions: none
	using namespace si::symbols;
	const ResourceBox box("fluidx3d_test_sdf_domains.stl", 4.0f, 2.0f, 1.0f);
	const SolidHalves whole = voxelized(box.name(), 1u);
	const SolidHalves split = voxelized(box.name(), 2u);
	std::printf("\nsolid cells west/east: one domain %llu/%llu, two domains %llu/%llu\n", (unsigned long long)whole.west,
	            (unsigned long long)whole.east, (unsigned long long)split.west, (unsigned long long)split.east);
	const double total = (double)(whole.west + whole.east);
	const double difference = (double)((whole.west > split.west ? whole.west - split.west : split.west - whole.west) +
	                                   (whole.east > split.east ? whole.east - split.east : split.east - whole.east));
	// the model in both halves, and the same cells in each: a relative difference of exactly 0 passes
	const double error = (whole.west == 0u || whole.east == 0u) ? 1.0 : difference / total;
	const double error_limit{ 1E-9 }; // exactly the same cells
	PhysicsCheck::report("SDF voxelization in two domains", error, error_limit);
}
