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
			for(const int corner : triangle) file.write(reinterpret_cast<const char*>(corners[corner]), 3 * sizeof(float));
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
	std::uint64_t west = 0u, east = 0u;
};

/// Voxelizes the box in a lattice of `domains` along X and counts its solid cells in each half.
SolidHalves voxelized(const std::string& model, std::uint32_t domains) {
	Simulation sim(Domain::around(Model(model)).clearances(0.5_m, 1.0_m, 0.3_m).cell_size(0.1_m).vram_limit(2000_mb).gpus(domains, 1u, 1u),
	               Fluid::AIR, Speed::from_si(1.0f));
	const FieldReader fields = sim.fields();
	const float middle = 0.5f * (float)sim.lbm().get_Nx() * 0.1f;
	SolidHalves halves;
	fields.for_each_cell([&](const FieldReader::Cell& c) {
		if(!c.solid) return;
		if(c.center.x.si() < middle) halves.west++;
		else halves.east++;
	});
	return halves;
}

} // namespace

void main_setup() { // extensions: none
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
	PhysicsCheck::report("SDF voxelization in two domains", error, 1E-9);
}
