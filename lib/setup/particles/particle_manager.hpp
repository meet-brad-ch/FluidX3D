#pragma once

#include "core/types.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <vector>

extern Units units; // global units object from lbm.cpp

// Seeding patterns in SI units for ParticleManager.
class ParticleSeeder {
public:
    struct Sphere {
        float3 center_m;
        float32_t radius_m;
        uint32_t count;
    };

    // count particles at random positions inside a sphere (center and radius in m)
    ParticleSeeder& sphere(float3 center_m, float32_t radius_m, uint32_t count) {
        spheres_.push_back({center_m, radius_m, count});
        return *this;
    }

    const std::vector<Sphere>& spheres() const { return spheres_; }

private:
    std::vector<Sphere> spheres_;
};

// Places the LBM's particles (PARTICLES extension, LBM created with particles) from seeding patterns in SI units.
// Call initialize() after configure_units() and before the simulation runs. Particles not seeded start at the domain center.
class ParticleManager {
public:
    explicit ParticleManager(LBM& lbm) : lbm_(lbm) {}

    ParticleSeeder& seed() { return seeder_; }

    ParticleManager& set_visualization(bool enable = true) { // VIS_PARTICLES
        enable_visualization_ = enable;
        return *this;
    }

    void initialize() {
        if (lbm_.particles == nullptr) {
            print_warning("ParticleManager: LBM has no particles allocated. "
                         "Create LBM with particles_N > 0.");
            return;
        }

        const uint64_t particle_count = lbm_.particles->length();
        if (particle_count == 0) {
            print_warning("ParticleManager: No particles allocated.");
            return;
        }

        const float32_t center_x = 0.5f * (float32_t)lbm_.get_Nx();
        const float32_t center_y = 0.5f * (float32_t)lbm_.get_Ny();
        const float32_t center_z = 0.5f * (float32_t)lbm_.get_Nz();

        uint64_t particle_idx = 0;
        uint32_t current_seed = 42u;

        for (const auto& s : seeder_.spheres()) {
            for (uint32_t i = 0; i < s.count && particle_idx < particle_count; i++) {
                float3 offset; // random point in the unit sphere (rejection sampling)
                do {
                    offset.x = random_symmetric(current_seed, 1.0f);
                    offset.y = random_symmetric(current_seed, 1.0f);
                    offset.z = random_symmetric(current_seed, 1.0f);
                } while (offset.x*offset.x + offset.y*offset.y + offset.z*offset.z > 1.0f);

                float3 pos_m = s.center_m;
                pos_m.x += offset.x * s.radius_m;
                pos_m.y += offset.y * s.radius_m;
                pos_m.z += offset.z * s.radius_m;
                set_particle_position(particle_idx++, position_to_lbm(pos_m));
            }
        }

        while (particle_idx < particle_count) {
            lbm_.particles->x[particle_idx] = center_x;
            lbm_.particles->y[particle_idx] = center_y;
            lbm_.particles->z[particle_idx] = center_z;
            particle_idx++;
        }

        if (enable_visualization_) {
            lbm_.graphics.visualization_modes |= VIS_PARTICLES;
        }
    }

private:
    LBM& lbm_;
    ParticleSeeder seeder_;
    bool enable_visualization_ = true;

    void set_particle_position(uint64_t index, float3 pos_lbm) {
        if (index < lbm_.particles->length()) {
            lbm_.particles->x[index] = pos_lbm.x;
            lbm_.particles->y[index] = pos_lbm.y;
            lbm_.particles->z[index] = pos_lbm.z;
        }
    }

    float3 position_to_lbm(float3 pos_si) const {
        return float3(units.x(pos_si.x), units.x(pos_si.y), units.x(pos_si.z));
    }

    static float32_t random_symmetric(uint32_t& seed, float32_t magnitude) { // LCG
        seed = seed * 1103515245u + 12345u;
        float32_t r = (float32_t)(seed & 0x7FFFFFFFu) / (float32_t)0x7FFFFFFFu;
        return magnitude * (2.0f * r - 1.0f);
    }
};
