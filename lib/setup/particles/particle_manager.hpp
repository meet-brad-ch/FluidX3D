#pragma once

#include "setup/core/types.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <vector>

#ifndef PARTICLES
#error "setup/particles/particle_manager.hpp needs PARTICLES in defines.hpp"
#endif // PARTICLES

extern Units units; // global units object from lbm.cpp

// Seeding patterns in SI units for ParticleManager; positions in m from the domain's corner at the origin.
class ParticleSeeder {
public:
    struct Pattern {
        enum class Shape { SPHERE, CUBE } shape;
        float3 center_m;
        float32_t size_m; // the sphere's radius, the cube's half side
        uint32_t count;
    };

    // count particles at random positions inside a sphere (center and radius in m)
    ParticleSeeder& sphere(float3 center_m, float32_t radius_m, uint32_t count) {
        patterns_.push_back({ Pattern::Shape::SPHERE, center_m, radius_m, count });
        return *this;
    }

    // count particles at random positions inside an axis-aligned cube (center and half side in m)
    ParticleSeeder& cube(float3 center_m, float32_t half_side_m, uint32_t count) {
        patterns_.push_back({ Pattern::Shape::CUBE, center_m, half_side_m, count });
        return *this;
    }

    const std::vector<Pattern>& patterns() const { return patterns_; }

private:
    std::vector<Pattern> patterns_;
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

        uint64_t particle_idx = 0;
        uint seed = 42u; // the core's random generator, seeded as the original examples
        for (const ParticleSeeder::Pattern& pattern : seeder_.patterns()) {
            const float3 center = position_to_lbm(pattern.center_m);
            for (uint32_t i = 0; i < pattern.count && particle_idx < particle_count; i++) {
                float3 offset; // random point in the unit cube, or (rejection sampling) in the unit sphere
                do {
                    offset.x = random_symmetric(seed, 1.0f);
                    offset.y = random_symmetric(seed, 1.0f);
                    offset.z = random_symmetric(seed, 1.0f);
                } while (pattern.shape == ParticleSeeder::Pattern::Shape::SPHERE &&
                         offset.x*offset.x + offset.y*offset.y + offset.z*offset.z > 1.0f);
                set_particle_position(particle_idx++, center + float3(units.x(pattern.size_m * offset.x),
                                                                      units.x(pattern.size_m * offset.y),
                                                                      units.x(pattern.size_m * offset.z)));
            }
        }
        while (particle_idx < particle_count) set_particle_position(particle_idx++, float3(0.0f)); // the domain center

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

    // the core's particle positions are in cells from the domain center
    float3 position_to_lbm(float3 pos_si) const {
        return float3(units.x(pos_si.x) - 0.5f * (float32_t)lbm_.get_Nx(),
                      units.x(pos_si.y) - 0.5f * (float32_t)lbm_.get_Ny(),
                      units.x(pos_si.z) - 0.5f * (float32_t)lbm_.get_Nz());
    }
};
