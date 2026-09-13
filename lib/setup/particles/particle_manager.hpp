#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/unit_scale.hpp"
#include "lbm.hpp"
#include <vector>

#ifndef PARTICLES
#error "setup/particles/particle_manager.hpp needs PARTICLES in defines.hpp"
#endif // PARTICLES

/// Seeding patterns for ParticleManager: positions in metres from the domain's origin corner.
class ParticleSeeder {
public:
    struct Pattern {
        enum class Shape { SPHERE, CUBE } shape;
        Position center;
        Length size; ///< the sphere's radius, the cube's half side
        uint32_t count;
    };

    /// count particles at random positions inside a sphere.
    ParticleSeeder& sphere(Position center, Length radius, uint32_t count) {
        patterns_.push_back({ Pattern::Shape::SPHERE, center, radius, count });
        return *this;
    }

    /// count particles at random positions inside an axis-aligned cube.
    ParticleSeeder& cube(Position center, Length half_side, uint32_t count) {
        patterns_.push_back({ Pattern::Shape::CUBE, center, half_side, count });
        return *this;
    }

    const std::vector<Pattern>& patterns() const { return patterns_; }

private:
    std::vector<Pattern> patterns_;
};

/// @brief Places the simulation's particles (PARTICLES extension, Simulation::set_particles()) from seeding patterns
/// in metres (Simulation::particles()). Particles not seeded start at the domain center.
/// @code
/// sim.particles().seed().cube({ 0.5_m, 0.5_m, 0.5_m }, 0.125_m, 32768u);
/// sim.particles().initialize();
/// @endcode
class ParticleManager {
public:
    /// @param lbm   an LBM created with particles
    /// @param unit_scale the simulation's unit scale
    ParticleManager(LBM& lbm, const UnitScale& unit_scale) : lbm_(lbm), units_(unit_scale) {}

    ParticleSeeder& seed() { return seeder_; }

    /// The particles are drawn (VIS_PARTICLES; default).
    ParticleManager& show() {
        visible_ = true;
        return *this;
    }

    /// The particles are not drawn.
    ParticleManager& hide() {
        visible_ = false;
        return *this;
    }

    /// Writes the particles' positions; call before the simulation runs.
    void initialize() {
        if(lbm_.particles == nullptr) {
            print_warning("ParticleManager: LBM has no particles allocated. "
                         "Create LBM with particles_N > 0.");
            return;
        }

        const uint64_t particle_count = lbm_.particles->length();
        if(particle_count == 0) {
            print_warning("ParticleManager: No particles allocated.");
            return;
        }

        uint64_t particle_idx = 0;
        uint seed = 42u; // the core's random generator, seeded as the original examples
        for(const ParticleSeeder::Pattern& pattern : seeder_.patterns()) {
            const float3 center = position_to_lbm(pattern.center);
            for(uint32_t i = 0; i < pattern.count && particle_idx < particle_count; i++) {
                float3 offset; // random point in the unit cube, or (rejection sampling) in the unit sphere
                do {
                    offset.x = random_symmetric(seed, 1.0f);
                    offset.y = random_symmetric(seed, 1.0f);
                    offset.z = random_symmetric(seed, 1.0f);
                } while(pattern.shape == ParticleSeeder::Pattern::Shape::SPHERE &&
                        offset.x*offset.x + offset.y*offset.y + offset.z*offset.z > 1.0f);
                set_particle_position(particle_idx++, center + float3(units_.length(pattern.size * offset.x),
                                                                      units_.length(pattern.size * offset.y),
                                                                      units_.length(pattern.size * offset.z)));
            }
        }
        while(particle_idx < particle_count) set_particle_position(particle_idx++, float3(0.0f)); // the domain center

        if(visible_) {
            lbm_.graphics.visualization_modes |= VIS_PARTICLES;
        }
    }

private:
    LBM& lbm_;
    UnitScale units_;
    ParticleSeeder seeder_;
    bool visible_ = true;

    void set_particle_position(uint64_t index, float3 pos_lbm) {
        if(index < lbm_.particles->length()) {
            lbm_.particles->x[index] = pos_lbm.x;
            lbm_.particles->y[index] = pos_lbm.y;
            lbm_.particles->z[index] = pos_lbm.z;
        }
    }

    // the core's particle positions are in cells from the domain center
    float3 position_to_lbm(const Position& p) const {
        return float3(units_.length(p.x) - 0.5f * (float32_t)lbm_.get_Nx(),
                      units_.length(p.y) - 0.5f * (float32_t)lbm_.get_Ny(),
                      units_.length(p.z) - 0.5f * (float32_t)lbm_.get_Nz());
    }
};
