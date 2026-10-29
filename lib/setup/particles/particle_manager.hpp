#pragma once

#include "setup/core/types.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include "lbm.hpp"
#include <vector>

#ifndef PARTICLES
#error "setup/particles/particle_manager.hpp needs PARTICLES in defines.hpp"
#endif // PARTICLES

/// @file particle_manager.hpp
/// @brief The seeding patterns of the particles (ParticleSeeder) and their placement on the LBM (ParticleManager).

/// @brief Seeding patterns for ParticleManager: particles at random positions in a sphere or a cube, in metres from
/// the domain's origin corner.
class ParticleSeeder {
public:
    /// Particles at random positions in a shape.
    struct Pattern {
        /// The shape's kind.
        enum class Shape {
            SPHERE, ///< a sphere
            CUBE    ///< an axis-aligned cube
        };
        Shape shape = Shape::SPHERE;  ///< the shape's kind
        si::Position center{};        ///< the shape's centre in metres from the origin corner
        si::Length size{ au::ZERO };  ///< the sphere's radius, or half the cube's side
        uint32_t count{};             ///< the number of particles
    };

    /// @brief Adds particles at random positions inside a sphere.
    /// @param center the sphere's centre in metres from the origin corner
    /// @param radius the sphere's radius
    /// @param count the number of particles
    /// @return this seeder
    ParticleSeeder& sphere(si::Position center, si::Length radius, uint32_t count) {
        patterns_.push_back(Pattern{ .shape = Pattern::Shape::SPHERE, .center = center, .size = radius, .count = count });
        return *this;
    }

    /// @brief Adds particles at random positions inside an axis-aligned cube.
    /// @param center the cube's centre in metres from the origin corner
    /// @param half_side half the cube's side
    /// @param count the number of particles
    /// @return this seeder
    ParticleSeeder& cube(si::Position center, si::Length half_side, uint32_t count) {
        patterns_.push_back(Pattern{ .shape = Pattern::Shape::CUBE, .center = center, .size = half_side, .count = count });
        return *this;
    }

    /// @return the patterns, in the order they were added
    const std::vector<Pattern>& patterns() const { return patterns_; }

private:
    std::vector<Pattern> patterns_{}; ///< the patterns, in the order they were added
};

/// @brief Places the simulation's particles (PARTICLES extension, Simulation::set_particles()) from seeding patterns
/// in metres (Simulation::particles()). Particles beyond the patterns start at the domain's centre.
///
/// Owned by Simulation, which makes it with the LBM; it keeps a reference to the LBM. initialize() writes the core's
/// particle positions on the host (in cells from the domain's centre, as the core keeps them) and, with show(), the
/// VIS_PARTICLES graphics flag.
/// @code
/// using namespace si::symbols;
/// sim.particles().seed().cube({ 0.5 * m, 0.5 * m, 0.5 * m }, 0.125 * m, 32768u);
/// sim.particles().initialize();
/// @endcode
class ParticleManager {
public:
    /// The seed of the core's random generator, as in the original examples.
    static constexpr uint first_seed = 42u;

    /// @brief Makes the manager of an LBM.
    /// @param lbm an LBM created with particles
    /// @param converter the simulation's conversions, for the cell size
    ParticleManager(LBM& lbm, const si::LatticeConverter& converter) : lbm_(lbm), converter_(converter) {}

    /// @return the seeder to add patterns to
    ParticleSeeder& seed() { return seeder_; }

    /// @brief Draws the particles (VIS_PARTICLES; the default).
    /// @return this manager
    ParticleManager& show() {
        visible_ = true;
        return *this;
    }

    /// @brief Does not draw the particles.
    /// @return this manager
    ParticleManager& hide() {
        visible_ = false;
        return *this;
    }

    /// @brief Writes the particles' positions; call it before the simulation runs. Patterns with more particles than
    /// the LBM holds are cut at the LBM's count.
    ///
    /// Ends the program with print_error() when the LBM has no particles (Simulation::set_particles() with a count of
    /// 0, or an LBM made without particles): seeding patterns that cannot be placed are an error, not a no-op.
    void initialize() {
        if(lbm_.particles == nullptr) {
            print_error("ParticleManager::initialize(): the LBM has no particles; call Simulation::set_particles() with a count above 0");
        }
        const uint64_t particle_count{ lbm_.particles->length() };
        if(particle_count == 0u) {
            print_error("ParticleManager::initialize(): the LBM has 0 particles; call Simulation::set_particles() with a count above 0");
        }
        uint64_t particle_index{ 0u };
        uint seed{ first_seed };
        for(const ParticleSeeder::Pattern& pattern : seeder_.patterns()) {
            seed_pattern(pattern, particle_count, particle_index, seed);
        }
        const float3 domain_centre{ 0.0f };
        while(particle_index < particle_count) {
            set_particle_position(particle_index, domain_centre);
            particle_index++;
        }
        if(visible_) {
            lbm_.graphics.visualization_modes |= VIS_PARTICLES;
        }
    }

private:
    LBM& lbm_;                       ///< the LBM with the particles (not owned)
    si::LatticeConverter converter_; ///< the simulation's conversions
    ParticleSeeder seeder_{};        ///< the seeding patterns
    bool visible_ = true;            ///< true when the particles are drawn

    /// @brief Places the particles of one pattern.
    /// @param pattern the shape, centre, size and count of the particles to place
    /// @param particle_count the particles the LBM holds
    /// @param particle_index the next particle to place, advanced
    /// @param seed the core's random seed, advanced
    void seed_pattern(const ParticleSeeder::Pattern& pattern, uint64_t particle_count, uint64_t& particle_index, uint& seed) {
        const float3 centre{ position_to_lbm(pattern.center) };
        const bool sphere{ pattern.shape == ParticleSeeder::Pattern::Shape::SPHERE };
        for(uint32_t i = 0u; i < pattern.count && particle_index < particle_count; i++) {
            const float3 offset{ random_offset(seed, sphere) };
            const float x{ static_cast<float>(converter_.cells(pattern.size * static_cast<double>(offset.x))) };
            const float y{ static_cast<float>(converter_.cells(pattern.size * static_cast<double>(offset.y))) };
            const float z{ static_cast<float>(converter_.cells(pattern.size * static_cast<double>(offset.z))) };
            const float3 position{ centre + float3(x, y, z) };
            set_particle_position(particle_index, position);
            particle_index++;
        }
    }

    /// The most draws for one point in the sphere. A draw is inside with the probability pi/6 (0.52), so 64 draws
    /// all outside have the probability 1e-20: more means a broken random generator.
    static constexpr uint32_t max_sphere_draws = 64u;

    /// @brief Gives a random point in the cube from -1 to 1 along each axis, or (rejection sampling) in the unit
    /// sphere.
    ///
    /// Ends the program with print_error() when max_sphere_draws draws are all outside the sphere.
    /// @param seed the core's random seed, advanced
    /// @param sphere true for a point in the unit sphere
    /// @return the point
    static float3 random_offset(uint& seed, bool sphere) {
        float3 offset{};
        for(uint32_t draw = 0u; draw < max_sphere_draws; draw++) {
            offset.x = random_symmetric(seed, 1.0f);
            offset.y = random_symmetric(seed, 1.0f);
            offset.z = random_symmetric(seed, 1.0f);
            const float radius_squared{ offset.x * offset.x + offset.y * offset.y + offset.z * offset.z };
            const bool outside{ sphere && radius_squared > 1.0f };
            if(!outside) {
                return offset;
            }
        }
        print_error("ParticleManager: " + to_string(max_sphere_draws) + " random draws were all outside the unit sphere");
        return offset;
    }

    /// @brief Sets a particle's position, when the index is within the particles.
    /// @param index the particle's index, counted from 0
    /// @param pos_lbm the position in cells from the domain's centre
    void set_particle_position(uint64_t index, float3 pos_lbm) {
        if(index < lbm_.particles->length()) {
            lbm_.particles->x[index] = pos_lbm.x;
            lbm_.particles->y[index] = pos_lbm.y;
            lbm_.particles->z[index] = pos_lbm.z;
        }
    }

    /// @brief Gives a position in the core's particle coordinates: in cells from the domain's centre.
    /// @param p a position in metres from the domain's origin corner
    /// @return p in cells minus half the grid along each axis
    float3 position_to_lbm(const si::Position& p) const {
        const double half_x{ 0.5 * static_cast<double>(lbm_.get_Nx()) };
        const double half_y{ 0.5 * static_cast<double>(lbm_.get_Ny()) };
        const double half_z{ 0.5 * static_cast<double>(lbm_.get_Nz()) };
        const double x{ converter_.cells(p.x) - half_x };
        const double y{ converter_.cells(p.y) - half_y };
        const double z{ converter_.cells(p.z) - half_z };
        const float3 position{ static_cast<float>(x), static_cast<float>(y), static_cast<float>(z) };
        return position;
    }
};
