#pragma once

#include "core/types.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <vector>
#include <functional>
#include <cmath>

extern Units units;  // Global units object from lbm.cpp

/**
 * @file particle_manager.hpp
 * @brief Fluent particle seeding and management for FluidX3D simulations
 *
 * Provides a readable, SI-unit based API for setting up and managing
 * particle tracer simulations including seeding patterns, visualization,
 * and position export.
 *
 * @note Requires PARTICLES extension enabled in defines.hpp
 * @note For 2-way coupling, also requires VOLUME_FORCE and FORCE_FIELD
 */

/**
 * @class ParticleSeeder
 * @brief Helper class for defining particle seeding patterns
 *
 * This class defines seeding patterns that are applied during initialization.
 * All positions are specified in SI units (meters) and converted to LBM units.
 */
class ParticleSeeder {
public:
    /**
     * @brief Seed particles at a single point
     * @param position_m Position in meters (SI units)
     * @param count Number of particles to seed at this location
     * @return Reference for method chaining
     */
    ParticleSeeder& point(float3 position_m, uint32_t count = 1) {
        SeedPoint sp;
        sp.type = SeedType::POINT;
        sp.position = position_m;
        sp.count = count;
        seeds_.push_back(sp);
        return *this;
    }

    /**
     * @brief Seed particles along a line
     * @param start_m Start position in meters
     * @param end_m End position in meters
     * @param count Number of particles along the line
     * @return Reference for method chaining
     */
    ParticleSeeder& line(float3 start_m, float3 end_m, uint32_t count) {
        SeedPoint sp;
        sp.type = SeedType::LINE;
        sp.position = start_m;
        sp.end_position = end_m;
        sp.count = count;
        seeds_.push_back(sp);
        return *this;
    }

    /**
     * @brief Seed particles in a plane (rectangular grid)
     * @param center_m Center position in meters
     * @param normal Normal direction of the plane
     * @param width_m Width of the plane in meters
     * @param height_m Height of the plane in meters
     * @param count_x Number of particles along width
     * @param count_y Number of particles along height
     * @return Reference for method chaining
     */
    ParticleSeeder& plane(float3 center_m, Axis normal, float32_t width_m, float32_t height_m,
                          uint32_t count_x, uint32_t count_y) {
        SeedPoint sp;
        sp.type = SeedType::PLANE;
        sp.position = center_m;
        sp.axis = normal;
        sp.width = width_m;
        sp.height = height_m;
        sp.count_x = count_x;
        sp.count_y = count_y;
        sp.count = count_x * count_y;
        seeds_.push_back(sp);
        return *this;
    }

    /**
     * @brief Seed particles in a circular disk
     * @param center_m Center position in meters
     * @param normal Normal direction of the disk
     * @param radius_m Radius of the disk in meters
     * @param count Number of particles (distributed uniformly)
     * @return Reference for method chaining
     */
    ParticleSeeder& disk(float3 center_m, Axis normal, float32_t radius_m, uint32_t count) {
        SeedPoint sp;
        sp.type = SeedType::DISK;
        sp.position = center_m;
        sp.axis = normal;
        sp.radius = radius_m;
        sp.count = count;
        seeds_.push_back(sp);
        return *this;
    }

    /**
     * @brief Seed particles in a spherical region
     * @param center_m Center position in meters
     * @param radius_m Radius of the sphere in meters
     * @param count Number of particles (distributed uniformly)
     * @return Reference for method chaining
     */
    ParticleSeeder& sphere(float3 center_m, float32_t radius_m, uint32_t count) {
        SeedPoint sp;
        sp.type = SeedType::SPHERE;
        sp.position = center_m;
        sp.radius = radius_m;
        sp.count = count;
        seeds_.push_back(sp);
        return *this;
    }

    /**
     * @brief Seed particles in a box volume
     * @param min_m Minimum corner in meters
     * @param max_m Maximum corner in meters
     * @param count Number of particles (distributed uniformly)
     * @return Reference for method chaining
     */
    ParticleSeeder& box(float3 min_m, float3 max_m, uint32_t count) {
        SeedPoint sp;
        sp.type = SeedType::BOX;
        sp.position = min_m;
        sp.end_position = max_m;
        sp.count = count;
        seeds_.push_back(sp);
        return *this;
    }

    /**
     * @brief Seed particles using a custom predicate
     * @param predicate Function that returns true for positions where particles should be seeded
     * @param count Number of particles to attempt placing
     * @param bounds_min_m Minimum bounds for sampling in meters
     * @param bounds_max_m Maximum bounds for sampling in meters
     * @return Reference for method chaining
     */
    ParticleSeeder& custom(std::function<bool(float3)> predicate,
                           uint32_t count, float3 bounds_min_m, float3 bounds_max_m) {
        SeedPoint sp;
        sp.type = SeedType::CUSTOM;
        sp.position = bounds_min_m;
        sp.end_position = bounds_max_m;
        sp.count = count;
        sp.predicate = predicate;
        seeds_.push_back(sp);
        return *this;
    }

    /**
     * @brief Get total number of particles to be seeded
     * @return Total particle count
     */
    uint32_t total_count() const {
        uint32_t total = 0;
        for (const auto& s : seeds_) {
            total += s.count;
        }
        return total;
    }

    enum class SeedType { POINT, LINE, PLANE, DISK, SPHERE, BOX, CUSTOM };

    struct SeedPoint {
        SeedType type;
        float3 position{0.0f, 0.0f, 0.0f};
        float3 end_position{0.0f, 0.0f, 0.0f};
        Axis axis = Axis::Z;
        float32_t radius = 0.0f;
        float32_t width = 0.0f;
        float32_t height = 0.0f;
        uint32_t count = 0;
        uint32_t count_x = 0;
        uint32_t count_y = 0;
        std::function<bool(float3)> predicate;
    };

    const std::vector<SeedPoint>& seeds() const { return seeds_; }

private:
    std::vector<SeedPoint> seeds_;
};

/**
 * @class ParticleManager
 * @brief Fluent API for particle management in FluidX3D simulations
 *
 * This class simplifies setting up particle simulations including:
 * - SI-unit based particle seeding
 * - Various seeding patterns (point, line, plane, volume)
 * - Particle visualization configuration
 * - Position export in SI units
 *
 * @par Example (tracer particles in lid-driven cavity):
 * @code
 * // Setup units first
 * units.set_m_kg_s(L-2, u, 1.0f, domain_size_m, velocity_mps, rho);
 *
 * ParticleManager particles(lbm);
 * particles.seed()
 *     .sphere(float3(0.5f, 0.5f, 0.5f), 0.125f, 1000);  // Center, 1/8 domain radius
 * particles.set_visualization(true);
 * particles.initialize();
 * @endcode
 *
 * @par Example (particles seeded along inlet):
 * @code
 * ParticleManager particles(lbm);
 * particles.seed()
 *     .plane(float3(0.1f, 0.5f, 0.5f), Axis::X, 0.4f, 0.4f, 10, 10);
 * particles.initialize();
 * @endcode
 */
class ParticleManager {
public:
    /**
     * @brief Construct ParticleManager for an LBM simulation
     * @param lbm Reference to the LBM simulation object
     *
     * @note The LBM object must have been constructed with particles_N > 0
     */
    explicit ParticleManager(LBM& lbm) : lbm_(lbm) {}

    // ========================================================================
    // Seeding Configuration
    // ========================================================================

    /**
     * @brief Access the particle seeder for configuring seed patterns
     * @return Reference to ParticleSeeder for method chaining
     *
     * @par Example:
     * @code
     * particles.seed()
     *     .line(start, end, 100)
     *     .sphere(center, radius, 500);
     * @endcode
     */
    ParticleSeeder& seed() {
        return seeder_;
    }

    /**
     * @brief Set the seed for random number generation
     * @param s Seed value (default: 42)
     * @return Reference for method chaining
     */
    ParticleManager& set_random_seed(uint32_t s) {
        seed_ = s;
        return *this;
    }

    // ========================================================================
    // Visualization
    // ========================================================================

    /**
     * @brief Enable or disable particle visualization
     * @param enable True to enable particle rendering (default: true)
     * @return Reference for method chaining
     *
     * @note Uses VIS_PARTICLES visualization mode
     */
    ParticleManager& set_visualization(bool enable = true) {
        enable_visualization_ = enable;
        return *this;
    }

    // ========================================================================
    // Initialization and Update
    // ========================================================================

    /**
     * @brief Initialize particles with configured seeding patterns
     *
     * This method applies all configured seeding patterns and sets up
     * particle visualization if enabled.
     *
     * @note Must be called after LBM is created but before simulation runs
     * @note Units must be configured before calling this method
     */
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

        // Get domain center in LBM units for reference
        const float32_t center_x = 0.5f * (float32_t)lbm_.get_Nx();
        const float32_t center_y = 0.5f * (float32_t)lbm_.get_Ny();
        const float32_t center_z = 0.5f * (float32_t)lbm_.get_Nz();

        uint64_t particle_idx = 0;
        uint32_t current_seed = seed_;

        // Apply each seeding pattern
        for (const auto& s : seeder_.seeds()) {
            switch (s.type) {
                case ParticleSeeder::SeedType::POINT:
                    for (uint32_t i = 0; i < s.count && particle_idx < particle_count; i++) {
                        float3 pos_lbm = position_to_lbm(s.position);
                        // Add small random jitter for multiple particles at same point
                        if (s.count > 1) {
                            pos_lbm.x += random_symmetric(current_seed, 0.5f);
                            pos_lbm.y += random_symmetric(current_seed, 0.5f);
                            pos_lbm.z += random_symmetric(current_seed, 0.5f);
                        }
                        set_particle_position(particle_idx++, pos_lbm);
                    }
                    break;

                case ParticleSeeder::SeedType::LINE:
                    for (uint32_t i = 0; i < s.count && particle_idx < particle_count; i++) {
                        float32_t t = (s.count > 1) ? (float32_t)i / (float32_t)(s.count - 1) : 0.5f;
                        float3 pos_m = lerp(s.position, s.end_position, t);
                        set_particle_position(particle_idx++, position_to_lbm(pos_m));
                    }
                    break;

                case ParticleSeeder::SeedType::PLANE:
                    for (uint32_t iy = 0; iy < s.count_y && particle_idx < particle_count; iy++) {
                        for (uint32_t ix = 0; ix < s.count_x && particle_idx < particle_count; ix++) {
                            float32_t tx = (s.count_x > 1) ? (float32_t)ix / (float32_t)(s.count_x - 1) - 0.5f : 0.0f;
                            float32_t ty = (s.count_y > 1) ? (float32_t)iy / (float32_t)(s.count_y - 1) - 0.5f : 0.0f;
                            float3 pos_m = s.position;
                            // Offset based on plane orientation
                            switch (s.axis) {
                                case Axis::X: pos_m.y += tx * s.width; pos_m.z += ty * s.height; break;
                                case Axis::Y: pos_m.x += tx * s.width; pos_m.z += ty * s.height; break;
                                case Axis::Z: pos_m.x += tx * s.width; pos_m.y += ty * s.height; break;
                            }
                            set_particle_position(particle_idx++, position_to_lbm(pos_m));
                        }
                    }
                    break;

                case ParticleSeeder::SeedType::DISK:
                    for (uint32_t i = 0; i < s.count && particle_idx < particle_count; i++) {
                        // Use Fibonacci spiral for uniform distribution
                        float32_t r = s.radius * std::sqrt((float32_t)i / (float32_t)s.count);
                        float32_t theta = (float32_t)i * 2.4f; // Golden angle approximation
                        float3 pos_m = s.position;
                        switch (s.axis) {
                            case Axis::X: pos_m.y += r * std::cos(theta); pos_m.z += r * std::sin(theta); break;
                            case Axis::Y: pos_m.x += r * std::cos(theta); pos_m.z += r * std::sin(theta); break;
                            case Axis::Z: pos_m.x += r * std::cos(theta); pos_m.y += r * std::sin(theta); break;
                        }
                        set_particle_position(particle_idx++, position_to_lbm(pos_m));
                    }
                    break;

                case ParticleSeeder::SeedType::SPHERE:
                    for (uint32_t i = 0; i < s.count && particle_idx < particle_count; i++) {
                        // Random point in unit sphere using rejection sampling
                        float3 offset;
                        do {
                            offset.x = random_symmetric(current_seed, 1.0f);
                            offset.y = random_symmetric(current_seed, 1.0f);
                            offset.z = random_symmetric(current_seed, 1.0f);
                        } while (offset.x*offset.x + offset.y*offset.y + offset.z*offset.z > 1.0f);

                        float3 pos_m = s.position;
                        pos_m.x += offset.x * s.radius;
                        pos_m.y += offset.y * s.radius;
                        pos_m.z += offset.z * s.radius;
                        set_particle_position(particle_idx++, position_to_lbm(pos_m));
                    }
                    break;

                case ParticleSeeder::SeedType::BOX:
                    for (uint32_t i = 0; i < s.count && particle_idx < particle_count; i++) {
                        float3 pos_m;
                        pos_m.x = s.position.x + random_uniform(current_seed) * (s.end_position.x - s.position.x);
                        pos_m.y = s.position.y + random_uniform(current_seed) * (s.end_position.y - s.position.y);
                        pos_m.z = s.position.z + random_uniform(current_seed) * (s.end_position.z - s.position.z);
                        set_particle_position(particle_idx++, position_to_lbm(pos_m));
                    }
                    break;

                case ParticleSeeder::SeedType::CUSTOM:
                    if (s.predicate) {
                        uint32_t attempts = 0;
                        const uint32_t max_attempts = s.count * 100;
                        while (particle_idx < particle_count &&
                               (particle_idx - (particle_count - s.count)) < s.count &&
                               attempts < max_attempts) {
                            float3 pos_m;
                            pos_m.x = s.position.x + random_uniform(current_seed) * (s.end_position.x - s.position.x);
                            pos_m.y = s.position.y + random_uniform(current_seed) * (s.end_position.y - s.position.y);
                            pos_m.z = s.position.z + random_uniform(current_seed) * (s.end_position.z - s.position.z);
                            if (s.predicate(pos_m)) {
                                set_particle_position(particle_idx++, position_to_lbm(pos_m));
                            }
                            attempts++;
                        }
                    }
                    break;
            }
        }

        // Fill remaining particles at domain center (if any)
        while (particle_idx < particle_count) {
            lbm_.particles->x[particle_idx] = center_x;
            lbm_.particles->y[particle_idx] = center_y;
            lbm_.particles->z[particle_idx] = center_z;
            particle_idx++;
        }

        // Configure visualization
        if (enable_visualization_) {
            lbm_.graphics.visualization_modes |= VIS_PARTICLES;
        }

        initialized_ = true;
    }

    /**
     * @brief Update particles (integrate forward in time)
     *
     * Call this in the simulation loop to advance particles.
     * For passive tracers in stationary flow, this is optional.
     */
    void update() {
        // Particle integration is handled by LBM kernel automatically
        // This method is provided for consistency with other managers
    }

    // ========================================================================
    // Position Access
    // ========================================================================

    /**
     * @brief Get all particle positions in SI units (meters)
     * @return Vector of particle positions in SI coordinates
     */
    std::vector<float3> get_positions_si() const {
        std::vector<float3> positions;
        if (lbm_.particles == nullptr) return positions;

        const uint64_t count = lbm_.particles->length();
        positions.reserve(count);

        for (uint64_t i = 0; i < count; i++) {
            float3 pos_lbm(lbm_.particles->x[i], lbm_.particles->y[i], lbm_.particles->z[i]);
            positions.push_back(position_to_si(pos_lbm));
        }

        return positions;
    }

    /**
     * @brief Get all particle positions in LBM units (cells)
     * @return Vector of particle positions in LBM coordinates
     */
    std::vector<float3> get_positions_lbm() const {
        std::vector<float3> positions;
        if (lbm_.particles == nullptr) return positions;

        const uint64_t count = lbm_.particles->length();
        positions.reserve(count);

        for (uint64_t i = 0; i < count; i++) {
            positions.push_back(float3(
                lbm_.particles->x[i],
                lbm_.particles->y[i],
                lbm_.particles->z[i]
            ));
        }

        return positions;
    }

    /**
     * @brief Get single particle position in SI units
     * @param index Particle index
     * @return Position in SI coordinates (meters)
     */
    float3 get_position_si(uint64_t index) const {
        if (lbm_.particles == nullptr || index >= lbm_.particles->length()) {
            return float3(0.0f, 0.0f, 0.0f);
        }
        float3 pos_lbm(lbm_.particles->x[index], lbm_.particles->y[index], lbm_.particles->z[index]);
        return position_to_si(pos_lbm);
    }

    /**
     * @brief Get number of particles
     * @return Particle count
     */
    uint64_t count() const {
        return lbm_.particles ? lbm_.particles->length() : 0;
    }

    /**
     * @brief Check if particle manager is initialized
     * @return True if initialize() has been called
     */
    bool is_initialized() const { return initialized_; }

private:
    LBM& lbm_;
    ParticleSeeder seeder_;
    uint32_t seed_ = 42u;
    bool enable_visualization_ = true;
    bool initialized_ = false;

    // Set particle position in LBM units
    void set_particle_position(uint64_t index, float3 pos_lbm) {
        if (index < lbm_.particles->length()) {
            lbm_.particles->x[index] = pos_lbm.x;
            lbm_.particles->y[index] = pos_lbm.y;
            lbm_.particles->z[index] = pos_lbm.z;
        }
    }

    // Convert SI position to LBM position
    float3 position_to_lbm(float3 pos_si) const {
        // Use units object for conversion
        return float3(
            units.x(pos_si.x),
            units.x(pos_si.y),
            units.x(pos_si.z)
        );
    }

    // Convert LBM position to SI position
    float3 position_to_si(float3 pos_lbm) const {
        // Use units object for conversion
        return float3(
            units.si_x(pos_lbm.x),
            units.si_x(pos_lbm.y),
            units.si_x(pos_lbm.z)
        );
    }

    // Linear interpolation helper
    static float3 lerp(float3 a, float3 b, float32_t t) {
        return float3(
            a.x + t * (b.x - a.x),
            a.y + t * (b.y - a.y),
            a.z + t * (b.z - a.z)
        );
    }

    // Random number helpers (simple LCG for thread-safety)
    static float32_t random_symmetric(uint32_t& seed, float32_t magnitude) {
        seed = seed * 1103515245u + 12345u;
        float32_t r = (float32_t)(seed & 0x7FFFFFFFu) / (float32_t)0x7FFFFFFFu;
        return magnitude * (2.0f * r - 1.0f);
    }

    static float32_t random_uniform(uint32_t& seed) {
        seed = seed * 1103515245u + 12345u;
        return (float32_t)(seed & 0x7FFFFFFFu) / (float32_t)0x7FFFFFFFu;
    }
};
