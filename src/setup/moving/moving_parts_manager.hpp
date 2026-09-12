#pragma once

#include "core/types.hpp"
#include "lbm.hpp"
#include "moving/moving_part.hpp"
#include "simulation/simulation_setup.hpp"
#include <vector>
#include <memory>

/**
 * @class MovingPartsManager
 * @brief Manages multiple moving parts and their simulation loop integration
 *
 * This class:
 * - Loads and scales STL meshes to match parent geometry
 * - Converts SI units to LBM units for angular velocities
 * - Provides simple update() method for simulation loop
 * - Handles unvoxelize/voxelize for tumbling parts
 *
 * @par Example - Helicopter with two rotors:
 * @code
 * SimulationSetup sim(SimulationConfig("bell_222_body.stl").set_vram_mb(8000));
 * sim.setup();
 * sim.configure_units(tip_speed_mps, Fluid::AIR);
 * LBM lbm = sim.create_lbm(Fluid::AIR);
 * sim.voxelize(lbm);
 *
 * MovingPartsManager parts(sim, lbm);
 * parts.add(MovingPart("main_rotor.stl")
 *           .set_rotation_axis(RotationAxis::Z)
 *           .set_rpm(348.0f));
 * parts.add(MovingPart("tail_rotor.stl")
 *           .set_rotation_axis(RotationAxis::X)
 *           .set_rpm(-500.0f));
 * parts.initialize();
 *
 * while(lbm.get_t() <= lbm_T) {
 *     parts.update();   // Re-voxelizes parts that need it
 *     lbm.run(4);       // Run 4 timesteps
 * }
 * @endcode
 *
 * @par Example - Propeller aircraft:
 * @code
 * MovingPartsManager parts(sim, lbm);
 * parts.add(MovingPart("propeller.stl")
 *           .set_rotation_axis(RotationAxis::Y)
 *           .set_tip_speed_mps(100.0f)
 *           .set_update_interval(4));
 * parts.initialize();
 *
 * while(lbm.get_t() <= lbm_T) {
 *     parts.update();
 *     lbm.run(4);  // Run 4 timesteps (matching update interval)
 * }
 * @endcode
 */
class MovingPartsManager {
public:
    /**
     * @brief Construct manager with reference to SimulationSetup and LBM
     * @param setup Reference to SimulationSetup (for scaling/units)
     * @param lbm Reference to LBM simulation
     */
    MovingPartsManager(SimulationSetup& setup, LBM& lbm)
        : setup_(setup), lbm_(lbm) {}

    /**
     * @brief Add a moving part to the manager
     * @param part MovingPart configuration
     * @return Reference for method chaining
     *
     * Parts are registered but not loaded until initialize() is called.
     */
    MovingPartsManager& add(const MovingPart& part) {
        configs_.push_back(part);
        return *this;
    }

    /**
     * @brief Initialize all moving parts
     * @return Reference for method chaining
     *
     * This method:
     * 1. Loads the body STL to determine scaling/positioning
     * 2. Loads all moving part STL meshes
     * 3. Scales and positions them to match parent geometry
     * 4. Sets rotation centers to center of mass
     * 5. Converts angular velocities to LBM units
     * 6. Performs initial voxelization
     *
     * Must be called after LBM is created and static geometry is voxelized.
     */
    MovingPartsManager& initialize() {
        if(configs_.empty()) {
            initialized_ = true;
            return *this;
        }

        // Get scaling info from SimulationSetup
        const float32_t scale = setup_.get_mesh_scale_factor();
        const float3& center_lbm = setup_.get_results().center_lbm;

        // Load body mesh to get its bounding box center for translation calculation
        const string body_path = setup_.get_stl_path();
        Mesh* body_mesh = read_stl(body_path);
        const float3 body_center_mesh = body_mesh->get_bounding_box_center();
        delete body_mesh;

        // Calculate translation: T = center_lbm - body_center_scaled
        const float3 body_center_scaled = body_center_mesh * scale;
        translation_ = center_lbm - body_center_scaled;

        // Initialize each moving part
        for(const auto& config : configs_) {
            RuntimePart rp;
            rp.config = config;

            // Load STL
            const string part_path = get_resource_path(config.get_stl_filename());
            if(part_path.empty()) {
                print_warning("MovingPart STL not found: " + config.get_stl_filename());
                continue;
            }

            rp.mesh.reset(read_stl(part_path));

            // Scale mesh
            rp.mesh->scale(scale);

            // Apply custom offset if specified
            float3 custom_offset(0.0f);
            if(config.has_custom_offset()) {
                // SI units: convert meters to cells
                custom_offset = config.get_offset_m() * scale;
            } else if(config.has_offset_ratio()) {
                // Ratio of reference size: directly multiply by LBM reference size
                const float32_t lbm_ref_size = setup_.get_results().lbm_reference_size;
                custom_offset = config.get_offset_ratio() * lbm_ref_size;
            }

            // Translate to match body positioning + custom offset
            rp.mesh->translate(translation_ + custom_offset);

            // Set rotation center to center of mass
            rp.mesh->set_center(rp.mesh->get_center_of_mass());
            rp.rotation_center = rp.mesh->get_center();

            // Calculate angular velocity in LBM units
            rp.lbm_omega = calculate_lbm_omega(config, rp.mesh.get());
            rp.lbm_omega *= config.get_direction_multiplier();

            // Calculate rotation per update interval
            rp.update_interval = config.get_update_interval();
            rp.lbm_domega = rp.lbm_omega * (float32_t)rp.update_interval;

            // Store rotation axis
            rp.rotation_axis = config.get_rotation_axis();

            // Store static and tumble mode flags
            rp.is_static = config.is_static();
            rp.use_tumble_angle = config.uses_tumble_angle();
            if(rp.use_tumble_angle) {
                rp.tumble_angle = config.get_tumble_angle();
            }

            // Initial voxelization with velocity
            const float3 angular_vel = rp.rotation_axis * rp.lbm_omega;
            lbm_.voxelize_mesh_on_device(rp.mesh.get(), TYPE_S, rp.rotation_center,
                                         float3(0.0f), angular_vel);

            parts_.push_back(std::move(rp));
        }

        initialized_ = true;
        return *this;
    }

    /**
     * @brief Update all moving parts for current timestep
     *
     * This method should be called once per simulation step.
     * It checks which parts need re-voxelization based on their interval,
     * rotates the mesh, and re-voxelizes with updated velocity.
     *
     * @par Typical usage:
     * @code
     * while(lbm.get_t() <= lbm_T) {
     *     parts.update();
     *     lbm.run(4);  // Run 4 timesteps
     * }
     * @endcode
     */
    void update() {
        if(!initialized_) return;

        const uint64_t current_t = lbm_.get_t();

        for(auto& rp : parts_) {
            // Skip static parts (voxelized once with velocity, no updates needed)
            if(rp.is_static) {
                continue;
            }

            // Check if update needed
            if(current_t - rp.last_update_t < rp.update_interval) {
                continue;
            }

            // For tumbling parts: unvoxelize first
            if(rp.config.get_motion_type() == MotionType::TUMBLING) {
                lbm_.unvoxelize_mesh_on_device(rp.mesh.get(), TYPE_S);
            }

            // Rotate mesh by accumulated angle since last update
            // Use fixed tumble angle if configured, otherwise use velocity-based rotation
            const float32_t rotation_angle = rp.use_tumble_angle ? rp.tumble_angle : rp.lbm_domega;
            const float3x3 rotation_matrix(rp.rotation_axis, rotation_angle);
            rp.mesh->rotate(rotation_matrix);

            // Voxelize with angular velocity (zero for tumbling since rotation is discrete)
            const float3 angular_vel = rp.use_tumble_angle ? float3(0.0f) : rp.rotation_axis * rp.lbm_omega;
            lbm_.voxelize_mesh_on_device(rp.mesh.get(), TYPE_S, rp.rotation_center,
                                         float3(0.0f), angular_vel);

            rp.last_update_t = current_t;
        }
    }

    /**
     * @brief Get number of registered parts
     * @return Number of parts
     */
    size_t size() const { return parts_.size(); }

    /**
     * @brief Check if manager has been initialized
     * @return True if initialize() has been called
     */
    bool is_initialized() const { return initialized_; }

    /**
     * @brief Get mesh pointer for a part (for debugging/visualization)
     * @param index Part index
     * @return Pointer to Mesh, or nullptr if index invalid
     */
    const Mesh* get_mesh(size_t index) const {
        if(index >= parts_.size()) return nullptr;
        return parts_[index].mesh.get();
    }

    /**
     * @brief Get minimum update interval across all parts
     * @return Minimum update interval in timesteps
     */
    uint32_t get_min_update_interval() const {
        uint32_t min_interval = max_uint;
        for(const auto& rp : parts_) {
            if(rp.update_interval > 0 && rp.update_interval < min_interval) {
                min_interval = rp.update_interval;
            }
        }
        return min_interval == max_uint ? 4u : min_interval;  // 4: default when no part sets an interval
    }

    /**
     * @brief Run simulation with automatic part updates
     *
     * Handles the simulation loop internally, calling update() at the
     * appropriate intervals and running the LBM simulation.
     *
     * @param total_timesteps Total simulation timesteps
     *
     * @par Example:
     * @code
     * MovingPartsManager parts(sim, lbm);
     * parts.add(MovingPart("rotor.stl").set_rotation_axis(RotationAxis::Z).set_rpm(348.0f));
     * parts.initialize();
     * parts.run(total_timesteps);
     * @endcode
     */
    void run(uint64_t total_timesteps) {
        if(!initialized_) return;

        const uint32_t interval = get_min_update_interval();
        lbm_.run(0u, total_timesteps);
        while(lbm_.get_t() < total_timesteps) {
            update();
            lbm_.run(interval, total_timesteps);
        }
    }

    /**
     * @brief Run simulation for specified time in seconds
     *
     * @param seconds Simulation time in seconds
     * @param units Units object for time conversion
     *
     * @par Example:
     * @code
     * parts.run(simulation_time_s, units);
     * @endcode
     */
    void run(float32_t seconds, const Units& units) {
        run(units.t(seconds));
    }

private:
    SimulationSetup& setup_;
    LBM& lbm_;
    std::vector<MovingPart> configs_;
    float3 translation_{0.0f};  // Translation from mesh coords to LBM coords
    bool initialized_{false};

    struct RuntimePart {
        MovingPart config{""};
        std::unique_ptr<Mesh> mesh;
        float32_t lbm_omega{0.0f};      // Angular velocity in LBM units
        float32_t lbm_domega{0.0f};     // Rotation per update interval
        float3 rotation_center{0.0f};
        float3 rotation_axis{0.0f, 1.0f, 0.0f};
        uint32_t update_interval{4};
        uint64_t last_update_t{0};
        bool is_static{false};          // Static parts skip update()
        bool use_tumble_angle{false};   // Use fixed angle instead of velocity-based
        float32_t tumble_angle{0.0f};   // Fixed rotation angle per update
    };

    std::vector<RuntimePart> parts_;

    /**
     * @brief Calculate LBM angular velocity from part configuration
     * @param config MovingPart configuration
     * @param mesh Loaded mesh (for radius calculation)
     * @return Angular velocity in LBM units
     */
    float32_t calculate_lbm_omega(const MovingPart& config, const Mesh* mesh) {
        const float32_t scale = setup_.get_mesh_scale_factor();
        const float32_t radius_lbm = 0.5f * mesh->get_max_size();

        // Handle LBM-unit tip speed directly (for Re-based simulations)
        if(config.get_velocity_mode() == MovingPart::VelocityMode::TIP_SPEED
           && config.is_tip_speed_in_lbm_units()) {
            // omega_lbm = tip_speed_lbm / radius_lbm
            return config.get_tip_speed_lbm() / radius_lbm;
        }

        float32_t si_omega = 0.0f;

        switch(config.get_velocity_mode()) {
            case MovingPart::VelocityMode::TIP_SPEED: {
                // omega = tip_speed / radius
                // radius in SI = (mesh_max_size / scale) / 2
                const float32_t radius_si = radius_lbm / scale;
                si_omega = config.get_tip_speed_mps() / radius_si;
                break;
            }
            case MovingPart::VelocityMode::RPM: {
                // omega = RPM * 2*pi / 60
                si_omega = config.get_rpm() * 2.0f * pif / 60.0f;
                break;
            }
            case MovingPart::VelocityMode::DIRECT: {
                si_omega = config.get_angular_velocity_radps();
                break;
            }
            case MovingPart::VelocityMode::ROLLING: {
                // omega = velocity / radius (use min size for wheel radius)
                const float32_t wheel_radius_lbm = 0.5f * mesh->get_min_size();
                const float32_t wheel_radius_si = wheel_radius_lbm / scale;
                si_omega = config.get_rolling_velocity_mps() / wheel_radius_si;
                break;
            }
        }

        // Convert SI omega (rad/s) to LBM units using global units object
        // In LBM, omega has units of 1/timestep
        // omega_lbm = omega_si * dt_si = omega_si / (lbm_u_ref / si_u_ref * si_length / lbm_length)
        // This simplifies to: omega_lbm = omega_si * si_length / (lbm_length * si_u_ref / lbm_u_ref)
        // Using units.t() for time conversion: omega_lbm = omega_si / units.omega_factor

        // For simplicity, use: omega_lbm = omega_si * (reference_size_si / reference_size_lbm) / (si_u / lbm_u)
        // But we don't have direct access to si_u. Use the scale factor relationship:
        // omega_lbm = omega_si * dt_lbm where dt_lbm is the time per LBM step in SI units

        // The simplest approach: omega in LBM is dimensionless (radians per timestep)
        // If lbm_u corresponds to si_u, then: omega_lbm = omega_si * (si_reference_size / lbm_reference_size) * (lbm_u / si_u)
        // Since scale = lbm_reference_size / si_reference_size, and lbm_u/si_u = scale * nu_ratio...
        // This gets complicated. Let's use the direct relationship:
        // omega_lbm * radius_lbm = tangential_velocity_lbm
        // tangential_velocity_lbm = units.u(tangential_velocity_si)
        // tangential_velocity_si = omega_si * radius_si
        // So: omega_lbm = units.u(omega_si * radius_si) / radius_lbm
        //              = units.u(omega_si * radius_si) / radius_lbm

        // Actually, since radius_lbm = radius_si * scale:
        // omega_lbm = units.u(omega_si * radius_si) / (radius_si * scale)

        // But units.u(v_si) = v_si * (lbm_u_ref / si_u_ref)
        // So: omega_lbm = omega_si * radius_si * (lbm_u_ref / si_u_ref) / (radius_si * scale)
        //              = omega_si * (lbm_u_ref / si_u_ref) / scale
        //              = omega_si * lbm_u_ref * si_reference_size / (si_u_ref * lbm_reference_size)

        // Simplify: omega_lbm = omega_si * (si_reference_size / si_u_ref) * (lbm_u_ref / lbm_reference_size)

        // Using existing units object:
        // units.t(1.0) gives LBM timesteps per SI second
        // omega_lbm = omega_si / units.t(1.0) = omega_si * (time_si / time_lbm)
        // But that's inverted... let me check units.hpp

        // For now, use the direct calculation based on tangential velocity
        // We know: tangential_vel_lbm = setup_.to_lbm_velocity(tangential_vel_si)
        // And: tangential_vel_si = omega_si * radius_si
        // So: omega_lbm = tangential_vel_lbm / radius_lbm

        const float32_t max_radius_lbm = 0.5f * mesh->get_max_size();
        const float32_t max_radius_si = max_radius_lbm / scale;
        const float32_t tangential_vel_si = si_omega * max_radius_si;
        const float32_t tangential_vel_lbm = setup_.to_lbm_velocity(tangential_vel_si);
        const float32_t lbm_omega = tangential_vel_lbm / max_radius_lbm;

        return lbm_omega;
    }
};
