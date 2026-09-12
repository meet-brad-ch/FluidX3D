#pragma once

#include "setup/core/types.hpp"
#include "lbm.hpp"
#include "setup/moving/moving_part.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/model_placement.hpp"
#include "setup/simulation/simulation_setup.hpp"
#include <vector>
#include <memory>
#include <optional>

// Loads moving parts with the transform of the SimulationSetup model, and re-voxelizes them as they rotate.
// Call initialize() after the static geometry is voxelized, then update() in the run loop (or use run()).
class MovingPartsManager {
public:
    MovingPartsManager(SimulationSetup& setup, LBM& lbm)
        : setup_(setup), lbm_(lbm) {}

    MovingPartsManager& add(const MovingPart& part) {
        configs_.push_back(part);
        return *this;
    }

    /// Loads the parts with the model's transform (see MovingPart::centered_on_model()) and voxelizes them with their
    /// angular velocity; call after configure_units().
    MovingPartsManager& initialize() {
        initialized_ = true;
        if(configs_.empty()) return *this;

        // Before the LBM is initialized, voxelize_mesh_on_device() reads the flags and velocities back from the device,
        // which would drop what the host set since (BoundaryBuilder's faces, the initial velocity): send them first.
        // At t = 0 the host is up to date also after run(0), which has just copied it to the device.
        if(lbm_.get_t() == 0ull) {
            lbm_.flags.write_to_device();
            lbm_.u.write_to_device();
        }

        const ModelPlacement placement = model_placement();
        for(const auto& config : configs_) {
            const string part_path = get_resource_path(config.get_stl_filename());
            if(part_path.empty()) {
                print_warning("MovingPart STL not found: " + config.get_stl_filename());
                continue;
            }

            RuntimePart rp;
            rp.config = config;
            if(const auto& offset = config.get_centered_offset()) {
                const float3 offset_lbm(setup_.to_lbm_length(offset->x), setup_.to_lbm_length(offset->y), setup_.to_lbm_length(offset->z));
                rp.mesh = placement.load_centered(part_path, offset_lbm);
            } else {
                rp.mesh = placement.load(part_path);
            }

            rp.mesh->set_center(rp.mesh->get_center_of_mass()); // rotate around the center of mass
            rp.rotation_center = rp.mesh->get_center();

            rp.lbm_omega = calculate_lbm_omega(config, rp.mesh.get()) * config.get_direction_multiplier();
            rp.update_interval = config.get_update_interval();
            rp.lbm_domega = rp.lbm_omega * (float32_t)rp.update_interval;
            rp.rotation_axis = config.get_rotation_axis();
            rp.use_tumble_angle = config.uses_tumble_angle();
            if(rp.use_tumble_angle) {
                rp.tumble_angle = config.get_tumble_angle();
            }

            const float3 angular_vel = rp.rotation_axis * rp.lbm_omega;
            lbm_.voxelize_mesh_on_device(rp.mesh.get(), TYPE_S, rp.rotation_center,
                                         float3(0.0f), angular_vel);

            parts_.push_back(std::move(rp));
        }
        return *this;
    }

    // rotate and re-voxelize every part whose update interval has passed
    void update() {
        if(!initialized_) return;

        const uint64_t current_t = lbm_.get_t();

        for(auto& rp : parts_) {
            if(current_t - rp.last_update_t < rp.update_interval) {
                continue;
            }

            if(rp.config.get_motion_type() == MotionType::TUMBLING) {
                lbm_.unvoxelize_mesh_on_device(rp.mesh.get(), TYPE_S);
            }

            const float32_t rotation_angle = rp.use_tumble_angle ? rp.tumble_angle : rp.lbm_domega;
            const float3x3 rotation_matrix(rp.rotation_axis, rotation_angle);
            rp.mesh->rotate(rotation_matrix);

            // tumbling moves in discrete steps: no angular velocity on the cells
            const float3 angular_vel = rp.use_tumble_angle ? float3(0.0f) : rp.rotation_axis * rp.lbm_omega;
            lbm_.voxelize_mesh_on_device(rp.mesh.get(), TYPE_S, rp.rotation_center,
                                         float3(0.0f), angular_vel);

            rp.last_update_t = current_t;
        }
    }

    uint32_t get_min_update_interval() const {
        uint32_t min_interval = max_uint;
        for(const auto& rp : parts_) {
            if(rp.update_interval > 0 && rp.update_interval < min_interval) {
                min_interval = rp.update_interval;
            }
        }
        return min_interval == max_uint ? 4u : min_interval; // 4: default when no part sets an interval
    }

    // run total_timesteps, updating the parts every get_min_update_interval() steps
    void run(uint64_t total_timesteps) {
        if(!initialized_) return;

        const uint32_t interval = get_min_update_interval();
        lbm_.run(0u, total_timesteps);
        while(lbm_.get_t() < total_timesteps) {
            update();
            lbm_.run(interval, total_timesteps);
        }
    }

    /// Runs this long, updating the parts every get_min_update_interval() steps.
    void run(Duration time) {
        run(setup_.to_lbm_timesteps(time));
    }

private:
    SimulationSetup& setup_;
    LBM& lbm_;
    std::vector<MovingPart> configs_;
    bool initialized_{false};

    // the transform of the setup's model (exits with a message if the setup has none)
    ModelPlacement model_placement() const {
        std::optional<ModelPlacement> placement;
        try {
            placement.emplace(ModelPlacement::of(setup_.get_results()));
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        return *placement;
    }

    struct RuntimePart {
        MovingPart config{""};
        std::unique_ptr<Mesh> mesh;
        float32_t lbm_omega{0.0f};      // angular velocity in rad per time step
        float32_t lbm_domega{0.0f};     // rotation per update interval
        float3 rotation_center{0.0f};
        float3 rotation_axis{0.0f, 1.0f, 0.0f};
        uint32_t update_interval{4};
        uint64_t last_update_t{0};
        bool use_tumble_angle{false};
        float32_t tumble_angle{0.0f};   // rotation per update when tumbling
    };

    std::vector<RuntimePart> parts_;

    // angular velocity in rad per time step: the tip speed at half the part's largest dimension
    float32_t calculate_lbm_omega(const MovingPart& config, const Mesh* mesh) const {
        const float32_t radius_lbm = 0.5f * mesh->get_max_size();
        return setup_.to_lbm_velocity(config.get_tip_speed()) / radius_lbm;
    }
};
