#pragma once

#include "core/types.hpp"
#include "lbm.hpp"
#include "moving/moving_part.hpp"
#include "simulation/simulation_setup.hpp"
#include <vector>
#include <memory>

// Loads moving parts scaled and placed like the SimulationSetup geometry, and re-voxelizes them as they rotate.
// Call initialize() after the static geometry is voxelized, then update() in the run loop (or use run()).
class MovingPartsManager {
public:
    MovingPartsManager(SimulationSetup& setup, LBM& lbm)
        : setup_(setup), lbm_(lbm) {}

    MovingPartsManager& add(const MovingPart& part) {
        configs_.push_back(part);
        return *this;
    }

    // load, scale and place all parts, and voxelize them with their angular velocity
    MovingPartsManager& initialize() {
        if(configs_.empty()) {
            initialized_ = true;
            return *this;
        }

        const float32_t scale = setup_.get_mesh_scale_factor();
        const float3& center_lbm = setup_.get_results().center_lbm;

        // translation from mesh coordinates to cells: the body's bounding box center goes to center_lbm
        const string body_path = setup_.get_stl_path();
        Mesh* body_mesh = read_stl(body_path);
        const float3 body_center_mesh = body_mesh->get_bounding_box_center();
        delete body_mesh;
        translation_ = center_lbm - body_center_mesh * scale;

        for(const auto& config : configs_) {
            RuntimePart rp;
            rp.config = config;

            const string part_path = get_resource_path(config.get_stl_filename());
            if(part_path.empty()) {
                print_warning("MovingPart STL not found: " + config.get_stl_filename());
                continue;
            }

            rp.mesh.reset(read_stl(part_path));
            rp.mesh->scale(scale);

            float3 custom_offset(0.0f);
            if(config.has_offset_ratio()) {
                custom_offset = config.get_offset_ratio() * setup_.get_results().lbm_reference_size;
            }
            rp.mesh->translate(translation_ + custom_offset);

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

        initialized_ = true;
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

    void run(float32_t seconds, const Units& units) {
        run(units.t(seconds));
    }

private:
    SimulationSetup& setup_;
    LBM& lbm_;
    std::vector<MovingPart> configs_;
    float3 translation_{0.0f}; // mesh coordinates to cells
    bool initialized_{false};

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
        return setup_.to_lbm_velocity(config.get_tip_speed_mps()) / radius_lbm;
    }
};
