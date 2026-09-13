#pragma once

#include "setup/core/types.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/core/unit_scale.hpp"
#include "setup/domain/domain_plan.hpp"
#include "setup/domain/model_placement.hpp"
#include "setup/moving/moving_part.hpp"
#include "setup/simulation/runner.hpp"
#include "lbm.hpp"
#include <algorithm>
#include <memory>
#include <optional>
#include <vector>

/// @brief The moving parts of the simulation (Simulation::parts()): loaded with the transform of its model, voxelized
/// with their angular velocity, and turned and re-voxelized while it runs.
///
/// Call initialize() after the boundaries are set; it schedules the turns with the simulation's runner.
/// @code
/// sim.parts().add(MovingPart("rotor.stl").set_rotation_axis(Axis::Y).set_tip_speed(100.0_mps)).initialize();
/// @endcode
class MovingPartsManager {
public:
    /// @param lbm    the LBM the parts are voxelized in
    /// @param unit_scale  the simulation's unit scale
    /// @param plan   the simulation's plan, for its model's transform (see MovingPart::centered_on_model())
    /// @param runner the simulation's runner, which turns the parts
    MovingPartsManager(LBM& lbm, const UnitScale& unit_scale, const DomainPlan& plan, Runner& runner)
        : lbm_(lbm), units_(unit_scale), plan_(plan), runner_(runner) {}

    MovingPartsManager& add(const MovingPart& part) {
        configs_.push_back(part);
        return *this;
    }

    /// Loads the parts with the model's transform, voxelizes them with their angular velocity and schedules their
    /// turns, each at its update interval.
    MovingPartsManager& initialize() {
        if(initialized_) print_error("MovingPartsManager: initialize() was already called");
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

            auto rp = std::make_unique<RuntimePart>();
            rp->config = config;
            if(const auto& offset = config.get_centered_offset()) {
                const float3 offset_lbm(units_.length(offset->x), units_.length(offset->y), units_.length(offset->z));
                rp->mesh = placement.load_centered(part_path, offset_lbm);
            } else {
                rp->mesh = placement.load(part_path);
            }

            rp->mesh->set_center(rp->mesh->get_center_of_mass()); // rotate around the center of mass
            rp->rotation_center = rp->mesh->get_center();
            rp->rotation_axis = config.get_rotation_axis();
            rp->tumbling = config.get_motion_type() == MotionType::TUMBLING;
            rp->lbm_omega = lbm_angular_velocity(config, rp->mesh.get()) * config.get_direction_multiplier();
            rp->update_interval = update_interval(config, rp->lbm_omega, rp->mesh.get());

            lbm_.voxelize_mesh_on_device(rp->mesh.get(), TYPE_S, rp->rotation_center, float3(0.0f), angular_velocity(*rp));

            if(rp->lbm_omega != 0.0f) {
                RuntimePart& part = *rp;
                runner_.every(part.update_interval, [this, &part](Duration) { turn(part); });
            }
            parts_.push_back(std::move(rp));
        }
        return *this;
    }

private:
    LBM& lbm_;
    UnitScale units_;
    const DomainPlan& plan_;
    Runner& runner_;
    std::vector<MovingPart> configs_;
    bool initialized_{false};

    // the transform of the simulation's model (exits with a message if it has none)
    ModelPlacement model_placement() const {
        std::optional<ModelPlacement> placement;
        try {
            placement.emplace(ModelPlacement::of(plan_));
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        return *placement;
    }

    struct RuntimePart {
        MovingPart config{""};
        std::unique_ptr<Mesh> mesh;
        float32_t lbm_omega{0.0f};      // angular velocity in rad per time step
        float3 rotation_center{0.0f};
        float3 rotation_axis{0.0f, 1.0f, 0.0f};
        bool tumbling{false};
        Duration update_interval{};
        uint64_t last_update_t{0};
    };

    std::vector<std::unique_ptr<RuntimePart>> parts_; ///< stable addresses: the runner's tasks refer to them

    // the angular velocity on the part's cells: tumbling parts turn in steps and have none
    static float3 angular_velocity(const RuntimePart& rp) {
        return rp.tumbling ? float3(0.0f) : rp.rotation_axis * rp.lbm_omega;
    }

    // rad per time step: a tumbling part's rate, or the tip speed at half the part's largest dimension
    float32_t lbm_angular_velocity(const MovingPart& config, const Mesh* mesh) const {
        if(config.get_motion_type() == MotionType::TUMBLING) {
            return config.get_tumble_rate().rad_per_s() * units_.time_step().si();
        }
        const float32_t radius_lbm = 0.5f * mesh->get_max_size();
        return units_.velocity(config.get_tip_speed()) / radius_lbm;
    }

    // the part's update interval, by default the time its tip takes to move half a cell (at least one time step)
    Duration update_interval(const MovingPart& config, float32_t lbm_omega, const Mesh* mesh) const {
        if(const auto& interval = config.get_update_interval()) return *interval;
        const float32_t tip_speed_lbm = fabs(lbm_omega) * 0.5f * mesh->get_max_size();
        const float32_t steps = tip_speed_lbm > 0.0f ? std::max(1.0f, floorf(0.5f / tip_speed_lbm)) : 1.0f;
        return units_.si_time((uint64_t)steps);
    }

    // turns the part by the angle since its last update and re-voxelizes it
    void turn(RuntimePart& rp) {
        const uint64_t t = lbm_.get_t();
        if(t == rp.last_update_t) return; // no time has passed (the start)
        if(rp.tumbling) lbm_.unvoxelize_mesh_on_device(rp.mesh.get(), TYPE_S);
        rp.mesh->rotate(float3x3(rp.rotation_axis, rp.lbm_omega * (float32_t)(t - rp.last_update_t)));
        lbm_.voxelize_mesh_on_device(rp.mesh.get(), TYPE_S, rp.rotation_center, float3(0.0f), angular_velocity(rp));
        rp.last_update_t = t;
    }
};
