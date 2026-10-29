#pragma once

#include "setup/core/types.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/domain_plan.hpp"
#include "setup/domain/model_placement.hpp"
#include "setup/moving/moving_part.hpp"
#include "setup/simulation/runner.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"
#include "lbm.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

/// @file moving_parts_manager.hpp
/// @brief The moving parts of a simulation, voxelized and turned while it runs (MovingPartsManager).

/// @brief Holds the moving parts of the simulation (Simulation::parts()): loads them with the transform of its model,
/// voxelizes them with their angular velocity, and turns and re-voxelizes them while it runs.
///
/// Owned by Simulation, which makes it with the LBM; it keeps references to the LBM, the plan and the runner. Call
/// initialize() after the boundaries are set: it writes the parts' cells (TYPE_S, with their wall velocity) on the
/// device and schedules the turns with the simulation's runner.
/// @code
/// using namespace si::symbols;
/// sim.parts().add(MovingPart("rotor.stl").set_rotation_axis(Axis::Y).set_tip_speed(100.0 * m / s)).initialize();
/// @endcode
class MovingPartsManager {
public:
    /// @brief Makes the manager of an LBM.
    /// @param lbm the LBM in which the parts are voxelized
    /// @param converter the simulation's conversions, for the lengths, speeds, rates and times
    /// @param plan the simulation's plan, for its model's transform (see MovingPart::centered_on_model())
    /// @param runner the simulation's runner, which turns the parts
    MovingPartsManager(LBM& lbm, const si::LatticeConverter& converter, const DomainPlan& plan, Runner& runner)
        : lbm_(lbm), converter_(converter), plan_(plan), runner_(runner) {}

    /// @brief Adds a part.
    /// @param part the part's file and motion
    /// @return this manager
    MovingPartsManager& add(const MovingPart& part) {
        configs_.push_back(part);
        return *this;
    }

    /// @brief Loads the parts with the model's transform, voxelizes them with their angular velocity and schedules
    /// their turns, each at its update interval.
    ///
    /// Ends the program with print_error() on a second call, when the simulation has no model (ModelPlacement), or
    /// when a part's STL is not found in resources/ (a run without the part would be a different machine).
    /// @return this manager
    MovingPartsManager& initialize() {
        if(initialized_) {
            print_error("MovingPartsManager: initialize() was already called");
        }
        initialized_ = true;
        if(configs_.empty()) {
            return *this;
        }
        // Before the LBM is initialized, voxelize_mesh_on_device() reads the flags and velocities back from the device,
        // which would drop what the host set since (BoundaryBuilder's faces, the initial velocity): send them first.
        // At t = 0 the host is up to date also after run(0), which has just copied it to the device.
        if(lbm_.get_t() == 0ull) {
            lbm_.flags.write_to_device();
            lbm_.u.write_to_device();
        }
        const ModelPlacement placement{ model_placement() };
        for(const MovingPart& config : configs_) {
            add_runtime_part(config, placement);
        }
        return *this;
    }

private:
    /// A part while the simulation runs.
    struct RuntimePart {
        MovingPart config{ "" };                       ///< the part as added
        std::unique_ptr<Mesh> mesh{};                  ///< its mesh, placed and turned
        float32_t lbm_omega{ 0.0f };                   ///< its angular velocity in radians per time step
        float3 rotation_center{ 0.0f };                ///< its centre of mass in cells
        float3 rotation_axis{ 0.0f, 1.0f, 0.0f };      ///< the unit vector of its axis
        bool tumbling{ false };                        ///< true when it tumbles (turned in steps) instead of rotating
        si::Duration update_interval{ au::ZERO };      ///< the simulated time between its turns
        uint64_t last_update_t{ 0u };                  ///< the time step of its last turn
    };

    LBM& lbm_;                                          ///< the LBM in which the parts are voxelized (not owned)
    si::LatticeConverter converter_;                    ///< the simulation's conversions
    const DomainPlan& plan_;                            ///< the simulation's plan (not owned)
    Runner& runner_;                                    ///< the simulation's runner (not owned)
    std::vector<MovingPart> configs_{};                 ///< the parts added
    bool initialized_{ false };                         ///< true after initialize()
    std::vector<std::unique_ptr<RuntimePart>> parts_{}; ///< stable addresses: the runner's tasks refer to them

    /// @brief Gives the transform of the simulation's model; ends the program with print_error() when it has none.
    /// @return the transform
    ModelPlacement model_placement() const {
        std::optional<ModelPlacement> placement{};
        try {
            placement.emplace(ModelPlacement::of(plan_));
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        const ModelPlacement checked{ placement.value() }; // print_error() exits, so a placement is here
        return checked;
    }

    /// @brief Loads one part, voxelizes it and schedules its turns. Ends the program with print_error() when the
    /// part's STL is not found in resources/.
    /// @param config the part's file and motion
    /// @param placement the model's transform
    void add_runtime_part(const MovingPart& config, const ModelPlacement& placement) {
        const string part_path{ get_resource_path(config.get_stl_filename()) };
        if(part_path.empty()) {
            print_error("MovingPartsManager: the moving part's STL " + config.get_stl_filename() + " is not in resources/");
        }
        std::unique_ptr<RuntimePart> rp{ std::make_unique<RuntimePart>() };
        rp->config = config;
        rp->mesh = load_mesh(config, placement, part_path);
        rp->mesh->set_center(rp->mesh->get_center_of_mass()); // rotate around the centre of mass
        rp->rotation_center = rp->mesh->get_center();
        rp->rotation_axis = config.get_rotation_axis();
        rp->tumbling = config.get_motion_type() == MotionType::TUMBLING;
        rp->lbm_omega = lbm_angular_velocity(config, rp->mesh.get()) * config.get_direction_multiplier();
        rp->update_interval = update_interval(config, rp->lbm_omega, rp->mesh.get());
        const float3 wall_velocity{ angular_velocity(*rp) };
        lbm_.voxelize_mesh_on_device(rp->mesh.get(), TYPE_S, rp->rotation_center, float3(0.0f), wall_velocity);
        if(rp->lbm_omega != 0.0f) {
            RuntimePart& part{ *rp };
            runner_.every(part.update_interval, [this, &part](si::Duration) { turn(part); });
        }
        parts_.push_back(std::move(rp));
    }

    /// @brief Loads a part's mesh: with the model's transform, or centred on the model plus an offset.
    /// @param config the part's file and motion
    /// @param placement the model's transform
    /// @param part_path the part's STL file, resolved
    /// @return the mesh
    std::unique_ptr<Mesh> load_mesh(const MovingPart& config, const ModelPlacement& placement, const string& part_path) const {
        const std::optional<si::Position>& offset{ config.get_centered_offset() };
        if(!offset.has_value()) {
            std::unique_ptr<Mesh> mesh{ placement.load(part_path) };
            return mesh;
        }
        const si::Position& offset_m{ offset.value() };
        const float x{ static_cast<float>(converter_.cells(offset_m.x)) };
        const float y{ static_cast<float>(converter_.cells(offset_m.y)) };
        const float z{ static_cast<float>(converter_.cells(offset_m.z)) };
        const float3 offset_lbm{ x, y, z };
        std::unique_ptr<Mesh> mesh{ placement.load_centered(part_path, offset_lbm) };
        return mesh;
    }

    /// @brief Gives the angular velocity on the part's cells; tumbling parts turn in steps and have none.
    /// @param rp the part while the simulation runs
    /// @return the axis times the angular velocity in radians per time step, or 0 for a tumbling part
    static float3 angular_velocity(const RuntimePart& rp) {
        if(rp.tumbling) {
            const float3 none{ 0.0f };
            return none;
        }
        const float3 omega{ rp.rotation_axis * rp.lbm_omega };
        return omega;
    }

    /// @brief Gives a part's angular velocity in radians per time step: a tumbling part's rate, or the tip speed at
    /// half the part's largest dimension.
    /// @param config the part's file and motion
    /// @param mesh its mesh, placed
    /// @return the angular velocity in radians per time step
    float32_t lbm_angular_velocity(const MovingPart& config, const Mesh* mesh) const {
        if(config.get_motion_type() == MotionType::TUMBLING) {
            const float32_t rate{ converter_.angular_rate(config.get_tumble_rate()) };
            return rate;
        }
        const float32_t radius_lbm{ 0.5f * mesh->get_max_size() };
        const float32_t tip_speed_lbm{ converter_.velocity(config.get_tip_speed()) };
        const float32_t omega{ tip_speed_lbm / radius_lbm };
        return omega;
    }

    /// @brief Gives a part's update interval: the one set, or by default the time its tip takes to move half a cell
    /// (at least one time step).
    /// @param config the part's file and motion
    /// @param lbm_omega its angular velocity in radians per time step
    /// @param mesh its mesh, placed
    /// @return the simulated time between the part's turns
    si::Duration update_interval(const MovingPart& config, float32_t lbm_omega, const Mesh* mesh) const {
        const std::optional<si::Duration>& interval{ config.get_update_interval() };
        if(interval.has_value()) {
            const si::Duration set_interval{ interval.value() };
            return set_interval;
        }
        const float32_t tip_speed_lbm{ std::fabs(lbm_omega) * 0.5f * mesh->get_max_size() };
        float32_t steps{ 1.0f };
        if(tip_speed_lbm > 0.0f) {
            const float32_t half_cell_steps{ std::floor(0.5f / tip_speed_lbm) };
            steps = std::max(1.0f, half_cell_steps);
        }
        const si::Duration time{ converter_.time(static_cast<uint64_t>(steps)) };
        return time;
    }

    /// @brief Turns a part by the angle since its last update and re-voxelizes it.
    /// @param rp the part while the simulation runs
    void turn(RuntimePart& rp) {
        const uint64_t t{ lbm_.get_t() };
        if(t == rp.last_update_t) {
            return; // no time has passed (the start)
        }
        if(rp.tumbling) {
            lbm_.unvoxelize_mesh_on_device(rp.mesh.get(), TYPE_S);
        }
        const float32_t elapsed{ static_cast<float32_t>(t - rp.last_update_t) };
        const float32_t angle{ rp.lbm_omega * elapsed };
        rp.mesh->rotate(float3x3(rp.rotation_axis, angle));
        const float3 wall_velocity{ angular_velocity(rp) };
        lbm_.voxelize_mesh_on_device(rp.mesh.get(), TYPE_S, rp.rotation_center, float3(0.0f), wall_velocity);
        rp.last_update_t = t;
    }
};
