#pragma once

#include "setup/core/types.hpp"
#include "setup/core/fluids.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/core/transport_build.hpp"
#include "setup/core/time_step_rule.hpp"
#include "setup/core/diffusivity.hpp"
#include "setup/core/boussinesq_buoyancy.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/boundaries/boundary_builder.hpp"
#include "setup/domain/domain.hpp"
#include "setup/domain/domain_plan.hpp"
#include "setup/domain/lattice.hpp"
#include "setup/domain/run_estimate.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/lattice_mach.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include "setup/graphics/graphics_config.hpp"
#include "setup/moving/moving_parts_manager.hpp"
#include "setup/simulation/field_reader.hpp"
#include "setup/simulation/mesh_loader.hpp"
#include "setup/simulation/runner.hpp"
#include "lbm.hpp"
#include "units.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#ifdef SURFACE
#include "setup/surface/surface_builder.hpp"
#include "setup/surface/wave_boundary.hpp"
#endif // SURFACE
#ifdef TEMPERATURE
#include "setup/boundaries/thermal_builder.hpp"
#endif // TEMPERATURE
#ifdef GRAPHICS
#include "setup/graphics/video_recorder.hpp"
#endif // GRAPHICS
#ifdef FORCE_FIELD
#include "setup/analysis/force_analyzer.hpp"
#endif // FORCE_FIELD
#ifdef PARTICLES
#include "setup/particles/particle_manager.hpp"
#endif // PARTICLES

/// @file simulation.hpp
/// @brief A simulation in physical units (Simulation): it owns the core's LBM and the components that act on it.

/// @brief A simulation in physical units.
///
/// The domain, the fluid and a reference speed (usually the fastest in the flow) or a time step rule set the
/// lattice: the grid, the cell size and the time step. They make the simulation's one converter (converter()), which
/// makes every lattice value the core gets. The Reynolds and lattice Mach numbers are printed. The core's LBM is
/// created on the first use (a builder, the graphics, a run) with the physics set before it: gravity or a body
/// force, the surface tension, the temperatures, the particles. A model in the domain is voxelized then. The builders
/// write to the grid; the owned components (video(), parts(), forces(), particles(), wave_maker()) keep their state
/// while it runs; run_for() or run() drive the simulation, with tasks every interval of simulated time.
/// @code
/// using namespace si::symbols;
/// Simulation sim(Domain::around(Model("Cow_t.stl").length(2.4 * m)).size(1.85 * m, 3.69 * m, 1.85 * m).on_floor().vram(1000.0 * MiB),
///                Fluid::AIR, 1.0 * m / s);
/// sim.boundaries().set_solid_floor().set_open_boundaries().initialize_velocity_y(1.0 * m / s).apply();
/// sim.graphics().show_surface().show_vortices().apply();
/// sim.video().add(CameraView::orbit(-40.0 * deg, 20.0 * deg)).set_length(10.0 * s); // written when built for video
/// sim.run_for(10.0 * s);
/// @endcode
class Simulation {
public:
    /// @brief Makes a simulation whose time step puts a reference speed at a lattice Mach number.
    ///
    /// Ends the program with print_error() when the domain's settings conflict, its geometry file is not found, the
    /// domain cannot be planned, or the build's per-level forcing cannot run on the planned split.
    /// @param domain the domain in physical units
    /// @param fluid the fluid: its density becomes the lattice density 1, its viscosity the lattice viscosity
    /// @param reference_speed the reference speed, usually the fastest in the flow, more than 0
    /// @param mach how compressible the simulation makes the flow; it sets the time step (see si::LatticeMach)
    Simulation(const Domain& domain, const FluidProperties& fluid, si::Speed reference_speed, si::LatticeMach mach = si::LatticeMach())
        : domain_(validated(domain)), fluid_(fluid), reference_speed_(reference_speed), plan_(prepared_plan(domain_)),
          converter_(mach_converter(plan_, reference_speed, mach, fluid.density)) {
        set_core_units();
        const float mach_number{ static_cast<float>(mach.value()) };
        const float compressibility_percent{ 100.0f * sq(mach_number) };
        print_info("Lattice Mach number " + to_string(mach_number, 4u) + " (compressibility error ~" + to_string(compressibility_percent, 2u) + " %)");
        const double reynolds{ reynolds_number() };
        const uint32_t reynolds_whole{ static_cast<uint32_t>(reynolds) };
        print_info("Re = " + to_string(reynolds_whole));
    }

    /// @brief Makes a simulation whose time step comes from a rule at the domain's cell size, for flows without one
    /// fastest speed (an atmosphere: TimeStepRule::constant_lattice_speed keeps dx/dt as the cell changes).
    ///
    /// Ends the program with print_error() when the domain has no Domain::cell_size(), its settings conflict, its
    /// geometry file is not found, it cannot be planned, the cell is not a float exactly, or the build's per-level
    /// forcing cannot run on the planned split.
    /// @param domain the domain, with Domain::cell_size()
    /// @param fluid the fluid: its density becomes the lattice density 1, its viscosity the lattice viscosity (unless
    /// set_viscosity())
    /// @param rule the time step at the cell size
    Simulation(const Domain& domain, const FluidProperties& fluid, const TimeStepRule& rule)
        : domain_(validated(domain)), fluid_(fluid), reference_speed_(rule_speed(rule, domain_)), plan_(prepared_plan(domain_)),
          converter_(rule_converter(domain_, rule, fluid.density)) {
        set_core_units();
        const float step_s{ static_cast<float>(converter_.step().in(au::seconds)) };
        const float cell_m{ static_cast<float>(converter_.cell().in(au::meters)) };
        const std::string step_text{ to_string(step_s, 5u) };
        const std::string cell_text{ to_string(cell_m, 3u) };
        const std::string rule_text{ rule.describe() };
        print_info("Time step " + step_text + " s at " + cell_text + " m cells (" + rule_text + ")");
    }

    Simulation(const Simulation&) = delete;            ///< not copyable: it owns the LBM
    Simulation& operator=(const Simulation&) = delete; ///< not copyable: it owns the LBM

    /// @name Physics, before the first use
    /// @{

    /// @brief Sets gravity along the negative direction of an axis (VOLUME_FORCE). Before the first use only.
    /// @param gravity the gravitational acceleration, for example 9.81 m/s²
    /// @param axis the axis along which gravity points down, Z by default
    /// @return this simulation
    Simulation& set_gravity(si::Acceleration gravity, Axis axis = Axis::Z) {
        before_first_use("set_gravity()");
        gravity_axis_ = axis;
        const si::Acceleration none{ au::ZERO };
        const si::Acceleration down_x{ axis == Axis::X ? -gravity : none };
        const si::Acceleration down_y{ axis == Axis::Y ? -gravity : none };
        const si::Acceleration down_z{ axis == Axis::Z ? -gravity : none };
        const si::AccelerationVector force{ down_x, down_y, down_z };
        Simulation& self{ set_body_force(force) };
        return self;
    }

    /// @brief Sets a body force per mass in every cell (VOLUME_FORCE), such as gravity on a slope or a pressure
    /// gradient over the density. It can change while the simulation runs (a turning gravity).
    ///
    /// Ends the program with print_error() when set_boussinesq_buoyancy() is set.
    /// @param force the force per mass
    /// @return this simulation
    Simulation& set_body_force(const si::AccelerationVector& force) {
        if(buoyancy_) {
            print_error("Simulation: set_gravity() and set_body_force() are excluded by set_boussinesq_buoyancy(), which is set");
        }
        body_force_ = force;
        if(lbm_) {
            const float3 f{ lattice(force) };
            lbm_->set_f(f.x, f.y, f.z);
        }
        return *this;
    }

    /// @brief Sets the surface tension of a free surface (SURFACE); 0 is none. Before the first use only.
    /// @param surface_tension the surface tension, 0 or more
    /// @return this simulation
    Simulation& set_surface_tension(si::SurfaceTension surface_tension) {
        before_first_use("set_surface_tension()");
        surface_tension_ = surface_tension;
        return *this;
    }

    /// @brief Sets the temperature range of a thermal simulation (TEMPERATURE): cold and hot become the lattice
    /// temperatures 0.5 and 1.5, and the fluid's thermal expansion drives the buoyancy. Before the first use only.
    ///
    /// Ends the program with print_error() when hot is not above cold.
    /// @param cold the cold temperature
    /// @param hot the hot temperature, above the cold one
    /// @return this simulation
    Simulation& set_temperatures(si::Temperature cold, si::Temperature hot) {
        before_first_use("set_temperatures()");
        if(!(hot > cold)) {
            print_error("Simulation::set_temperatures(): the hot temperature must be above the cold one");
        }
        const si::TemperatureDifference unit{ hot - cold };
        const si::Temperature reference{ cold + unit * 0.5 };
        set_temperature_scale(reference, unit);
        return *this;
    }

    /// @brief Sets the temperature scale of a thermal simulation (TEMPERATURE) directly: the lattice temperature 1 at
    /// the reference and 1 more per unit (an atmosphere: theta_0 and 1 K). The alternative to set_temperatures().
    /// Before the first use only.
    ///
    /// Ends the program with print_error() when the unit is not more than 0.
    /// @param reference the temperature that becomes the lattice temperature 1
    /// @param unit the temperature difference that becomes the lattice temperature difference 1, more than 0
    /// @return this simulation
    Simulation& set_temperature_scale(si::Temperature reference, si::TemperatureDifference unit) {
        before_first_use("set_temperature_scale()");
        try {
            converter_ = converter_.with_temperature_scale(reference, unit);
        } catch(const SetupError& error) {
            const std::string message{ std::string("Simulation: ") + error.what() };
            print_error(message);
        }
        thermal_ = true;
        return *this;
    }

    /// @brief Sets the kinematic viscosity in place of the fluid's: a physical value, or a lattice value (an LES floor
    /// that scales with the grid; Diffusivity::lattice). Before the first use only.
    /// @param viscosity the kinematic viscosity, physical (m²/s) or in lattice units
    /// @return this simulation
    Simulation& set_viscosity(const Diffusivity& viscosity) {
        before_first_use("set_viscosity()");
        viscosity_ = viscosity;
        return *this;
    }

    /// @brief Sets the thermal diffusivity in place of the fluid's (TEMPERATURE): a physical value or a lattice value.
    /// The core gets the alpha that makes it diffuse with this value in every build (ScalarTransportRules::core_alpha()).
    /// Before the first use only.
    /// @param diffusivity the thermal diffusivity, physical (m²/s) or in lattice units
    /// @return this simulation
    Simulation& set_thermal_diffusivity(const Diffusivity& diffusivity) {
        before_first_use("set_thermal_diffusivity()");
        thermal_diffusivity_ = diffusivity;
        return *this;
    }

    /// @brief Sets the Boussinesq buoyancy of an atmosphere (VOLUME_FORCE, TEMPERATURE): the carrier as the volume
    /// force down z, and beta from g and theta_0 (BoussinesqBuoyancy), in place of the fluid's thermal expansion. It
    /// needs a temperature scale of 1 K per lattice unit. Before the first use only.
    ///
    /// Ends the program with print_error() when set_gravity() or set_body_force() is set.
    /// @param buoyancy gravity, theta_0 and the carrier force of the Boussinesq buoyancy
    /// @return this simulation
    Simulation& set_boussinesq_buoyancy(const BoussinesqBuoyancy& buoyancy) {
        before_first_use("set_boussinesq_buoyancy()");
        const si::Acceleration none{ au::ZERO };
        const bool forced{ body_force_.x != none || body_force_.y != none || body_force_.z != none };
        if(forced) {
            print_error("Simulation::set_boussinesq_buoyancy(): excludes set_gravity() and set_body_force(), which are set");
        }
        buoyancy_ = buoyancy;
        return *this;
    }

    /// @brief Sets the particles (PARTICLES), seeded with particles(). Before the first use only.
    /// @param count the number of particles
    /// @param relative_density the particles' density over the fluid's (a ratio; 2-way coupling with FORCE_FIELD)
    /// @return this simulation
    Simulation& set_particles(uint32_t count, float32_t relative_density = 1.0f) {
        before_first_use("set_particles()");
        particle_count_ = count;
        particle_density_ = relative_density;
        return *this;
    }

    /// @brief Voxelizes the model so that the fluid's force on it is measured (FORCE_FIELD): forces(). Before the
    /// first use only.
    ///
    /// Ends the program with print_error() without FORCE_FIELD in the build.
    /// @return this simulation
    Simulation& measure_forces() {
        before_first_use("measure_forces()");
#ifndef FORCE_FIELD
        print_error("Simulation::measure_forces() needs FORCE_FIELD in the EXTENSIONS");
#endif // FORCE_FIELD
        measure_forces_ = true;
        return *this;
    }

    /// @brief Keeps the domain's model out of the voxelization when the LBM is created: it is a moving part (parts())
    /// or voxelized by hand. Before the first use only.
    /// @return this simulation
    Simulation& skip_model_voxelization() {
        before_first_use("skip_model_voxelization()");
        voxelize_model_ = false;
        return *this;
    }
    /// @}

    /// @name The setup of the grid (a new builder each time)
    /// @{

    /// @brief Gives a builder of the boundaries, the solid objects and the initial flow. It creates the LBM.
    /// @return the builder
    BoundaryBuilder boundaries() { return BoundaryBuilder(lbm(), converter_, &runner()); }

    /// @brief Gives a configuration of the graphics. It creates the LBM.
    /// @return the configuration
    GraphicsConfig graphics() { return GraphicsConfig(lbm(), converter_); }
#ifdef SURFACE
    /// @brief Gives a builder of the free surface setup. It creates the LBM.
    /// @return the builder
    SurfaceBuilder surface() { return SurfaceBuilder(lbm(), converter_); }
#endif // SURFACE
#ifdef TEMPERATURE
    /// @brief Gives a builder of the thermal walls and the start. It creates the LBM.
    ///
    /// Ends the program with print_error() without set_temperatures() or set_temperature_scale().
    /// @return the builder
    ThermalBuilder thermal() {
        if(!thermal_) {
            print_error("Simulation: call set_temperatures() before the first use of a thermal simulation");
        }
        return ThermalBuilder(lbm(), converter_, gravity_axis_);
    }
#endif // TEMPERATURE
    /// @}

    /// @brief Gives a snapshot of the fields in physical units: read from the device now, or the setup before the
    /// first run. It creates the LBM.
    /// @return the snapshot
    FieldReader fields() { return FieldReader(lbm(), converter_, started_); }

    /// @name The components that act while the simulation runs (owned)
    /// @{

    /// @brief Gives the moving parts. It creates the LBM.
    /// @return the moving parts' manager
    MovingPartsManager& parts() {
        lbm();
        return *parts_;
    }
#ifdef SURFACE
    /// @brief Gives the wave maker. It creates the LBM.
    /// @return the wave maker
    WaveBoundary& wave_maker() {
        lbm();
        return *wave_maker_;
    }
#endif // SURFACE
#ifdef GRAPHICS
    /// @brief Gives the video, written by run_for() when built with GRAPHICS but not INTERACTIVE_GRAPHICS. It creates
    /// the LBM.
    /// @return the video recorder
    VideoRecorder& video() {
        lbm();
        return *video_;
    }
#endif // GRAPHICS
#ifdef FORCE_FIELD
    /// @brief Gives the force on the measured solids (measure_forces(), Solid::MEASURED). It creates the LBM.
    ///
    /// Ends the program with print_error() when nothing is measured.
    /// @return the force analyzer
    ForceAnalyzer& forces() {
        lbm();
        if(!measures_a_solid()) {
            print_error("Simulation::forces(): nothing is measured; call measure_forces() before the first use, or add a Solid::MEASURED shape");
        }
        return *forces_;
    }
#endif // FORCE_FIELD
#ifdef PARTICLES
    /// @brief Gives the particles' seeding. It creates the LBM.
    ///
    /// Ends the program with print_error() without set_particles().
    /// @return the particle manager
    ParticleManager& particles() {
        lbm();
        if(particle_count_ == 0u) {
            print_error("Simulation::particles(): call set_particles() before the first use");
        }
        return *particles_;
    }
#endif // PARTICLES
    /// @}

    /// @name Running
    /// @{

    /// @brief Calls a task every interval of simulated time, from the start on or first after one interval (see
    /// Runner). It creates the LBM.
    /// @param interval the interval; less than half a time step counts as one step
    /// @param task the task, called with the simulated time since the start
    /// @param first when the task is called the first time
    /// @return this simulation
    Simulation& every(si::Duration interval, Runner::Task task, Runner::FirstCall first = Runner::FirstCall::AT_ZERO) {
        runner().every(interval, std::move(task), first);
        return *this;
    }

    /// @brief Calls a task every time step. It creates the LBM.
    /// @param task the task, called with the simulated time since the start
    /// @return this simulation
    Simulation& every_step(Runner::Task task) {
        runner().every_step(std::move(task));
        return *this;
    }

    /// @brief Runs a simulated time from now. The first call records the video, if there is one.
    /// @param time the simulated time, rounded to the nearest time step
    void run_for(si::Duration time) {
        start_video(time);
        started_ = true;
        runner().run_for(time);
    }

    /// @brief Runs until a task calls stop() (with interactive graphics: until the window is closed).
    void run() {
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
        if(video().has_views()) {
            print_warning("Simulation::run(): a video needs run_for() with the simulated time it spans; no frames are written");
        }
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
        started_ = true;
        runner().run();
    }

    /// @brief Ends run() or run_for() once the tasks of this time step are done; call it from a task.
    void stop() { runner().stop(); }
    /// @}

    /// @name For expert use
    /// @{

    /// @brief Gives the core's LBM, creating it on the first use.
    /// @return the LBM
    LBM& lbm() {
        if(!lbm_) {
            create();
        }
        return *lbm_;
    }

    /// @brief Gives the conversions between the Setup API's quantities and this simulation's lattice: its cell, time
    /// step and density, and its temperature scale when one is set.
    /// @return the converter
    const si::LatticeConverter& converter() const { return converter_; }

    /// @brief Gives this build's device memory per cell, its DDF buffer per cell, the devices' smallest buffer limit
    /// and the core's halo layers: for an estimate before any Simulation exists (CblDescription::estimate()).
    /// @return the lattice memory
    static LatticeMemory build_lattice_memory() { return lattice_memory(); }

    /// @brief Estimates what a run of this simulation costs, before the LBM exists: the planned grid and domains, the
    /// device memory per domain with its halo, the DDF buffer against the device's limit, the steps, and the wall time
    /// from a measured speed.
    ///
    /// Ends the program with print_error() for a speed that is not more than 0.
    /// @param time the simulated time of the run
    /// @param speed a measured speed of this build on this device, or none for no wall time
    /// @return the estimate
    RunEstimate estimate(si::Duration time, std::optional<si::LatticeUpdateRate> speed = std::nullopt) const {
        std::optional<RunEstimate> e{};
        try {
            const GridSize grid{ .x = plan_.Nx, .y = plan_.Ny, .z = plan_.Nz };
            const LatticeMemory memory{ lattice_memory() };
            const std::uint64_t steps{ converter_.steps(time) };
            e.emplace(RunEstimate::of(grid, plan_.gpus, memory, steps, speed));
        } catch(const SetupError& error) {
            const std::string message{ std::string("Simulation::estimate(): ") + error.what() };
            print_error(message);
        }
        return *e;
    }

    /// @return the grid and the model's place, in cells
    const DomainPlan& plan() const { return plan_; }

    /// @brief Gives the model's file name.
    /// @return the file in resources/, or empty for Domain::box()
    std::string model_file() const {
        std::string file{};
        if(domain_.model()) {
            file = domain_.model()->file();
        }
        return file;
    }

    /// @brief Gives the Reynolds number of the reference length (the model's length, or the domain's longest side)
    /// and the reference speed.
    /// @return reference length times reference speed over the fluid's kinematic viscosity (a ratio)
    double reynolds_number() const {
        const si::Length length{ plan_.reference_length };
        const si::KinematicViscosity product{ length * reference_speed_ };
        const double reynolds{ au::as_raw_number(product / fluid_.kinematic_viscosity) };
        return reynolds;
    }
    /// @}

private:
    Domain domain_;                  ///< the domain
    FluidProperties fluid_;          ///< the fluid
    si::Speed reference_speed_{ au::ZERO }; ///< the reference speed (the rule's lattice speed dx/dt with a time step rule)
    DomainPlan plan_;                ///< the grid and the model's place
    si::LatticeConverter converter_; ///< the conversions: the cell, the time step, the density, the temperature scale

    si::AccelerationVector body_force_{};                         ///< set_body_force() or set_gravity(): a force per mass
    Axis gravity_axis_ = Axis::Z;                                 ///< the axis of set_gravity()
    si::SurfaceTension surface_tension_{ au::ZERO };              ///< set_surface_tension()
    bool thermal_ = false;                                        ///< true after set_temperatures() or set_temperature_scale()
    std::optional<Diffusivity> viscosity_{};                      ///< set_viscosity(): in place of the fluid's, or none
    std::optional<Diffusivity> thermal_diffusivity_{};            ///< set_thermal_diffusivity(): in place of the fluid's, or none
    std::optional<BoussinesqBuoyancy> buoyancy_{};                ///< set_boussinesq_buoyancy(), or none
    uint32_t particle_count_ = 0u;                                ///< set_particles(): the number of particles
    float32_t particle_density_ = 1.0f;                           ///< set_particles(): the density over the fluid's
    bool measure_forces_ = false;                                 ///< true after measure_forces()
    bool voxelize_model_ = true;                                  ///< false after skip_model_voxelization()
    bool video_started_ = false;                                  ///< true when the video's frames are scheduled
    bool started_ = false;                                        ///< true after run_for() or run(): the device holds the fields

    std::unique_ptr<LBM> lbm_{};                  ///< the core's LBM, created on the first use
    std::unique_ptr<Runner> runner_{};            ///< the runner, created with the LBM
    std::unique_ptr<MovingPartsManager> parts_{}; ///< the moving parts, created with the LBM
#ifdef SURFACE
    std::unique_ptr<WaveBoundary> wave_maker_{};  ///< the wave maker, created with the LBM
#endif // SURFACE
#ifdef GRAPHICS
    std::unique_ptr<VideoRecorder> video_{};      ///< the video, created with the LBM
#endif // GRAPHICS
#ifdef FORCE_FIELD
    std::unique_ptr<ForceAnalyzer> forces_{};     ///< the force analyzer, created with the LBM
#endif // FORCE_FIELD
#ifdef PARTICLES
    std::unique_ptr<ParticleManager> particles_{}; ///< the particles, created with the LBM
#endif // PARTICLES

    /// @brief Checks a domain's settings; ends the program with print_error() when they conflict.
    /// @param domain the domain as the user gave it
    /// @return a copy of the domain (a copy, so that no reference to a temporary argument outlives the call)
    static Domain validated(const Domain& domain) {
        try {
            domain.validate();
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        Domain checked{ domain };
        return checked;
    }

    /// @brief Gives the domain's plan with its voxelization chosen. Ends the program with print_error() when the
    /// geometry file is not found, the domain cannot be planned, the build's per-level forcing cannot run on the
    /// planned split, or the model's SDF cannot be made.
    /// @param domain the validated domain
    /// @return the plan
    static DomainPlan prepared_plan(const Domain& domain) {
        if(domain.model()) {
            validate_geometry_file(domain);
        }
        DomainPlan plan{ planned(domain) };
        // the build's per-level forcing cannot run on a split along z: refused here, before any allocation
        const TransportBuild build{ current_build() };
        const std::optional<std::string> refusal{ DomainPlanner::level_forcing_refusal(plan, build) };
        if(refusal) {
            print_error(*refusal);
        }
        try {
            DomainPlanner::choose_voxelization(plan, domain);
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        return plan;
    }

    /// @brief Gives the converter of the Mach path: the plan's reference length and cells, the reference speed at the
    /// Mach number's lattice speed, and the fluid's density. Ends the program with print_error() when a value is not
    /// more than 0.
    /// @param plan the domain's plan
    /// @param reference_speed the flow's reference speed, usually its fastest
    /// @param mach the lattice Mach number of the reference speed
    /// @param density the fluid's density
    /// @return the converter
    static si::LatticeConverter mach_converter(const DomainPlan& plan, si::Speed reference_speed, si::LatticeMach mach, si::Density density) {
        std::optional<si::LatticeConverter> converter{};
        try {
            const si::Length reference_length{ plan.reference_length };
            const double reference_cells{ static_cast<double>(plan.lbm_reference_size) };
            const si::LatticeConverter c{ si::LatticeConverter::from_reference(reference_length, reference_cells, reference_speed, mach) };
            converter.emplace(c.with_density(density));
        } catch(const SetupError& error) {
            const std::string message{ std::string("Simulation: ") + error.what() };
            print_error(message);
        }
        return *converter;
    }

    /// @brief Gives the converter of a time step rule at the domain's cell, with the fluid's density. The converter
    /// holds the step as the core will: a float (one source of dt for the core and the API). Ends the program with
    /// print_error() when the cell is not a float exactly or a value is not more than 0.
    /// @param domain the validated domain, with its cell size
    /// @param rule the time step rule
    /// @param density the fluid's density
    /// @return the converter
    static si::LatticeConverter rule_converter(const Domain& domain, const TimeStepRule& rule, si::Density density) {
        std::optional<si::LatticeConverter> converter{};
        try {
            const si::Length cell{ *domain.cell() };
            const si::Duration step{ rule.step(cell) };
            const si::LatticeConverter c(cell, step);
            converter.emplace(c.with_density(density));
        } catch(const SetupError& error) {
            const std::string message{ std::string("Simulation: ") + error.what() };
            print_error(message);
        }
        return *converter;
    }

    /// @brief Sets the core's global units object from the converter (labels and file output).
    void set_core_units() const {
        const si::CoreUnits numbers{ converter_.core_units() };
        units.set_m_kg_s(numbers.m, numbers.kg, numbers.s);
    }

    /// @brief Gives this build's lattice memory: the device memory per cell of this example's lattice (it depends on
    /// its extensions), the bytes per cell of its DDF buffer, the smallest buffer limit of the devices (the planner
    /// splits a lattice over it) and the core's halo layers per domain cut.
    /// @return the lattice memory
    static LatticeMemory lattice_memory() {
        uint32_t max_buffer_mb{ 0u };
        for(const Device_Info& device : get_devices(false)) {
            if(max_buffer_mb == 0u || device.max_global_buffer < max_buffer_mb) {
                max_buffer_mb = device.max_global_buffer;
            }
        }
#ifdef D2Q9
        const uint32_t dimensions{ 2u };
#else
        const uint32_t dimensions{ 3u };
#endif
        const double cell_bytes{ static_cast<double>(bytes_per_cell_device()) };
        const double ddf_bytes{ static_cast<double>(ddf_bytes_per_cell()) };
        const double max_buffer_mib{ static_cast<double>(max_buffer_mb) }; // the core's MB
        const LatticeMemory memory{ .memory_per_cell = au::bytes(cell_bytes),
                                    .dimensions = dimensions,
                                    .buffer_memory_per_cell = au::bytes(ddf_bytes),
                                    .max_buffer = si::mebibytes(max_buffer_mib),
                                    .halo_layers = halo_width() };
        return memory;
    }

    /// @brief Gives the bytes per cell of the DDF buffer fi, the largest single buffer: the velocity set's DDFs, each 2
    /// bytes with FP16S or FP16C, else 4 (the core's fi, Memory<fpxx>(device, N, velocity_set), from the same defines).
    /// @return the bytes per cell
    static uint32_t ddf_bytes_per_cell() {
#if defined(D2Q9)
        const uint32_t ddfs{ 9u };
#elif defined(D3Q15)
        const uint32_t ddfs{ 15u };
#elif defined(D3Q27)
        const uint32_t ddfs{ 27u };
#else
        const uint32_t ddfs{ 19u }; // D3Q19, the default
#endif
#if defined(FP16S) || defined(FP16C)
        const uint32_t bytes_per_ddf{ 2u };
#else
        const uint32_t bytes_per_ddf{ 4u };
#endif
        const uint32_t bytes{ ddfs * bytes_per_ddf };
        return bytes;
    }

    /// @brief Plans a domain; ends the program with print_error() when it cannot be planned.
    /// @param domain the validated domain
    /// @return the plan
    static DomainPlan planned(const Domain& domain) {
        std::optional<DomainPlan> plan{};
        try {
            plan.emplace(DomainPlanner::plan(domain, lattice_memory()));
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        return *plan;
    }

    /// @brief Checks that the model's file and the files it needs() are in resources/; otherwise prints its
    /// instructions() and ends the program with print_error().
    /// @param domain a domain with a model
    static void validate_geometry_file(const Domain& domain) {
        const Model& model{ *domain.model() };
        std::string missing{};
        if(get_resource_path(model.file()).empty()) {
            missing = model.file();
        }
        for(const std::string& file : model.required_files()) {
            if(!get_resource_path(file).empty()) {
                continue;
            }
            if(!missing.empty()) {
                missing += ", ";
            }
            missing += file;
        }
        if(missing.empty()) {
            return;
        }
        for(const std::string& line : model.instruction_lines()) {
            print_info(line);
        }
        std::string message{ "geometry file not found: " + missing + "; searched in" };
#ifdef FLUIDX3D_RESOURCE_DIR
        message += " " + std::string(FLUIDX3D_RESOURCE_DIR) + "/ and";
#endif // FLUIDX3D_RESOURCE_DIR
        message += " " + get_exe_path() + "resources/";
        print_error(message);
    }

    /// @brief Ends the program with print_error() when the LBM already exists.
    /// @param what the setter that must come before the first use
    void before_first_use(const char* what) const {
        if(lbm_) {
            print_error(std::string("Simulation: call ") + what + " before the simulation's first use (a builder, the graphics or a run)");
        }
    }

    /// @brief Converts an acceleration vector to the lattice.
    /// @param a an acceleration
    /// @return the acceleration in cells per time step squared along x, y and z
    float3 lattice(const si::AccelerationVector& a) const {
        const float x{ converter_.acceleration(a.x) };
        const float y{ converter_.acceleration(a.y) };
        const float z{ converter_.acceleration(a.z) };
        const float3 lattice_acceleration{ x, y, z };
        return lattice_acceleration;
    }

    /// @brief Gives the rule's lattice speed dx/dt at the domain's cell: the reference speed of a rule simulation.
    /// Ends the program with print_error() when the domain has no cell size.
    /// @param rule a time step rule
    /// @param domain the validated domain
    /// @return the lattice speed
    static si::Speed rule_speed(const TimeStepRule& rule, const Domain& domain) {
        if(!domain.cell()) {
            print_error("Simulation: a time step rule needs the domain's cell size (Domain::cell_size())");
        }
        std::optional<si::Speed> speed{};
        try {
            const si::Length cell{ *domain.cell() };
            speed.emplace(rule.lattice_speed(cell));
        } catch(const SetupError& error) {
            const std::string message{ std::string("Simulation: ") + error.what() };
            print_error(message);
        }
        return *speed;
    }

    /// @brief Gives the lattice force per cell: the body force, or the Boussinesq buoyancy's carrier when it is set.
    /// Ends the program with print_error() when the buoyancy has no temperature scale or does not fit the column.
    /// @return the force along x, y and z in lattice units
    float3 lattice_force() const {
        if(!buoyancy_) {
            const float3 body_force{ lattice(body_force_) };
            return body_force;
        }
        if(!thermal_) {
            print_error("Simulation: set_boussinesq_buoyancy() needs a temperature scale (set_temperature_scale())");
        }
        const BoussinesqBuoyancy& buoyancy{ buoyancy_.value() };
        try {
            buoyancy.validate(plan_.Nz);
        } catch(const SetupError& error) {
            const std::string message{ std::string("Simulation: ") + error.what() };
            print_error(message);
        }
        const float fz{ buoyancy.lattice_force() };
        const float3 force{ 0.0f, 0.0f, fz };
        return force;
    }

    /// @brief Gives the thermal diffusivity in lattice units: set_thermal_diffusivity(), else the fluid's.
    /// @return the diffusivity, 0 without a temperature scale
    float32_t lattice_thermal_diffusivity() const {
        if(!thermal_) {
            return 0.0f;
        }
        const Diffusivity fluid_diffusivity{ Diffusivity::physical(fluid_.thermal_diffusivity) };
        const Diffusivity diffusivity{ thermal_diffusivity_.value_or(fluid_diffusivity) };
        const float32_t lattice_value{ diffusivity.lattice_value(converter_) };
        return lattice_value;
    }

    /// @brief Gives the core's thermal expansion beta: the Boussinesq buoyancy's, else the fluid's.
    /// Ends the program with print_error() when the Boussinesq buoyancy has a temperature unit other than 1 K.
    /// @return beta, 0 without a temperature scale
    float32_t lattice_expansion() const {
        if(!thermal_) {
            return 0.0f;
        }
        if(!buoyancy_) {
            const float32_t fluid_expansion{ converter_.expansion(fluid_.thermal_expansion) };
            return fluid_expansion;
        }
        // BoussinesqBuoyancy's beta is per kelvin: the lattice temperature unit must be 1 K (an atmosphere's scale)
        if(converter_.temperature_unit() != si::LatticeConverter::kernel_temperature_unit) {
            print_error("Simulation: set_boussinesq_buoyancy() needs a temperature scale of 1 K per lattice unit");
        }
        const BoussinesqBuoyancy& buoyancy{ buoyancy_.value() };
        const float32_t expansion{ buoyancy.lattice_expansion(converter_) };
        return expansion;
    }

    /// @brief Gives the kinematic viscosity in lattice units: set_viscosity(), else the fluid's.
    /// @return the viscosity in cells squared per time step
    float32_t lattice_viscosity() const {
        const Diffusivity fluid_viscosity{ Diffusivity::physical(fluid_.kinematic_viscosity) };
        const Diffusivity viscosity{ viscosity_.value_or(fluid_viscosity) };
        const float32_t nu{ viscosity.lattice_value(converter_) };
        return nu;
    }

    /// @brief Gives the alpha for the core that makes it diffuse with the wanted diffusivity in every build
    /// (ScalarTransportRules::core_alpha(): without THERMAL_DIFFUSIVITY_FIX the core diffuses with alpha/2), and
    /// prints the transport's warnings for this build. Ends the program with print_error() when the build cannot carry
    /// the diffusivity.
    /// @param diffusivity the thermal diffusivity in lattice units
    /// @param beta the core's thermal expansion
    /// @param f the lattice force
    /// @return the alpha for the core, 0 without a temperature scale
    float32_t core_alpha(float32_t diffusivity, float32_t beta, const float3& f) const {
        if(!thermal_) {
            return 0.0f;
        }
        const TransportBuild build{ current_build() };
        float32_t alpha{ 0.0f };
        try {
            alpha = ScalarTransportRules::core_alpha(diffusivity, build);
        } catch(const SetupError& error) {
            const std::string message{ std::string("Simulation: ") + error.what() };
            print_error(message);
        }
        // the walls are not known here: the warnings assume the scalar can touch them. Buoyancy needs an expansion
        // and a force for it to act along.
        const bool expands{ beta != 0.0f };
        const bool forced{ f.x != 0.0f || f.y != 0.0f || f.z != 0.0f };
        const bool buoyant{ expands && forced };
        const ScalarContext context{ .solid_walls = true, .buoyant = buoyant, .moist = build.moisture };
        const std::vector<std::string> warnings{ ScalarTransportRules::warnings(diffusivity, build, context) };
        for(const std::string& line : warnings) {
            const std::string message{ "Simulation: " + line };
            print_warning(message);
        }
        return alpha;
    }

    /// @brief Ends the program with print_error() when a physics setting needs an extension that the build lacks.
    /// @param f the lattice force
    void check_extensions(const float3& f) const {
#ifndef VOLUME_FORCE
        const bool forced{ f.x != 0.0f || f.y != 0.0f || f.z != 0.0f };
        if(forced) {
            print_error("Simulation: gravity or a body force needs VOLUME_FORCE in the EXTENSIONS");
        }
#else
        (void)f;
#endif // VOLUME_FORCE
#ifndef SURFACE
        if(surface_tension_ != si::SurfaceTension{ au::ZERO }) {
            print_error("Simulation: a surface tension needs SURFACE in the EXTENSIONS");
        }
#endif // SURFACE
#ifdef TEMPERATURE
        if(!thermal_) {
            print_error("Simulation: a thermal simulation (TEMPERATURE) needs set_temperatures()");
        }
#else
        if(thermal_) {
            print_error("Simulation: set_temperatures() needs TEMPERATURE in the EXTENSIONS");
        }
#endif // TEMPERATURE
#ifndef PARTICLES
        if(particle_count_ > 0u) {
            print_error("Simulation: set_particles() needs PARTICLES in the EXTENSIONS");
        }
#endif // PARTICLES
    }

    /// @brief Creates the LBM with the physics set so far, voxelizes the model and creates the owned components.
    void create() {
        const float3 f{ lattice_force() };
        check_extensions(f);
        const float32_t beta{ lattice_expansion() };
        const float32_t nu{ lattice_viscosity() };
        const float32_t diffusivity{ lattice_thermal_diffusivity() };
        const float32_t alpha{ core_alpha(diffusivity, beta, f) };
        const float32_t sigma{ lattice_surface_tension() };
        lbm_ = std::make_unique<LBM>(plan_.Nx, plan_.Ny, plan_.Nz, plan_.gpus.x, plan_.gpus.y, plan_.gpus.z, nu, f.x, f.y, f.z, sigma, alpha, beta,
                                     particle_count_, particle_density_);
        if(domain_.model() && voxelize_model_) {
            voxelize();
        }
        runner_ = std::make_unique<Runner>(*lbm_, converter_);
        parts_ = std::make_unique<MovingPartsManager>(*lbm_, converter_, plan_, *runner_);
#ifdef SURFACE
        wave_maker_ = std::make_unique<WaveBoundary>(*lbm_, converter_, *runner_);
#endif // SURFACE
#ifdef GRAPHICS
        video_ = std::make_unique<VideoRecorder>(*lbm_, converter_);
#endif // GRAPHICS
#ifdef FORCE_FIELD
        forces_ = std::make_unique<ForceAnalyzer>(*lbm_, converter_);
#endif // FORCE_FIELD
#ifdef PARTICLES
        particles_ = std::make_unique<ParticleManager>(*lbm_, converter_);
#endif // PARTICLES
    }

    /// @brief Gives the surface tension in lattice units. A surface tension of 0 needs no density.
    /// @return the lattice surface tension
    float32_t lattice_surface_tension() const {
        if(surface_tension_ == si::SurfaceTension{ au::ZERO }) {
            return 0.0f;
        }
        const float32_t sigma{ converter_.surface_tension(surface_tension_) };
        return sigma;
    }

    /// @brief Voxelizes the domain's model as planned: a mirrored half model, an SDF or an STL.
    void voxelize() {
        const uchar flag{ measure_forces_ ? static_cast<uchar>(TYPE_S | TYPE_X) : static_cast<uchar>(TYPE_S) };
        if(plan_.mirror.has_value()) { // symmetric half model
            const Axis mirror{ plan_.mirror.value() };
            const std::unique_ptr<Mesh> mesh{ MeshLoader::load_mirrored(plan_, mirror) };
            lbm_->voxelize_mesh_on_device(mesh.get(), flag);
            return;
        }
        if(plan_.voxelize_sdf) {
            lbm_->voxelize_sdf(plan_.voxelization_path, plan_.center_lbm, plan_.rotation_matrix, plan_.voxel_size, flag);
        } else {
            lbm_->voxelize_stl(plan_.voxelization_path, plan_.center_lbm, plan_.rotation_matrix, plan_.voxel_size, flag);
        }
    }

    /// @brief Gives the runner, creating the LBM if needed.
    /// @return the runner
    Runner& runner() {
        lbm();
        return *runner_;
    }

    /// @brief Schedules the video's frames on the first run, when built with GRAPHICS but not INTERACTIVE_GRAPHICS.
    /// @param time the simulated time of the run
    void start_video(si::Duration time) {
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
        if(video().has_views() && !video_started_) {
            video_->start(*runner_, time);
            video_started_ = true;
        }
#else
        (void)time;
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
    }

#ifdef FORCE_FIELD
    /// @brief Tells whether a solid is measured: the model with measure_forces(), or a Solid::MEASURED shape on the
    /// host's flags.
    /// @return true when a solid is measured
    bool measures_a_solid() {
        if(measure_forces_) {
            return true;
        }
        const uint64_t count{ lbm_->get_N() };
        for(uint64_t n = 0ull; n < count; n++) {
            const uchar measured_bit{ static_cast<uchar>(lbm_->flags[n] & TYPE_X) };
            if(measured_bit != 0u) {
                return true;
            }
        }
        return false;
    }
#endif // FORCE_FIELD
};
