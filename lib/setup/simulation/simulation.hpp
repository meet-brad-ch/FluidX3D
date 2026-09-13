#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/fluids.hpp"
#include "setup/core/unit_scale.hpp"
#include "setup/core/temperature_scale.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/boundaries/boundary_builder.hpp"
#include "setup/domain/domain.hpp"
#include "setup/domain/domain_plan.hpp"
#include "setup/domain/lattice.hpp"
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

extern Units units; ///< the core's global units, set once for its labels and file output

/// @brief A simulation in physical units.
///
/// The domain, the fluid and the reference speed (usually the fastest in the flow) set the lattice: the grid, the cell
/// size and the time step; the Reynolds and lattice Mach numbers are printed. The core's LBM is created on the first
/// use (a builder, the graphics, a run) with the physics set before it: gravity or a body force, the surface tension,
/// the temperatures, the particles. A model in the domain is voxelized then. The builders write to the grid, the
/// owned components (video(), parts(), forces(), particles(), wave_maker()) keep their state while it runs, and
/// run_for() or run() drive the simulation, with tasks every interval of simulated time.
/// @code
/// Simulation sim(Domain::around(Model("Cow_t.stl").length(2.4_m)).size(1.85_m, 3.69_m, 1.85_m).on_floor().vram(1000_mb),
///                Fluid::AIR, 1.0_mps);
/// sim.boundaries().set_solid_floor().set_open_boundaries().initialize_velocity_y(1.0_mps).apply();
/// sim.graphics().show_surface().show_vortices().apply();
/// sim.video().add(CameraView::orbit(-40_deg, 20_deg)).set_length(10.0_s); // written when built for video
/// sim.run_for(10.0_s);
/// @endcode
class Simulation {
public:
    /// @param domain           the domain in physical units (exits with a message if its settings conflict or its
    ///                         geometry file is not found)
    /// @param fluid            its density sets the lattice density 1, its viscosity the lattice viscosity
    /// @param reference_speed  the reference velocity, usually the fastest in the flow
    /// @param mach             how compressible the simulation makes the flow; it sets the time step (see LatticeMach)
    Simulation(const Domain& domain, const FluidProperties& fluid, Speed reference_speed, LatticeMach mach = LatticeMach())
        : domain_(validated(domain)), fluid_(fluid), reference_speed_(reference_speed) {
        if(domain_.model()) validate_geometry_file();
        plan_ = planned();
        DomainPlanner::choose_voxelization(plan_, domain_);
        scale_ = UnitScale::from_reference(Length::from_si(plan_.si_reference_size), plan_.lbm_reference_size,
                                           reference_speed, mach.lattice_speed(), fluid.density);
        units.set_m_kg_s(scale_.cell_size().si(), scale_.mass_unit().si(), scale_.time_step().si());
        print_info("Lattice Mach number " + to_string(mach.value(), 4u) + " (compressibility error ~" +
                   to_string(100.0f * sq(mach.value()), 2u) + " %)");
        print_info("Re = " + to_string(static_cast<uint32_t>(reynolds_number())));
    }

    Simulation(const Simulation&) = delete;            ///< not copyable: it owns the LBM
    Simulation& operator=(const Simulation&) = delete; ///< not copyable: it owns the LBM

    /// @name Physics, before the first use
    /// @{

    /// @brief Gravity along the negative direction of an axis (VOLUME_FORCE).
    /// @param gravity the acceleration
    /// @param axis    the axis, Z by default
    /// @return this simulation
    Simulation& set_gravity(Acceleration gravity, Axis axis = Axis::Z) {
        before_first_use("set_gravity()");
        gravity_axis_ = axis;
        return set_body_force({ axis == Axis::X ? -gravity : Acceleration{},
                                axis == Axis::Y ? -gravity : Acceleration{},
                                axis == Axis::Z ? -gravity : Acceleration{} });
    }

    /// @brief A body force per mass in every cell (VOLUME_FORCE), such as gravity on a slope or a pressure gradient
    /// over the density; also while the simulation runs (a turning gravity).
    /// @param force the force per mass
    /// @return this simulation
    Simulation& set_body_force(const AccelerationVector& force) {
        body_force_ = force;
        if(lbm_) {
            const float3 f = lattice(force);
            lbm_->set_f(f.x, f.y, f.z);
        }
        return *this;
    }

    /// @brief The surface tension of a free surface (SURFACE); 0 is none.
    /// @param surface_tension the surface tension
    /// @return this simulation
    Simulation& set_surface_tension(SurfaceTension surface_tension) {
        before_first_use("set_surface_tension()");
        surface_tension_ = surface_tension;
        return *this;
    }

    /// @brief The temperature range of a thermal simulation (TEMPERATURE): cold and hot become the lattice
    /// temperatures 0.5 and 1.5, and the fluid's thermal expansion drives the buoyancy (see TemperatureScale).
    /// @param cold the cold temperature
    /// @param hot  the hot temperature, above the cold one
    /// @return this simulation
    Simulation& set_temperatures(Temperature cold, Temperature hot) {
        before_first_use("set_temperatures()");
        if(!(hot > cold)) print_error("Simulation::set_temperatures(): the hot temperature must be above the cold one");
        temperature_scale_ = TemperatureScale::between(cold, hot);
        return *this;
    }

    /// @brief Particles (PARTICLES), seeded with particles().
    /// @param count            the number of particles
    /// @param relative_density the particles' density relative to the fluid's (2-way coupling with FORCE_FIELD)
    /// @return this simulation
    Simulation& set_particles(uint32_t count, float32_t relative_density = 1.0f) {
        before_first_use("set_particles()");
        particle_count_ = count;
        particle_density_ = relative_density;
        return *this;
    }

    /// @brief The model is voxelized so the fluid's force on it is measured (FORCE_FIELD): forces().
    /// @return this simulation
    Simulation& measure_forces() {
        before_first_use("measure_forces()");
#ifndef FORCE_FIELD
        print_error("Simulation::measure_forces() needs FORCE_FIELD in the EXTENSIONS");
#endif // FORCE_FIELD
        measure_forces_ = true;
        return *this;
    }

    /// @brief The domain's model is not voxelized when the LBM is created: it is a moving part (parts()) or voxelized
    /// by hand.
    /// @return this simulation
    Simulation& skip_model_voxelization() {
        before_first_use("skip_model_voxelization()");
        voxelize_model_ = false;
        return *this;
    }
    /// @}

    /// @name The setup of the grid (a new builder each time)
    /// @{

    /// @return a builder of the boundaries, solid objects and initial flow
    BoundaryBuilder boundaries() { return BoundaryBuilder(lbm(), scale_, &runner()); }

    /// @return a configuration of the graphics
    GraphicsConfig graphics() { return GraphicsConfig(lbm(), scale_); }
#ifdef SURFACE
    /// @return a builder of the free surface setup
    SurfaceBuilder surface() { return SurfaceBuilder(lbm(), scale_); }
#endif // SURFACE
#ifdef TEMPERATURE
    /// @return a builder of the thermal walls and start (exits with a message without set_temperatures())
    ThermalBuilder thermal() { return ThermalBuilder(lbm(), scale_, temperature_scale(), gravity_axis_); }
#endif // TEMPERATURE
    /// @}

    /// @return a snapshot of the fields in physical units: read from the device now, or the setup before the first run
    FieldReader fields() { return FieldReader(lbm(), scale_, started_); }

    /// @name The components that act while the simulation runs (owned)
    /// @{

    /// @return the moving parts
    MovingPartsManager& parts() {
        lbm();
        return *parts_;
    }
#ifdef SURFACE
    /// @return the wave maker
    WaveBoundary& wave_maker() {
        lbm();
        return *wave_maker_;
    }
#endif // SURFACE
#ifdef GRAPHICS
    /// @return the video, written by run_for() when built with GRAPHICS but not INTERACTIVE_GRAPHICS
    VideoRecorder& video() {
        lbm();
        return *video_;
    }
#endif // GRAPHICS
#ifdef FORCE_FIELD
    /// @return the force on the measured solids (measure_forces(), Solid::MEASURED); exits with a message if there is
    /// none
    ForceAnalyzer& forces() {
        lbm();
        if(!measures_a_solid()) print_error("Simulation::forces(): nothing is measured; call measure_forces() before the first use, or add a Solid::MEASURED shape");
        return *forces_;
    }
#endif // FORCE_FIELD
#ifdef PARTICLES
    /// @return the particles' seeding (set_particles() first)
    ParticleManager& particles() {
        lbm();
        if(particle_count_ == 0u) print_error("Simulation::particles(): call set_particles() before the first use");
        return *particles_;
    }
#endif // PARTICLES
    /// @}

    /// @name Running
    /// @{

    /// @brief Calls a task every interval of simulated time, from the start on (see Runner).
    /// @param interval the interval
    /// @param task     the task
    /// @return this simulation
    Simulation& every(Duration interval, Runner::Task task) {
        runner().every(interval, std::move(task));
        return *this;
    }

    /// @brief Calls a task every time step.
    /// @param task the task
    /// @return this simulation
    Simulation& every_step(Runner::Task task) {
        runner().every_step(std::move(task));
        return *this;
    }

    /// @brief Runs a simulated time from now; the first call records the video, if there is one.
    /// @param time the simulated time
    void run_for(Duration time) {
        start_video(time);
        started_ = true;
        runner().run_for(time);
    }

    /// Runs until a task calls stop() (with interactive graphics: until the window is closed).
    void run() {
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
        if(video().has_views()) print_warning("Simulation::run(): a video needs run_for() with the simulated time it spans; no frames are written");
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
        started_ = true;
        runner().run();
    }

    /// Ends run() or run_for() once the tasks of this time step are done; call it from a task.
    void stop() { runner().stop(); }
    /// @}

    /// @name For expert use
    /// @{

    /// @return the core's LBM, created on the first use
    LBM& lbm() {
        if(!lbm_) create();
        return *lbm_;
    }

    /// @return the scale between SI and lattice units
    const UnitScale& unit_scale() const { return scale_; }

    /// @return the grid and the model's place, in cells
    const DomainPlan& plan() const { return plan_; }

    /// @return the scale of set_temperatures(); exits with a message if it is not set
    const TemperatureScale& temperature_scale() const {
        if(!temperature_scale_) print_error("Simulation: call set_temperatures() before the first use of a thermal simulation");
        return *temperature_scale_;
    }

    /// @return the model's file in resources/, empty for Domain::box()
    std::string model_file() const { return domain_.model() ? domain_.model()->file() : std::string(); }

    /// @return the Reynolds number of the reference length (the model's length, or the domain's longest side) and speed
    float32_t reynolds_number() const { return plan_.si_reference_size * reference_speed_.si() / fluid_.kinematic_viscosity.si(); }
    /// @}

private:
    Domain domain_;         ///< the domain
    FluidProperties fluid_; ///< the fluid
    Speed reference_speed_; ///< the reference velocity
    DomainPlan plan_;       ///< the grid and the model's place
    UnitScale scale_;       ///< the scale between SI and lattice units

    AccelerationVector body_force_;                     ///< set_body_force() or set_gravity()
    Axis gravity_axis_ = Axis::Z;                       ///< the axis of set_gravity()
    SurfaceTension surface_tension_;                    ///< set_surface_tension()
    std::optional<TemperatureScale> temperature_scale_; ///< set_temperatures()
    uint32_t particle_count_ = 0u;                      ///< set_particles()
    float32_t particle_density_ = 1.0f;                 ///< set_particles(): relative to the fluid's
    bool measure_forces_ = false;                       ///< measure_forces()
    bool voxelize_model_ = true;                        ///< false after skip_model_voxelization()
    bool video_started_ = false;                        ///< whether the video's frames are scheduled
    bool started_ = false;                              ///< run_for() or run() was called: the device holds the fields

    std::unique_ptr<LBM> lbm_;                  ///< the core's LBM, created on the first use
    std::unique_ptr<Runner> runner_;            ///< the runner, created with the LBM
    std::unique_ptr<MovingPartsManager> parts_; ///< the moving parts, created with the LBM
#ifdef SURFACE
    std::unique_ptr<WaveBoundary> wave_maker_;  ///< the wave maker, created with the LBM
#endif // SURFACE
#ifdef GRAPHICS
    std::unique_ptr<VideoRecorder> video_;      ///< the video, created with the LBM
#endif // GRAPHICS
#ifdef FORCE_FIELD
    std::unique_ptr<ForceAnalyzer> forces_;     ///< the force analyzer, created with the LBM
#endif // FORCE_FIELD
#ifdef PARTICLES
    std::unique_ptr<ParticleManager> particles_; ///< the particles, created with the LBM
#endif // PARTICLES

    /// @param domain a domain
    /// @return the domain, if its settings do not conflict (else exits with a message)
    static const Domain& validated(const Domain& domain) {
        try {
            domain.validate();
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        return domain;
    }

    /// @return the device memory per cell of this example's lattice (it depends on its extensions)
    static LatticeMemory lattice_memory() {
#ifdef D2Q9
        return { bytes_per_cell_device(), 2u };
#else
        return { bytes_per_cell_device(), 3u };
#endif
    }

    /// @return the domain's plan (exits with a message if the domain cannot be planned)
    DomainPlan planned() const {
        std::optional<DomainPlan> plan;
        try {
            plan.emplace(DomainPlanner::plan(domain_, lattice_memory()));
        } catch(const SetupError& error) {
            print_error(error.what()); // waits for Enter (Windows) and exits; nothing may follow it (C4702 with /GL)
        }
        return *plan;
    }

    /// Checks that the model's file and the files it needs() are in resources/; otherwise prints its instructions()
    /// and exits with a message.
    void validate_geometry_file() const {
        const Model& model = *domain_.model();
        string missing;
        if(get_resource_path(model.file()).empty()) missing = model.file();
        for(const std::string& file : model.required_files()) {
            if(get_resource_path(file).empty()) missing += (missing.empty() ? "" : ", ") + file;
        }
        if(missing.empty()) return;
        for(const std::string& line : model.instruction_lines()) print_info(line);
        string message = "geometry file not found: " + missing + "; searched in";
#ifdef FLUIDX3D_RESOURCE_DIR
        message += " " + string(FLUIDX3D_RESOURCE_DIR) + "/ and";
#endif // FLUIDX3D_RESOURCE_DIR
        message += " " + get_exe_path() + "resources/";
        print_error(message);
    }

    /// @brief Exits with a message if the LBM already exists.
    /// @param what the setter that must come before the first use
    void before_first_use(const char* what) const {
        if(lbm_) print_error(string("Simulation: call ") + what + " before the simulation's first use (a builder, the graphics or a run)");
    }

    /// @param a an acceleration
    /// @return the acceleration in lattice units
    float3 lattice(const AccelerationVector& a) const {
        return float3(scale_.acceleration(a.x), scale_.acceleration(a.y), scale_.acceleration(a.z));
    }

    /// Creates the LBM with the physics set so far, voxelizes the model and creates the owned components.
    void create() {
        const float3 f = lattice(body_force_);
#ifndef VOLUME_FORCE
        if(f.x != 0.0f || f.y != 0.0f || f.z != 0.0f) print_error("Simulation: gravity or a body force needs VOLUME_FORCE in the EXTENSIONS");
#endif // VOLUME_FORCE
#ifndef SURFACE
        if(surface_tension_.si() != 0.0f) print_error("Simulation: a surface tension needs SURFACE in the EXTENSIONS");
#endif // SURFACE
#ifdef TEMPERATURE
        if(!temperature_scale_) print_error("Simulation: a thermal simulation (TEMPERATURE) needs set_temperatures()");
#else
        if(temperature_scale_) print_error("Simulation: set_temperatures() needs TEMPERATURE in the EXTENSIONS");
#endif // TEMPERATURE
#ifndef PARTICLES
        if(particle_count_ > 0u) print_error("Simulation: set_particles() needs PARTICLES in the EXTENSIONS");
#endif // PARTICLES
        const float32_t alpha = temperature_scale_ ? scale_.viscosity(fluid_.thermal_diffusivity) : 0.0f; // same conversion as the viscosity
        const float32_t beta = temperature_scale_ ? temperature_scale_->expansion(fluid_.thermal_expansion) : 0.0f;
        lbm_ = std::make_unique<LBM>(plan_.Nx, plan_.Ny, plan_.Nz, plan_.gpus.x, plan_.gpus.y, plan_.gpus.z,
                                     scale_.viscosity(fluid_.kinematic_viscosity), f.x, f.y, f.z,
                                     scale_.surface_tension(surface_tension_), alpha, beta, particle_count_, particle_density_);
        if(domain_.model() && voxelize_model_) voxelize();
        runner_ = std::make_unique<Runner>(*lbm_, scale_);
        parts_ = std::make_unique<MovingPartsManager>(*lbm_, scale_, plan_, *runner_);
#ifdef SURFACE
        wave_maker_ = std::make_unique<WaveBoundary>(*lbm_, scale_, *runner_);
#endif // SURFACE
#ifdef GRAPHICS
        video_ = std::make_unique<VideoRecorder>(*lbm_, scale_);
#endif // GRAPHICS
#ifdef FORCE_FIELD
        forces_ = std::make_unique<ForceAnalyzer>(*lbm_, scale_);
#endif // FORCE_FIELD
#ifdef PARTICLES
        particles_ = std::make_unique<ParticleManager>(*lbm_, scale_);
#endif // PARTICLES
    }

    /// Voxelizes the domain's model as planned: a mirrored half model, an SDF or an STL.
    void voxelize() {
        const uchar flag = measure_forces_ ? (TYPE_S | TYPE_X) : TYPE_S;
        if(plan_.mirror) { // symmetric half model
            const std::unique_ptr<Mesh> mesh = MeshLoader::load_mirrored(plan_, *plan_.mirror);
            lbm_->voxelize_mesh_on_device(mesh.get(), flag);
            return;
        }
        if(plan_.voxelize_sdf) {
            lbm_->voxelize_sdf(plan_.voxelization_path, plan_.center_lbm, plan_.rotation_matrix, plan_.voxel_size, flag);
        } else {
            lbm_->voxelize_stl(plan_.voxelization_path, plan_.center_lbm, plan_.rotation_matrix, plan_.voxel_size, flag);
        }
    }

    /// @return the runner, creating the LBM if needed
    Runner& runner() {
        lbm();
        return *runner_;
    }

    /// @brief Schedules the video's frames on the first run, when built with GRAPHICS but not INTERACTIVE_GRAPHICS.
    /// @param time the simulated time of the run
    void start_video(Duration time) {
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
    /// @return whether a solid is measured: the model with measure_forces(), or a Solid::MEASURED shape on the host's
    /// flags
    bool measures_a_solid() {
        if(measure_forces_) return true;
        for(uint64_t n = 0ull; n < lbm_->get_N(); n++) {
            if(lbm_->flags[n] & TYPE_X) return true;
        }
        return false;
    }
#endif // FORCE_FIELD
};
