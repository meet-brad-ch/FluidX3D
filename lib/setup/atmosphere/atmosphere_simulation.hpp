#pragma once

/// @file atmosphere_simulation.hpp
/// @brief An atmosphere's simulation (AtmosphereSimulation): the hub of the middle layer (L2). Header-side: it needs
/// the build's defines and the core.

#include "setup/atmosphere/atmosphere_grid.hpp"
#include "setup/atmosphere/atmosphere_start_builder.hpp"
#include "setup/atmosphere/cbl_description.hpp"
#include "setup/atmosphere/heat_source_builder.hpp"
#include "setup/atmosphere/interval.hpp"
#include "setup/atmosphere/level_forcing.hpp"
#include "setup/core/scalar_transport_rules.hpp"
#include "setup/core/transport_build.hpp"
#include "setup/simulation/simulation.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"

#ifdef MOISTURE
#include "setup/atmosphere/moist_air_component.hpp"
#endif // MOISTURE

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <vector>

#if !defined(VOLUME_FORCE) || !defined(TEMPERATURE) || !defined(LEVEL_FORCING)
#error "setup/atmosphere/atmosphere_simulation.hpp needs VOLUME_FORCE, TEMPERATURE and LEVEL_FORCING in the EXTENSIONS"
#endif // VOLUME_FORCE, TEMPERATURE, LEVEL_FORCING

/// @brief An atmosphere's box over a heated floor: a Simulation in square cells with a solid floor and lid, the
/// Boussinesq buoyancy about theta_0, the lattice numbers of CblNumerics, and the components a run needs (the
/// start, the heat sources, the per-level forcing, the moist air). The middle layer: each part usable on its own.
///
/// @code
/// using namespace si::symbols;
/// AtmosphereSimulation atm(grid, sounding.reference(), CblNumerics{}, std::nullopt);
/// atm.start().sounding(sounding).noise(StartNoise::per_cell(0.1 * K, 300.0 * m, 2u)).apply();
/// atm.heat_sources().surface_flux(0.24 * K * m / s).apply();
/// atm.level_forcing().add_term(std::make_unique<SpongeTerm>(SpongeProfile::none(), sounding));
/// atm.level_forcing().update(nullptr, std::nullopt);
/// @endcode
class AtmosphereSimulation {
public:
    /// @brief Makes the atmosphere and creates the LBM: a box of the grid's cells with the levels kept whole in each
    /// domain, air with the numerics' viscosity and scalar diffusivity, the temperature scale theta_0 and the kernel's
    /// temperature unit, the Boussinesq buoyancy and the turbulent Prandtl number.
    ///
    /// Ends the program with print_error() when the Simulation refuses the box or one of the settings (see Simulation).
    /// @param grid the box's cells and levels
    /// @param theta_0 the reference potential temperature: the lattice temperature 1 and the buoyancy's theta_0
    /// @param numerics the time step rule, the diffusivities, Pr_t, gravity and the buoyancy's carrier
    /// @param domains the domains along x, y and z (none: the planner's choice; never along z)
    AtmosphereSimulation(const AtmosphereGrid& grid, si::Temperature theta_0, const CblNumerics& numerics, std::optional<uint3> domains)
        : grid_(grid), numerics_(numerics), sim_(domain_of(grid, domains), Fluid::AIR, numerics.time_step), converter_(sim_.converter().with_theta_0(theta_0)) {
        sim_.set_viscosity(numerics.momentum);
        const TransportBuild build = current_build();
        const Diffusivity scalar = numerics.scalar_for(build);
        sim_.set_thermal_diffusivity(scalar);
        sim_.set_temperature_scale(theta_0, si::LatticeConverter::kernel_temperature_unit);
        const BoussinesqBuoyancy buoyancy(numerics.gravity, theta_0, numerics.carrier);
        sim_.set_boussinesq_buoyancy(buoyancy);
        const float inverse_prandtl = numerics.prandtl.lattice_inverse();
        sim_.lbm().set_inv_prandtl_t(inverse_prandtl); // creates the LBM
    }

    AtmosphereSimulation(const AtmosphereSimulation&) = delete;            ///< not copyable: it owns the LBM
    AtmosphereSimulation& operator=(const AtmosphereSimulation&) = delete; ///< not copyable: it owns the LBM

    /// @brief Gives a builder of the start state (use it before the first run).
    /// @return the builder, which refers to this atmosphere's LBM
    AtmosphereStartBuilder start() { return AtmosphereStartBuilder(sim_.lbm(), grid_, converter_); }
    /// @brief Gives a builder of the floor's heat sources (use it after the start).
    /// @return the builder; its summary is kept here for the fingerprint
    HeatSourceBuilder heat_sources() { return HeatSourceBuilder(sim_.lbm(), grid_, converter_, &sources_); }
    /// @brief Gives the per-level forcing, the only writer of the core's level arrays; it is made on the first call.
    /// @return the level forcing, owned by this atmosphere
    LevelForcing& level_forcing() {
        if(!level_forcing_) {
            level_forcing_ = std::make_unique<LevelForcing>(sim_.lbm(), grid_, converter_);
        }
        return *level_forcing_;
    }
#ifdef MOISTURE
    /// @brief Makes the moist air of the run and hands its terms to the core (MoistAir::apply()).
    /// @param column the moist column of the start sounding (each level's pressure and theta_v)
    /// @param flux the floor's moisture flux
    /// @return the moist air, owned by this atmosphere
    MoistAir& moisture(MoistColumn column, si::MoistureFlux flux) {
        moist_ = std::make_unique<MoistAir>(sim_.lbm(), std::move(column), flux, converter_);
        moist_->apply();
        return *moist_;
    }
    /// @return the moist air, or none before moisture()
    const MoistAir* moist_air() const { return moist_.get(); }
#endif // MOISTURE

    /// @brief Adds a check that judges each interval and the end of the run.
    ///
    /// Ends the program with print_error() for a null check.
    /// @param check the check, owned from here on
    /// @return this atmosphere
    AtmosphereSimulation& add_check(std::unique_ptr<RunCheck> check) {
        if(!check) {
            print_error("AtmosphereSimulation::add_check(): no check");
        }
        checks_.push_back(std::move(check));
        return *this;
    }
    /// @brief Adds an observer that reads each interval and the end of the run.
    /// @param observer the observer, not owned: it must outlive run()
    /// @return this atmosphere
    AtmosphereSimulation& observe(IntervalObserver& observer) {
        observers_.push_back(&observer);
        return *this;
    }

    /// @brief Runs the spin-up and the recorded time. After each interval, in this order: the fields to the host, the
    /// layer means, the inversion height (and the mixed layer's top while recording), the observers, the level forcing
    /// (the nudging), the PROGRESS line, the checks. A check's FAILED line stops the program (exit 1). Once per
    /// atmosphere.
    /// @param schedule the spin-up, the recorded time and the interval
    /// @param inversion zi at the start (the inversion height before the convection reaches it)
    /// @param sponge_from the sponge's start (the flux minimum is searched below it), or none
    /// @param floor_flux the floor's kinematic heat flux, more than 0 (the scale of an entrainment flux for the
    /// inversion height)
    /// @return what the run did
    /// @throws SetupError when the floor's heat flux is not more than 0 (BoundaryLayerDiagnostics)
    RunSummary run(const RunSchedule& schedule, si::Length inversion, std::optional<si::Length> sponge_from, si::KinematicHeatFlux floor_flux) {
        if(ran_) {
            print_error("AtmosphereSimulation::run(): an atmosphere runs once");
        }
        ran_ = true;
        const std::uint64_t spin = converter_.steps(schedule.spinup);
        const std::uint64_t record = converter_.steps(schedule.record);
        const std::uint64_t total = spin + record;
        const IntervalPlan plan{ .spin = spin,
                                 .total = total,
                                 .diagnostics = BoundaryLayerDiagnostics(grid_.levels(), inversion, sponge_from, floor_flux),
                                 .statistics = LevelStatistics() };
        RunSummary summary{};
        const Clock clock{};
        // the interval's time from its step count (the core's float step), not from the Runner's float time
        Runner::Task task = [&](si::Duration) { interval(plan, clock, summary); };
        sim_.every(schedule.interval, std::move(task), Runner::FirstCall::AFTER_ONE_INTERVAL);
        const si::Duration run_time = converter_.time(total);
        sim_.run_for(run_time);
        finish(summary);
        return summary;
    }

    /// @brief An FNV-1a hash of the set-up: the flags, rho, u, T (and Q) of every cell, the level arrays, the levels'
    /// pressures and the scalars given to the core. Two set-ups with the same hash give the same run (the CBL
    /// example's FINGERPRINT line).
    /// @return the hash
    std::uint64_t fingerprint() {
        Fnv1a h{};
        hash_cells(h);
        if(level_forcing_) {
            const LevelArrays& a = level_forcing_->lattice_levels();
            const std::size_t rate_bytes = a.rate.size() * sizeof(float);
            h.add(a.rate.data(), rate_bytes);
            const std::size_t t_ref_bytes = a.T_ref.size() * sizeof(float);
            h.add(a.T_ref.data(), t_ref_bytes);
        }
        float moisture_rate{ 0.0f };
#ifdef MOISTURE
        if(moist_) {
            const MoistColumn& column = moist_->column();
            for(std::uint32_t z = 0u; z < column.levels(); z++) {
                const si::Pressure pressure = column.pressure(z);
                const double p = pressure.in(au::pascals);
                h.add(&p, sizeof(p));
            }
            moisture_rate = moist_->moisture_rate();
        }
#endif // MOISTURE
        const BoussinesqBuoyancy buoyancy(numerics_.gravity, converter_.theta_0(), numerics_.carrier);
        const float momentum = numerics_.momentum.lattice_value(converter_);
        const TransportBuild build = current_build();
        const Diffusivity scalar = numerics_.scalar_for(build);
        const float scalar_diffusivity = scalar.lattice_value(converter_);
        const float alpha = ScalarTransportRules::core_alpha(scalar_diffusivity, build);
        const float expansion = buoyancy.lattice_expansion(converter_);
        const float force = buoyancy.lattice_force();
        const float heat_rate = sources_.heat_rate;
        const float inverse_prandtl = numerics_.prandtl.lattice_inverse();
        const double theta_0_kelvin = si::in_kelvin(converter_.theta_0());
        const float theta_0 = static_cast<float>(theta_0_kelvin);
        const float scalars[] = { momentum, alpha, expansion, force, heat_rate, inverse_prandtl, theta_0, moisture_rate };
        h.add(scalars, sizeof(scalars));
        const std::uint64_t hash = h.value();
        return hash;
    }

    /// @return the box's cells and levels
    const AtmosphereGrid& grid() const { return grid_; }
    /// @return the run's conversions to the lattice
    const si::LatticeConverter& converter() const { return converter_; }
    /// @return the heat sources' summary (zero before heat_sources().apply())
    const SourceSummary& sources() const { return sources_; }
    /// @return the simulation (running, tasks, estimate)
    Simulation& simulation() { return sim_; }
    /// @brief For expert use: the core. A direct write to state a component owns (the level arrays, the moist terms)
    /// is overwritten at that component's next update.
    /// @return the core
    LBM& lbm() { return sim_.lbm(); }

private:
    /// @brief The 64-bit FNV-1a hash of a byte sequence.
    class Fnv1a {
    public:
        /// @brief Adds bytes to the hash, one after the other.
        /// @param p the first byte
        /// @param n the number of bytes
        void add(const void* p, std::size_t n) {
            const unsigned char* b = static_cast<const unsigned char*>(p);
            for(std::size_t i = 0u; i < n; i++) {
                const unsigned char byte_value = b[i];
                h_ ^= byte_value;
                h_ *= prime;
            }
        }
        /// @return the hash so far
        std::uint64_t value() const { return h_; }

    private:
        static constexpr std::uint64_t offset = 14695981039346656037ull; ///< FNV-1a's offset basis
        static constexpr std::uint64_t prime = 1099511628211ull;         ///< FNV-1a's prime
        std::uint64_t h_ = offset;                                       ///< the hash
    };

    /// @brief Adds every cell's flags, rho, u, T (and Q in a MOISTURE build) to a hash, in the core's cell order.
    /// @param h the hash
    void hash_cells(Fnv1a& h) {
        LBM& lbm = sim_.lbm();
        const ulong cells = lbm.get_N();
        for(ulong n = 0u; n < cells; n++) {
            const uchar f = lbm.flags[n];
            const float rho = lbm.rho[n];
            const float ux = lbm.u.x[n];
            const float uy = lbm.u.y[n];
            const float uz = lbm.u.z[n];
            const float t = lbm.T[n];
            const float values[] = { rho, ux, uy, uz, t };
            h.add(&f, sizeof(f));
            h.add(values, sizeof(values));
#ifdef MOISTURE
            const float q = lbm.Q[n];
            h.add(&q, sizeof(q));
#endif // MOISTURE
        }
    }

    /// @brief What every interval of one run needs: the step counts, the heights' rules, the layer statistics.
    struct IntervalPlan {
        std::uint64_t spin{};                  ///< the spin-up's steps
        std::uint64_t total{};                 ///< all steps
        BoundaryLayerDiagnostics diagnostics;  ///< zi and the mixed layer's top
        LevelStatistics statistics{};          ///< the layer means
    };

    /// @brief The smallest and the largest layer w extreme of an interval.
    struct WRange {
        si::Speed min{ au::ZERO }; ///< the smallest w over every level, the floor's and the lid's zeros too (the example's); not a number when a level's is not finite
        si::Speed max{ au::ZERO }; ///< the largest w over every level, the same way
    };

    /// @brief One interval, in the order of run(): the fields, the layer means, the heights, the observers, the level
    /// forcing, the PROGRESS line, the checks.
    /// @param plan the run's plan
    /// @param clock the run's wall clock
    /// @param summary the run's counts
    void interval(const IntervalPlan& plan, const Clock& clock, RunSummary& summary) {
        LBM& lbm = sim_.lbm();
        const std::uint64_t step = lbm.get_t();
        const si::Duration time = converter_.time(step);
        const bool recording = step > plan.spin;
        const bool moist = moist_air_column() != nullptr;
        const AtmosphereFields fields(lbm, grid_, converter_, moist);
        const LevelProfiles profiles = profiles_of(plan.statistics, fields);
        const si::Length zi = plan.diagnostics.inversion_height(profiles.means);
        std::optional<si::Length> top{};
        if(recording) {
            const si::Length mixed_top = mixed_layer_top(plan.diagnostics, profiles);
            top = mixed_top;
        }
        const WRange w = w_range(profiles);
        const IntervalSnapshot snapshot{ .time = time,
                                         .step = step,
                                         .recording = recording,
                                         .fields = fields,
                                         .profiles = profiles,
                                         .inversion = zi,
                                         .mixed_layer_top = top,
                                         .w_min = w.min,
                                         .w_max = w.max };
        for(IntervalObserver* o : observers_) {
            o->on_interval(snapshot);
        }
        if(level_forcing_) {
            level_forcing_->update(&profiles.means, zi);
        }
        const double wall_seconds = clock.stop();
        const si::Duration wall = au::seconds(wall_seconds);
        print_progress(time, step, plan.total, zi, w, wall);
        check(snapshot);
        summary.intervals++;
        if(recording) {
            summary.recorded++;
        }
    }

    /// @brief Gives the interval's level profiles from its fields.
    /// @param statistics the layer statistics
    /// @param fields the interval's fields
    /// @return the layer means, and the cloud levels in a moist run
    LevelProfiles profiles_of(const LevelStatistics& statistics, const AtmosphereFields& fields) const {
#ifdef MOISTURE
        const MoistColumn* column = moist_air_column();
        LevelProfiles profiles = statistics.compute(fields, column);
#else
        LevelProfiles profiles = statistics.compute(fields);
#endif // MOISTURE
        return profiles;
    }

    /// @brief Gives the mixed layer's top of the interval.
    ///
    /// Ends the program with print_error() and the reason when the profile has no mixed layer top (DiagnosticsError).
    /// @param diagnostics the heights' rules
    /// @param profiles the layer means
    /// @return the mixed layer's top
    static si::Length mixed_layer_top(const BoundaryLayerDiagnostics& diagnostics, const LevelProfiles& profiles) {
        try {
            const si::Length top = diagnostics.mixed_layer_top(profiles.means);
            return top;
        } catch(const DiagnosticsError& error) {
            print_error(error.what());
        }
        return au::meters(0.0); // not reached: print_error() ends the program
    }

    /// @brief Gives the w extremes of an interval over every level, the floor's and the lid's zeros too (the
    /// example's). A level whose extremes are not finite gives them unchanged, so that the blow-up check sees them (a
    /// max or a min over the levels would drop a NaN at the next finite level).
    /// @param profiles the layer means
    /// @return the smallest and the largest w extreme
    static WRange w_range(const LevelProfiles& profiles) {
        WRange w{};
        for(const LayerMeans& m : profiles.means) {
            const bool finite_max{ !au::isnan(m.w_max) && !au::isinf(m.w_max) };
            const bool finite_min{ !au::isnan(m.w_min) && !au::isinf(m.w_min) };
            if(!finite_max || !finite_min) {
                const WRange blown{ .min = m.w_min, .max = m.w_max };
                return blown;
            }
            w.max = au::max(w.max, m.w_max);
            w.min = au::min(w.min, m.w_min);
        }
        return w;
    }

    /// @brief Prints the PROGRESS line of an interval on stdout: the time, the steps, the share done, zi, the w
    /// extremes, the wall time and the speed in MLUPs.
    /// @param time the simulated time at the interval's end
    /// @param step the time step at the interval's end
    /// @param total the time steps of the whole run
    /// @param zi the inversion height
    /// @param w the w extremes
    /// @param wall the wall time since the run started
    void print_progress(si::Duration time, std::uint64_t step, std::uint64_t total, si::Length zi, const WRange& w, si::Duration wall) {
        constexpr double percent = 100.0;                 // per cent per ratio
        const double cells{ static_cast<double>(sim_.lbm().get_N()) };
        const double steps_done{ static_cast<double>(step) };
        const double updates{ cells * steps_done };
        const si::LatticeWorkAmount work = si::lattice_updates(updates);
        const auto rate = work / wall;
        const double mlups = rate.in(au::mega(si::lattice_updates) / au::second);
        const double time_s = time.in(au::seconds);
        const unsigned long long step_count{ static_cast<unsigned long long>(step) };
        const unsigned long long total_count{ static_cast<unsigned long long>(total) };
        const double scaled_step{ percent * steps_done };
        const double steps_total{ static_cast<double>(total) };
        const double done_percent{ scaled_step / steps_total };
        const double zi_m = zi.in(au::meters);
        const double w_min = w.min.in(au::meters / au::second);
        const double w_max = w.max.in(au::meters / au::second);
        const double wall_s = wall.in(au::seconds);
        std::printf("PROGRESS t %.0f s (%llu/%llu steps, %.1f %%) zi %.0f m w %.2f..%.2f m/s wall %.0f s, %.0f MLUPs\n", time_s,
                    step_count, total_count, done_percent, zi_m, w_min, w_max, wall_s, mlups);
        std::fflush(stdout);
    }

    /// @brief The checks of one interval: a FAILED line stops the program (exit 1).
    /// @param snapshot the interval
    void check(const IntervalSnapshot& snapshot) {
        for(const std::unique_ptr<RunCheck>& c : checks_) {
            const std::optional<std::string> failure = c->on_interval(snapshot);
            if(failure) {
                const char* line = failure->c_str();
                std::printf("%s\n", line);
                std::fflush(stdout);
                std::exit(1);
            }
        }
    }

    /// @brief The end: the checks' RESULT lines into the summary, then the observers.
    /// @param summary the run's summary
    void finish(RunSummary& summary) {
        for(const std::unique_ptr<RunCheck>& c : checks_) {
            const std::optional<CheckResult> r = c->on_finish();
            if(r) {
                const char* line = r->line.c_str();
                std::printf("%s\n", line);
                summary.results.push_back(*r);
                const bool passed = summary.passed && r->passed;
                summary.passed = passed;
            }
        }
        std::fflush(stdout);
        for(IntervalObserver* o : observers_) {
            o->on_finish(summary);
        }
    }

#ifdef MOISTURE
    /// @brief Gives the moist column of the run.
    /// @return the column, or null for a dry run
    const MoistColumn* moist_air_column() const {
        const MoistColumn* column{};
        if(moist_) {
            column = &moist_->column();
        }
        return column;
    }
#else
    /// @return none: no MOISTURE
    const void* moist_air_column() const { return nullptr; }
#endif // MOISTURE

    /// @brief Gives the Domain of a box: nx dx by ny dx by nz dx, the cell size dx, the levels kept whole in each
    /// domain.
    /// @param grid the box's cells and levels
    /// @param domains the domains along x, y and z, or none for the planner's choice
    /// @return the domain
    static Domain domain_of(const AtmosphereGrid& grid, std::optional<uint3> domains) {
        const si::Length dx = grid.cell();
        const double nx{ static_cast<double>(grid.nx()) };
        const double ny{ static_cast<double>(grid.ny()) };
        const double nz{ static_cast<double>(grid.levels().levels()) };
        const si::Length width{ dx * nx };
        const si::Length depth{ dx * ny };
        const si::Length height{ dx * nz };
        Domain d = Domain::box(width, depth, height);
        d.cell_size(dx);
        d.keep_levels_whole();
        if(domains) {
            d.gpus(domains->x, domains->y, domains->z);
        }
        return d;
    }

    AtmosphereGrid grid_;                            ///< the box's cells and levels
    CblNumerics numerics_;                           ///< the method's numbers
    Simulation sim_;                                 ///< owns the LBM and the Runner
    si::LatticeConverter converter_;                 ///< the conversions (the simulation's cell and step, theta_0)
    SourceSummary sources_{};                        ///< the heat sources' summary
    std::unique_ptr<LevelForcing> level_forcing_{};  ///< made on the first level_forcing()
    std::vector<std::unique_ptr<RunCheck>> checks_{}; ///< the run's checks, in order
    std::vector<IntervalObserver*> observers_{};     ///< not owned
    bool ran_ = false;                               ///< run() was called
#ifdef MOISTURE
    std::unique_ptr<MoistAir> moist_{};              ///< made by moisture()
#endif // MOISTURE
};
