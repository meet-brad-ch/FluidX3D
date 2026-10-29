#pragma once

/// @file convective_boundary_layer.hpp
/// @brief The high layer (L3) of the atmosphere API: a convective boundary layer run built from its description.
///
/// Header-side: it needs the build's defines (the EXTENSIONS) and the core.

#include "setup/atmosphere/atmosphere_simulation.hpp"
#include "setup/atmosphere/cbl_description.hpp"
#include "setup/atmosphere/lattice_parameters.hpp"

#include <memory>
#include <optional>

#if !defined(VOLUME_FORCE) || !defined(TEMPERATURE) || !defined(LEVEL_FORCING)
#error "setup/atmosphere/convective_boundary_layer.hpp needs VOLUME_FORCE, TEMPERATURE and LEVEL_FORCING in the EXTENSIONS"
#endif // VOLUME_FORCE, TEMPERATURE, LEVEL_FORCING

/// @brief Builds and runs a convective boundary layer from a CblDescription.
///
/// The high layer (L3): the user gives physical values only. The constructor checks the description, then builds the
/// middle layer (an AtmosphereSimulation, which owns the Simulation and the LBM) in a fixed order: the start state
/// (sounding, noise, start wind, water), the floor's heat sources, the level forcing (the sponge, and the nudging with
/// wind), the blow-up guard, in a moist run the moist air and its cloud report, and with wind the wind report. After
/// the constructor the core holds the whole set-up; run() then runs the spin-up and the recorded time once. To change
/// one aspect, step down with atmosphere(); for raw core access use lbm().
///
/// The run owns the LBM, so it is not copyable. Observers are not owned and must outlive run().
/// @code
/// using namespace si::symbols;
/// ConvectiveBoundaryLayer run(CblDescription().with_cell(10.0 * m));
/// ProfileCsv csv(folder / "profiles.csv", run.description().moist()); // an IntervalObserver
/// run.observe(csv);
/// const RunSummary summary = run.run();
/// @endcode
class ConvectiveBoundaryLayer {
public:
    /// @brief Checks the description and builds the whole run on the device (see the class description).
    ///
    /// Ends the program with print_error() when the description is not valid (CblDescription::validate(), the fault
    /// in the message), when a moist description is built without MOISTURE in the EXTENSIONS, and when the domain
    /// planner refuses the box (for example a box that does not fit the devices' memory: Simulation catches the
    /// planner's SetupError and calls print_error()).
    /// @param description the run to build (copied)
    /// @throws SetupError for faults that validate() does not check: a nudging time scale that is not more than 0
    /// (WindNudging), a noise block that is not a whole number of cells (StartNoise::sequence(), in the start's
    /// apply()), a gravity or a buoyancy carrier that is not more than 0 (BoussinesqBuoyancy), or a moist run's Bowen
    /// ratio that is not more than 0 (SurfaceForcing::moisture_flux())
    explicit ConvectiveBoundaryLayer(const CblDescription& description)
        : description_(validated(description)),
          atmosphere_(description_.grid(), description_.sounding().reference(), description_.numerics(), description_.domains()) {
        const CblDescription& d = description_;
        std::shared_ptr<const WindTarget> wind{};
        si::Duration nudge_tau{ au::ZERO };
        const std::optional<WindForcing>& held_wind = d.wind();
        if(held_wind.has_value()) {
            const WindForcing& held = *held_wind;
            wind = std::make_shared<PowerLawWind>(held.wind);
            nudge_tau = held.nudge_tau;
        }
        AtmosphereStartBuilder start = atmosphere_.start();
        start.sounding(d.sounding());
        start.noise(d.noise());
        start.wind(wind);
        start.moisture(d.moist());
        start.apply();
        HeatSourceBuilder sources = atmosphere_.heat_sources();
        sources.surface_flux(d.surface().flux);
        sources.patch(d.surface().patch);
        sources.apply();
        LevelForcing& forcing = atmosphere_.level_forcing();
        std::unique_ptr<LevelTerm> sponge = std::make_unique<SpongeTerm>(d.sponge(), d.sounding());
        forcing.add_term(std::move(sponge));
        const NudgingTerm* nudging{};
        if(wind != nullptr) {
            nudging = add_nudging(forcing, wind, nudge_tau);
        }
        forcing.update(nullptr, std::nullopt);
        atmosphere_.add_check(std::make_unique<BlowUpGuard>());
#ifdef MOISTURE
        if(d.moist()) {
            add_moist_air();
        }
#endif // MOISTURE
        if(nudging != nullptr) {
            atmosphere_.add_check(std::make_unique<WindHoldReport>(*nudging));
        }
    }

    ConvectiveBoundaryLayer(const ConvectiveBoundaryLayer&) = delete;            ///< not copyable: it owns the LBM
    ConvectiveBoundaryLayer& operator=(const ConvectiveBoundaryLayer&) = delete; ///< not copyable: it owns the LBM

    /// @brief Adds an observer that reads each interval and the run's end (a profile writer, a capture).
    /// @param observer the observer (not owned: it must outlive run()); observers are called in the order they are added
    /// @return this run, for chained calls
    ConvectiveBoundaryLayer& observe(IntervalObserver& observer) {
        atmosphere_.observe(observer);
        return *this;
    }
    /// @brief Adds a check that judges each interval and the run's end, after the run's own checks.
    ///
    /// Ends the program with print_error() for a null check (AtmosphereSimulation::add_check()).
    /// @param check the check (owned by the run); a FAILED line from its on_interval() ends the program (exit 1), a
    /// failed on_finish() result only makes RunSummary::passed false
    /// @return this run, for chained calls
    ConvectiveBoundaryLayer& add_check(std::unique_ptr<RunCheck> check) {
        atmosphere_.add_check(std::move(check));
        return *this;
    }
    /// @brief Runs the description's spin-up and recorded time, interval by interval (AtmosphereSimulation::run()).
    ///
    /// Ends the program with print_error() on a second call (a run runs once) and when a recorded interval has no mixed
    /// layer top (the DiagnosticsError of BoundaryLayerDiagnostics::mixed_layer_top()), and with exit code 1 when a
    /// check's on_interval() gives a FAILED line. A FAILED RESULT line at the end (on_finish(), for example RESULT
    /// CLOUD FAILED) does not end the program: it makes RunSummary::passed false.
    /// @return the intervals run, the recorded ones, and the checks' results
    RunSummary run() {
        const RunSchedule& schedule = description_.schedule();
        const si::Length inversion = description_.inversion();
        const std::optional<si::Length> sponge_from = description_.sponge().start();
        const si::KinematicHeatFlux floor_flux = description_.surface().flux;
        RunSummary summary = atmosphere_.run(schedule, inversion, sponge_from, floor_flux);
        return summary;
    }

    /// @return the run's description (as checked by the constructor)
    const CblDescription& description() const { return description_; }
    /// @brief Gives the lattice numbers that the run handed to the core (the low layer's read-only view).
    /// @return the numbers for this build's transports
    LatticeParameters parameters() const {
        const TransportBuild build = current_build();
        const LatticeParameters p = LatticeParameters::of(description_, build);
        return p;
    }
    /// @return the middle layer (L2) of the run, to read or change one aspect
    AtmosphereSimulation& atmosphere() { return atmosphere_; }
    /// @brief Gives the core, for expert use. A direct write is overwritten at the next update of the component that
    /// owns that state (see AtmosphereSimulation::lbm()).
    /// @return the run's LBM
    LBM& lbm() { return atmosphere_.lbm(); }

private:
    /// @brief Checks a description before the build.
    ///
    /// Ends the program with print_error() when CblDescription::validate() throws (the fault in the message) or when
    /// a moist description has no MOISTURE in the EXTENSIONS.
    /// @param d the description
    /// @return a copy of d, unchanged
    static CblDescription validated(const CblDescription& d) {
        try {
            d.validate();
        } catch(const SetupError& error) {
            const std::string message = std::string("ConvectiveBoundaryLayer: ") + error.what();
            print_error(message);
        }
        const bool moisture_missing = d.moist() && !current_build().moisture;
        if(moisture_missing) {
            print_error("ConvectiveBoundaryLayer: a moist description needs MOISTURE in the EXTENSIONS");
        }
        return d;
    }

    /// @brief Adds the nudging of the layer-mean wind to the start wind, as a NudgingTerm after the sponge.
    /// @param forcing the atmosphere's level forcing
    /// @param wind the start wind, which is the nudging's target
    /// @param tau the nudging's time scale (WindForcing::nudge_tau)
    /// @return the added term (owned by forcing), for the wind report
    /// @throws SetupError when the nudging time scale is not more than 0 (WindNudging); add_term() does not refuse
    /// the term, because a NudgingTerm does not own the relaxation
    static const NudgingTerm* add_nudging(LevelForcing& forcing, const std::shared_ptr<const WindTarget>& wind, si::Duration tau) {
        WindNudging held(wind, tau);
        std::unique_ptr<LevelTerm> term = std::make_unique<NudgingTerm>(std::move(held));
        LevelTerm& added = forcing.add_term(std::move(term));
        const NudgingTerm* nudging = &static_cast<NudgingTerm&>(added);
        return nudging;
    }

#ifdef MOISTURE
    /// @brief Hands a moist run's moist air to the core (the hydrostatic moist column and the floor's moisture flux)
    /// and adds the cloud report.
    /// @throws SetupError when the Bowen ratio is not more than 0 (SurfaceForcing::moisture_flux()), or the sounding
    /// has no total water (MoistColumn::hydrostatic())
    void add_moist_air() {
        const CblDescription& d = description_;
        MoistColumn column = MoistColumn::hydrostatic(d.sounding(), d.grid().levels());
        const si::MoistureFlux flux = d.surface().moisture_flux();
        const MoistAir& moist = atmosphere_.moisture(std::move(column), flux);
        const si::Temperature reference = d.sounding().reference();
        const si::Length inversion = d.inversion();
        const AtmosphereGrid grid = d.grid();
        const si::KinematicHeatFlux floor_flux = d.surface().flux;
        std::unique_ptr<RunCheck> report = std::make_unique<CloudBaseReport>(moist.column(), reference, inversion, grid.levels(), floor_flux);
        atmosphere_.add_check(std::move(report));
    }
#endif // MOISTURE

    CblDescription description_;       ///< the run's description (a copy)
    AtmosphereSimulation atmosphere_;  ///< the middle layer, which owns the LBM (declared after the description, so it is destroyed first)
};
