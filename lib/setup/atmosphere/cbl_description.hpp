#pragma once

#include "setup/atmosphere/atmosphere_grid.hpp"
#include "setup/atmosphere/heated_patch.hpp"
#include "setup/atmosphere/sounding.hpp"
#include "setup/atmosphere/sponge_profile.hpp"
#include "setup/atmosphere/start_noise.hpp"
#include "setup/atmosphere/wind_target.hpp"
#include "setup/core/boussinesq_buoyancy.hpp"
#include "setup/core/diffusivity.hpp"
#include "setup/core/scalar_transport_rules.hpp"
#include "setup/core/time_step_rule.hpp"
#include "setup/core/turbulent_prandtl.hpp"
#include "setup/domain/run_estimate.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/physical_constants.hpp"
#include "setup/units/units.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

/// @file cbl_description.hpp
/// @brief The high-level (L3) description of a convective boundary layer run: CblDescription and its parts.

/// @brief The rule that sets the lid's height: a fixed height, or a multiple of the inversion height zi.
///
/// Heights are above the floor's top (the solid floor's upper face, LevelGrid). A multiple of zi keeps the lid at the
/// same place relative to the boundary layer when zi changes (the tabulated sounding case: 2 zi).
/// @code
/// using namespace si::symbols;
/// const LidRule fixed = LidRule::at(2048.0 * m);
/// const LidRule relative = LidRule::in_inversion_heights(2.0);
/// const si::Length lid = relative.height(1000.0 * m); // 2000 m
/// @endcode
class LidRule {
public:
    /// @brief Makes a rule that puts the lid at a fixed height.
    /// @param ground_to_lid the lid's height above the floor's top, more than 0
    /// @return the rule
    /// @throws SetupError when the height is not more than 0
    static LidRule at(si::Length ground_to_lid);
    /// @brief Makes a rule that puts the lid at a multiple of zi.
    /// @param k the lid's height over zi (a ratio), more than 1 so that the lid is above the inversion
    /// @return the rule
    /// @throws SetupError when k is not more than 1
    static LidRule in_inversion_heights(double k);
    /// @brief Gives the lid's height for an inversion height.
    /// @param zi the inversion height at the start (used only by a rule from in_inversion_heights())
    /// @return the fixed height, or k times zi
    si::Length height(si::Length zi) const;
    /// @brief Describes the rule for the run's log.
    /// @return "the lid at <height> m" or "the lid at <k> zi"
    /// @throws SetupError when the text does not fit its buffer
    std::string describe() const;

private:
    /// @brief Makes the rule. Use at() or in_inversion_heights(), which check the value.
    /// @param height the fixed height, or none for a multiple of zi
    /// @param k the lid's height over zi (a ratio), used when height is none
    LidRule(std::optional<si::Length> height, double k) : height_(height), k_(k) {}
    std::optional<si::Length> height_{}; ///< the lid's fixed height above the floor's top, or none for a multiple of zi
    double k_{};                         ///< the lid's height over zi (a ratio), used when height_ is none
};

/// @brief The size of the box: its extent west to east and south to north, and the rule for its lid.
///
/// AtmosphereGrid::of() rounds each extent to whole cells (lround(extent / cell)). The defaults are Sullivan and
/// Patton 2011's box: 1280 m square, the lid at 2048 m.
struct ColumnExtent {
    static constexpr si::Length sp2011_side = au::meters(1280.0); ///< Sullivan and Patton 2011's extent, the same along x and y
    static constexpr si::Length sp2011_lid = au::meters(2048.0);  ///< Sullivan and Patton 2011's lid height

    si::Length east = sp2011_side;                 ///< the box's extent west to east
    si::Length north = sp2011_side;                ///< the box's extent south to north
    LidRule lid = LidRule::at(sp2011_lid);         ///< the rule for the lid's height above the floor's top
};

/// @brief The heating (and, in a moist run, the evaporation) at the floor.
///
/// The floor heats the air above it with a uniform kinematic heat flux. A heated patch adds stronger heating in one
/// round area, which anchors a thermal. In a moist run, the floor also gives off water: the moisture flux follows
/// from the heat flux and the Bowen ratio (moisture_flux()). The default heat flux is Sullivan and Patton 2011's
/// 0.24 K m/s, without a patch.
struct SurfaceForcing {
    static constexpr si::KinematicHeatFlux sp2011_flux = (au::kelvins * au::meters / au::second)(0.24); ///< Sullivan and Patton 2011's floor heat flux

    si::KinematicHeatFlux flux = sp2011_flux;                                   ///< the floor's kinematic heat flux w'θ' (the sensible heat flux over ρ cp)
    HeatedPatch patch = HeatedPatch::none();                                    ///< the round patch of stronger heating, or HeatedPatch::none()
    double bowen = 0.5;                                                         ///< the Bowen ratio B (sensible over latent heat flux, a ratio, more than 0), used only by a moist run

    /// @brief Gives the floor's moisture flux of a moist run from the heat flux and the Bowen ratio:
    /// E = flux cp / (Lv B), the rule of the MicroHH reference runs.
    /// @return the kinematic moisture flux (kg of water per kg of air, times m/s)
    /// @throws SetupError when the Bowen ratio is not more than 0
    si::MoistureFlux moisture_flux() const;
};

/// @brief A wind that is held during the run: each level's mean wind is nudged to a power law target.
///
/// The nudging adds, at each level, a force that moves the layer-mean wind toward the target over the time scale
/// nudge_tau (NudgingTerm). The target also gives the start wind.
struct WindForcing {
    static constexpr si::Duration microhh_nudge_tau = au::seconds(600.0); ///< the nudging time scale of the MicroHH reference runs (--nudge 600)

    PowerLawWind wind;                                 ///< the target wind profile and its direction
    si::Duration nudge_tau = microhh_nudge_tau;        ///< the nudging's time scale
};

/// @brief The numerical choices of the method: the time step rule, the diffusivities, the turbulent Prandtl number,
/// gravity and the buoyancy's carrier force.
///
/// The momentum and scalar diffusivities default to small lattice values (LES regularisations that scale with the
/// grid, D = value dx²/dt). The subgrid model (SUBGRID) carries the turbulent mixing. The run states these values in
/// its log.
struct CblNumerics {
    static constexpr si::Duration reference_step = au::seconds(0.04); ///< the time step at the reference cell (dx/dt 62.5 m/s)
    static constexpr si::Length reference_cell = au::meters(2.5);     ///< the cell of the reference time step
    static constexpr double default_viscosity = 1.0E-5;               ///< the default momentum diffusivity, in lattice units (dx²/dt)

    TimeStepRule time_step = TimeStepRule::constant_lattice_speed(reference_step, reference_cell); ///< the time step rule; the default keeps dx/dt at 62.5 m/s at every cell size
    Diffusivity momentum = Diffusivity::lattice(si::LatticeDiffusivity(default_viscosity)); ///< the fluid's viscosity (small: the Smagorinsky subgrid model carries the turbulence)
    std::optional<Diffusivity> scalar{};                                      ///< the diffusivity of heat and water, or none for the build's floor value (see scalar_for())
    TurbulentPrandtl prandtl = TurbulentPrandtl::one_third();                 ///< the turbulent Prandtl number Pr_t of the subgrid heat and water mixing
    si::Acceleration gravity = si::PhysicalConstants::gravity;                ///< the gravitational acceleration g
    double carrier = BoussinesqBuoyancy::default_carrier;                     ///< the buoyancy's carrier force fz in lattice units (see BoussinesqBuoyancy)

    /// @brief Gives the scalar diffusivity that the run uses: scalar when it is set, otherwise the floor value of the
    /// build's transport.
    /// @param build the build's transports (from the EXTENSIONS)
    /// @return scalar, or the floor: the lattice value 1e-4 with the finite volumes (SCALAR_FV), 0.014 with the D3Q7
    /// lattice (the smallest value that is safe next to walls in a buoyant flow)
    Diffusivity scalar_for(const TransportBuild& build) const;
};

/// @brief The run's times: the spin-up without records, then the recorded time, with one record per interval.
///
/// Both times must be whole intervals (CblDescription::validate()). The defaults: 1200 s of spin-up, 1800 s of
/// records, one record every 20 s.
struct RunSchedule {
    static constexpr si::Duration default_spinup = au::seconds(1200.0);  ///< the default spin-up
    static constexpr si::Duration default_record = au::seconds(1800.0);  ///< the default recorded time
    static constexpr si::Duration default_interval = au::seconds(20.0);  ///< the default time between two records

    si::Duration spinup = default_spinup;      ///< the time before the first record
    si::Duration record = default_record;      ///< the recorded time after the spin-up
    si::Duration interval = default_interval;  ///< the time between two records (and two rows of the profiles)
};

class CblDescription;

/// @brief Reads a CblDescription part by part, without knowing its layout (a header writer, a log).
///
/// CblDescription::visit() calls each function once, in the order of the declarations below. A new part of the
/// description adds a function here, so that every visitor must handle it.
class DescriptionVisitor {
public:
    virtual ~DescriptionVisitor() = default;
    /// @brief Receives the start state.
    /// @param sounding the start profile of the potential temperature (and of the total water in a moist run)
    /// @param inversion the inversion height zi at the start
    virtual void sounding(const Sounding& sounding, si::Length inversion) = 0;
    /// @brief Receives the box and its cells.
    /// @param extent the box in metres and its lid rule
    /// @param grid the cells that the extent and the cell size give
    virtual void column(const ColumnExtent& extent, const AtmosphereGrid& grid) = 0;
    /// @brief Receives the floor's heating.
    /// @param surface the heat flux, the patch and the Bowen ratio
    virtual void surface(const SurfaceForcing& surface) = 0;
    /// @brief Receives the held wind.
    /// @param wind the target and the nudging time scale, or none without wind
    virtual void wind(const std::optional<WindForcing>& wind) = 0;
    /// @brief Receives the damping layer under the lid.
    /// @param sponge the sponge (SpongeProfile::none() without a sponge)
    virtual void sponge(const SpongeProfile& sponge) = 0;
    /// @brief Receives the numerical choices.
    /// @param numerics the method's choices
    /// @param step the time step that the time step rule gives at the description's cell size
    virtual void numerics(const CblNumerics& numerics, si::Duration step) = 0;
    /// @brief Receives the run's times.
    /// @param schedule the spin-up, the recorded time and the interval
    virtual void schedule(const RunSchedule& schedule) = 0;
    /// @brief Receives the start noise.
    /// @param noise the random perturbation of the start temperature
    virtual void noise(const StartNoise& noise) = 0;
    /// @brief Receives whether the run is moist.
    /// @param moist true when the run carries water (moist buoyancy and clouds)
    virtual void moisture(bool moist) = 0;
};

/// @brief A convective boundary layer run in physical units: what to simulate, not how the lattice holds it.
///
/// This is the high-level (L3) entry of the atmosphere API. ConvectiveBoundaryLayer takes a description, converts it
/// to lattice units, builds the LBM and runs it. The description is a value: each with_*() function returns a
/// changed copy and leaves the original as it was.
///
/// The default is Sullivan and Patton 2011's dry convective boundary layer (J. Atmos. Sci. 68) at 10 m cells: a
/// floor heat flux of 0.24 K m/s, 300 K up to zi = 974 m, +8 K over the next 100 m, 3 K/km above, a 1280 m square
/// box, the lid at 2048 m, and per-cell start noise of 0.1 K below 300 m.
///
/// Each with_*() function checks only its own value. validate() checks the parts together (for example the lid
/// above zi, the patch inside the box). ConvectiveBoundaryLayer calls validate() before the build.
///
/// @code
/// using namespace si::symbols;
/// const CblDescription cbl = CblDescription()
///     .with_sounding(Sounding::from_csv(path).with_reference_from_mixed_layer(1864.0 * m), 1864.0 * m)
///     .with_extent({ 1920.0 * m, 1920.0 * m, LidRule::in_inversion_heights(2.0) })
///     .with_surface({ 0.262 * K * m / s, HeatedPatch::disc(400.0 * m, 0.0 * m, PatchMode::SURFACE_LAYER), 0.5 })
///     .with_moisture(true);
/// cbl.validate();
/// @endcode
class CblDescription {
public:
    /// @brief Makes the default description: Sullivan and Patton 2011's dry convective boundary layer at 10 m cells.
    CblDescription();

    /// @brief Replaces the start state.
    /// @param sounding the start profile. A tabulated sounding needs its theta_0 before validate()
    /// (Sounding::with_reference_from_mixed_layer()).
    /// @param inversion the inversion height zi at the start, more than 0. The boundary layer diagnostics start from
    /// it, and LidRule::in_inversion_heights() and the cell size check (at most zi / 10) use it.
    /// @return a copy with the sounding and zi
    /// @throws SetupError when the inversion height is not more than 0
    CblDescription with_sounding(Sounding sounding, si::Length inversion) const;
    /// @brief Replaces the cell size. The box and the lid stay in metres, so the cell count changes.
    /// @param cell the edge of one cubic cell, more than 0 and a float exactly (the core holds it as a float; 2.5 m
    /// and 10 m are, 2.3 m is not). The float check happens in converter() and the step counts.
    /// @return a copy with the cell size
    /// @throws SetupError when the cell size is not more than 0
    CblDescription with_cell(si::Length cell) const;
    /// @brief Replaces the box's size and its lid rule. Checked by validate().
    /// @param extent the box's extent west to east and south to north, and the lid rule
    /// @return a copy with the extent
    CblDescription with_extent(ColumnExtent extent) const;
    /// @brief Replaces the floor's heating. Checked by validate() (the patch inside the box).
    /// @param surface the heat flux, the patch and the Bowen ratio
    /// @return a copy with the forcing
    CblDescription with_surface(SurfaceForcing surface) const;
    /// @brief Replaces the held wind.
    /// @param wind the target and the nudging time scale, or none for no wind
    /// @return a copy with the wind
    CblDescription with_wind(std::optional<WindForcing> wind) const;
    /// @brief Replaces the damping layer under the lid. Checked by validate() (the sponge must start below the lid).
    /// @param sponge the sponge, or SpongeProfile::none()
    /// @return a copy with the sponge
    CblDescription with_sponge(SpongeProfile sponge) const;
    /// @brief Replaces the numerical choices.
    /// @param numerics the time step rule, the diffusivities, Pr_t, gravity and the carrier
    /// @return a copy with the choices
    CblDescription with_numerics(CblNumerics numerics) const;
    /// @brief Replaces the run's times. Checked by validate() (whole intervals).
    /// @param schedule the spin-up, the recorded time and the interval
    /// @return a copy with the times
    CblDescription with_schedule(RunSchedule schedule) const;
    /// @brief Replaces the start noise.
    /// @param noise the random perturbation of the start temperature, or StartNoise::none()
    /// @return a copy with the noise
    CblDescription with_noise(StartNoise noise) const;
    /// @brief Makes the run moist or dry. Checked by validate() (a moist run needs total water in the sounding) and
    /// by ConvectiveBoundaryLayer (a moist run needs MOISTURE in the build).
    /// @param moist true for a moist run: total water, a floor moisture flux, moist buoyancy and clouds
    /// @return a copy with the setting
    CblDescription with_moisture(bool moist) const;
    /// @brief Sets how the box is split over devices. Checked by validate() (no split along z).
    /// @param domains the number of domains along x, y and z (z must be 1), or none to let the domain planner choose
    /// (the planner splits only along x or y)
    /// @return a copy with the domains
    CblDescription with_domains(std::optional<uint3> domains) const;

    /// @brief Checks the description as a whole. Each with_*() function checked only its own value.
    /// @throws SetupError that names the first fault, checked in this order: a tabulated sounding without theta_0; a
    /// cell size that is not a float exactly; an interval shorter than half a time step; a spin-up or recorded time
    /// that is not a whole number of intervals; a box less than one cell across; a cell size larger than zi / 10; the
    /// lid not above zi; a sponge that does not start below the lid; a patch that is not inside the box; a floor heat
    /// flux, a nudging time scale, a gravity or a carrier that is not more than 0; a start noise block that the cell
    /// size does not divide; a moist run without total water in the sounding; a split along z
    void validate() const;

    /// @brief Gives the cells of the box: the extents and the lid height in whole cells (rounded to the nearest).
    /// @return the grid: nx by ny cells across, and the levels from the floor to the lid
    /// @throws SetupError when the box is less than one cell across
    AtmosphereGrid grid() const;
    /// @brief Gives the time step that the time step rule gives at the cell size.
    /// @return the time step, before the core rounds it to a float (converter() holds the rounded step)
    si::Duration time_step() const;
    /// @brief Gives the converter between this run's physical values and the core's lattice values.
    /// @return the converter with the cell size, the time step (rounded to a float, as the core holds it) and theta_0
    /// @throws SetupError when the tabulated sounding has no theta_0, or the cell size is not a float exactly
    si::LatticeConverter converter() const;
    /// @brief Gives the number of time steps of the spin-up.
    /// @return the spin-up over the core's float time step, rounded to the nearest whole step
    /// @throws SetupError when the cell size is not a float exactly
    std::uint64_t spinup_steps() const;
    /// @brief Gives the number of time steps of the recorded time.
    /// @return the recorded time over the core's float time step, rounded to the nearest whole step
    /// @throws SetupError when the cell size is not a float exactly
    std::uint64_t record_steps() const;
    /// @brief Gives the number of time steps between two records.
    /// @return the interval over the core's float time step, rounded to the nearest whole step (0 for an interval
    /// shorter than half a step)
    /// @throws SetupError when the cell size is not a float exactly
    std::uint64_t interval_steps() const;
    /// @brief Estimates the cost of the run before any device is used: the cells, the memory per domain, the time
    /// steps and, with a measured speed, the wall time. It does not call validate().
    /// @param lattice the build's memory per cell and halo layers (Simulation::build_lattice_memory())
    /// @param speed a measured speed of this build on this device, or none for no wall time
    /// @return the estimate for the description's domains (one domain when none are set)
    /// @throws SetupError when the box is less than one cell across, a domain count is 0, the speed is not more than
    /// 0, or the cell size is not a float exactly
    RunEstimate estimate(const LatticeMemory& lattice, std::optional<si::LatticeUpdateRate> speed) const;
    /// @brief Gives each part of the description to the visitor, in the order of DescriptionVisitor's functions.
    /// @param visitor the reader of the parts
    /// @throws SetupError when the box is less than one cell across (from grid())
    void visit(DescriptionVisitor& visitor) const;

    /// @return the start profile
    const Sounding& sounding() const { return sounding_; }
    /// @return the inversion height zi at the start
    si::Length inversion() const { return inversion_; }
    /// @return the edge of one cubic cell
    si::Length cell() const { return cell_; }
    /// @return the box's extent and its lid rule
    const ColumnExtent& extent() const { return extent_; }
    /// @return the floor's heating
    const SurfaceForcing& surface() const { return surface_; }
    /// @return the held wind, or none without wind
    const std::optional<WindForcing>& wind() const { return wind_; }
    /// @return the damping layer under the lid (SpongeProfile::none() without a sponge)
    const SpongeProfile& sponge() const { return sponge_; }
    /// @return the numerical choices
    const CblNumerics& numerics() const { return numerics_; }
    /// @return the run's times
    const RunSchedule& schedule() const { return schedule_; }
    /// @return the start noise
    const StartNoise& noise() const { return noise_; }
    /// @return true when the run carries water
    bool moist() const { return moist_; }
    /// @return the number of domains along x, y and z, or none when the domain planner chooses
    const std::optional<uint3>& domains() const { return domains_; }

private:
    /// The largest cell size over zi (a ratio): the mixed layer must be at least 10 cells deep to resolve its eddies.
    static constexpr double max_cell_over_zi = 0.1;

    /// @brief Checks the run's times against the time step.
    /// @throws SetupError when the interval is shorter than half a time step, or the spin-up or the recorded time is
    /// not a whole number of intervals
    void validate_times() const;
    /// @brief Checks the vertical layout: the cell size against zi, the lid above zi, the sponge below the lid.
    /// @param g the cells of the box
    /// @throws SetupError when the cell size is larger than zi / 10, the lid is not above zi, or the sponge does not
    /// start below the lid
    void validate_column(const AtmosphereGrid& g) const;
    /// @brief Checks that the heated patch is inside the box. Without a patch there is nothing to check.
    /// @param g the cells of the box
    /// @throws SetupError when the patch's radius is larger than half the box's width, or the patch reaches past the
    /// box's south or north edge
    void validate_patch(const AtmosphereGrid& g) const;
    /// @brief Checks the values that the run divides by or scales with: the floor's heat flux, the nudging's time
    /// scale, gravity and the carrier (BoussinesqBuoyancy), and the start noise's block against the cell size
    /// (StartNoise::sequence()).
    /// @throws SetupError when the heat flux, the nudging's time scale, gravity or the carrier is not more than 0, or
    /// the cell size does not divide the noise block
    void validate_forcing() const;

    Sounding sounding_;                ///< the start profile
    si::Length inversion_{ au::ZERO }; ///< the inversion height zi at the start
    si::Length cell_{ au::ZERO };      ///< the edge of one cubic cell
    ColumnExtent extent_{};            ///< the box's extent and its lid rule
    SurfaceForcing surface_{};         ///< the floor's heating
    std::optional<WindForcing> wind_{}; ///< the held wind, or none
    SpongeProfile sponge_;             ///< the damping layer under the lid
    CblNumerics numerics_{};           ///< the numerical choices
    RunSchedule schedule_{};           ///< the run's times
    StartNoise noise_;                 ///< the start noise
    bool moist_ = false;               ///< true when the run carries water
    std::optional<uint3> domains_{};   ///< the number of domains along x, y and z, or none for the planner's choice
};
