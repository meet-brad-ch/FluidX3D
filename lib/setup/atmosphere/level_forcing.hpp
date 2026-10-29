#pragma once

/// @file level_forcing.hpp
/// @brief The middle layer's (L2) per-level forcing of an atmosphere (LEVEL_FORCING): the sponge, the wind nudging
/// and user terms. Header-side: it needs the build's defines and the core (lbm.hpp).

#include "setup/atmosphere/atmosphere_grid.hpp"
#include "setup/atmosphere/boundary_layer_diagnostics.hpp"
#include "setup/atmosphere/sounding.hpp"
#include "setup/atmosphere/sponge_profile.hpp"
#include "setup/atmosphere/wind_nudging.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/core/transport_build.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"

#include "lbm.hpp"

#include <memory>
#include <optional>
#include <vector>

#ifndef LEVEL_FORCING
#error "setup/atmosphere/level_forcing.hpp needs LEVEL_FORCING in the EXTENSIONS"
#endif // LEVEL_FORCING

/// @brief The four per-level arrays that the core takes (LBM::set_level_forcing()), each indexed by the level z from
/// 0 (the floor) to levels - 1 (the lid): the horizontal forces, the relaxation rate of w and T, and the temperature
/// that T relaxes to.
struct LevelArrays {
    std::vector<float> fx{};    ///< the force along x per level, in lattice units (cells per time step squared)
    std::vector<float> fy{};    ///< the force along y per level, in lattice units
    std::vector<float> rate{};  ///< the relaxation rate of w and T per time step (a ratio; 0: no relaxation)
    std::vector<float> T_ref{}; ///< the lattice temperature that T relaxes to at the rate
};

/// @brief What a term reads when it contributes: the run's conversions, its levels, and the last interval's layer
/// means and inversion height (none at the start, before the first interval). Valid only during the contribution.
struct LevelContext {
    const si::LatticeConverter& converter;     ///< the run's conversions
    const AtmosphereGrid& grid;                ///< the box's cells and levels
    const std::vector<LayerMeans>* means{};    ///< the last interval's layer means (indexed by z), or nullptr at the start
    std::optional<si::Length> inversion{};     ///< the last interval's inversion height zi, or none at the start
};

/// @brief The interface of one contribution to the per-level forcing (a strategy). LevelForcing owns its terms and
/// calls them in the order they were added. At most one term owns the relaxation (rate and T_ref).
class LevelTerm {
public:
    virtual ~LevelTerm() = default;
    /// @brief Adds the term's part to the arrays of one update.
    /// @param arrays the arrays to add to; each update starts them with the forces 0, the rate 0 and T_ref 1
    /// @param context the conversions, the levels and the last interval's means
    virtual void contribute(LevelArrays& arrays, const LevelContext& context) = 0;
    /// @brief Tells whether the term writes rate and T_ref (they can have one owner only).
    /// @return true when the term owns the relaxation
    virtual bool owns_relaxation() const = 0;
};

/// @brief The term of the damping layer under the lid and the temperature that it relaxes to: the SpongeProfile's
/// rate per level, and T_ref from the start sounding at every level (the CBL example's set-up). Without a sponge the
/// rate is 0 at every level. It owns the relaxation.
class SpongeTerm final : public LevelTerm {
public:
    /// @brief Makes the term.
    /// @param sponge the damping layer (SpongeProfile::none(): the rate 0)
    /// @param reference the sounding that T relaxes to (the start sounding)
    SpongeTerm(SpongeProfile sponge, Sounding reference) : sponge_(sponge), reference_(std::move(reference)) {}

    /// @brief Writes the sponge's rate and the sounding's lattice temperature at every level.
    /// @param arrays the arrays of the update
    /// @param context the conversions and the levels
    /// @throws SetupError when the sponge is active and the lid is not above its start (SpongeProfile::lattice_rates()),
    /// or the reference is a tabulated sounding without theta_0 (Sounding::theta_excess())
    void contribute(LevelArrays& arrays, const LevelContext& context) override {
        const LevelGrid& levels = context.grid.levels();
        const std::vector<float> rates = sponge_.lattice_rates(levels, context.converter);
        for(std::uint32_t z = 0u; z < levels.levels(); z++) {
            const float rate = rates[z];
            arrays.rate[z] = rate;
            const si::Length height = levels.height(z);
            const si::TemperatureDifference excess = reference_.theta_excess(height);
            const float t_ref = context.converter.temperature_excess(excess); // the example's 1 + start_theta(h)
            arrays.T_ref[z] = t_ref;
        }
    }
    /// @return true: the sponge writes rate and T_ref
    bool owns_relaxation() const override { return true; }

private:
    SpongeProfile sponge_;          ///< the damping layer under the lid
    Sounding reference_;            ///< the sounding that T relaxes to
};

/// @brief The term of the wind nudging: it pushes the layer-mean wind toward a target with the forces of WindNudging
/// from the last layer means (no force before the first interval, when the start wind is the target). It keeps the
/// last deviation below zi for the WIND line (WindHoldReport).
class NudgingTerm final : public LevelTerm {
public:
    /// @brief Makes the term.
    /// @param nudging the target wind and the nudging's time scale
    explicit NudgingTerm(WindNudging nudging) : nudging_(std::move(nudging)) {}

    /// @brief Adds the nudging forces to fx and fy, and records the deviation below zi when the inversion is known.
    /// Before the first interval (no means) it adds nothing.
    /// @param arrays the arrays of the update
    /// @param context the conversions, the levels and the last interval's means and zi
    void contribute(LevelArrays& arrays, const LevelContext& context) override {
        if(context.means == nullptr) {
            return; // the start: no means yet, no force (the start wind is the target)
        }
        const LevelGrid& levels = context.grid.levels();
        const WindNudging::Forces f = nudging_.forces(*context.means, levels, context.converter);
        for(std::size_t z = 0u; z < f.fx.size(); z++) {
            arrays.fx[z] += f.fx[z];
            arrays.fy[z] += f.fy[z];
        }
        if(context.inversion) {
            deviation_ = nudging_.deviation(*context.means, levels, *context.inversion);
        }
    }
    /// @return false: the nudging writes only the forces
    bool owns_relaxation() const override { return false; }

    /// @return the last largest deviation of the layer-mean wind below zi from the target, over the target's
    /// reference speed (a ratio; 0 before the first interval)
    double deviation() const { return deviation_; }
    /// @return the target wind and the nudging's time scale
    const WindNudging& nudging() const { return nudging_; }

private:
    WindNudging nudging_;   ///< the target wind and the nudging's time scale
    double deviation_ = 0.0; ///< the last deviation below zi over the reference speed (a ratio)
};

/// @brief The only writer of the core's per-level forcing: it owns the terms and the arrays, and sends the arrays to
/// the core once per update.
///
/// Middle layer (L2). AtmosphereSimulation owns it for the run's lifetime and calls update() after each interval.
/// The first update(), before the run, is the caller's: ConvectiveBoundaryLayer makes it after adding the terms. It writes the core's level buffer (fx, fy, rate, T_ref per level) to the device at each update. A
/// direct call of LBM::set_level_forcing() is overwritten at the next update; add a LevelTerm instead.
class LevelForcing {
public:
    /// @brief Makes the forcing without terms.
    /// @param lbm the core; it must outlive the forcing
    /// @param grid the box's cells and levels
    /// @param converter the run's conversions
    LevelForcing(LBM& lbm, const AtmosphereGrid& grid, const si::LatticeConverter& converter) : lbm_(lbm), grid_(grid), converter_(converter) {}

    /// @brief Adds a term after the terms already added (the terms contribute in this order).
    /// @param term the contribution; the forcing takes ownership
    /// @return the term, for its own readings (a NudgingTerm's deviation); valid while the forcing lives
    /// @throws SetupError when term is empty, or a second term owns the relaxation
    LevelTerm& add_term(std::unique_ptr<LevelTerm> term) {
        if(!term) {
            throw SetupError("LevelForcing::add_term(): no term");
        }
        const bool owns = term->owns_relaxation();
        if(owns && relaxation_owned()) {
            throw SetupError("LevelForcing: two terms own the relaxation (rate, T_ref)");
        }
        terms_.push_back(std::move(term));
        return *terms_.back();
    }

    /// @brief Fills the arrays from the terms and sends them to the core (LBM::set_level_forcing()).
    ///
    /// The core ends the program with print_error() when its box is split along z.
    /// @param means the last interval's layer means (indexed by z), or nullptr at the start
    /// @param inversion the last interval's inversion height zi, or none at the start
    /// @throws SetupError from a term's contribute() (see SpongeTerm)
    void update(const std::vector<LayerMeans>* means, std::optional<si::Length> inversion) {
        const std::uint32_t nz = grid_.levels().levels();
        std::vector<float> fx(nz, 0.0f);
        std::vector<float> fy(nz, 0.0f);
        std::vector<float> rate(nz, 0.0f);
        std::vector<float> t_ref(nz, 1.0f);
        arrays_ = LevelArrays{ .fx = std::move(fx), .fy = std::move(fy), .rate = std::move(rate), .T_ref = std::move(t_ref) };
        const LevelContext context{ .converter = converter_, .grid = grid_, .means = means, .inversion = inversion };
        for(const std::unique_ptr<LevelTerm>& t : terms_) {
            t->contribute(arrays_, context);
        }
        lbm_.set_level_forcing(arrays_.fx, arrays_.fy, arrays_.rate, arrays_.T_ref);
    }

    /// @return the arrays of the last update (read-only: the low layer's (L1) view)
    const LevelArrays& lattice_levels() const { return arrays_; }

private:
    /// @brief Tells whether a term already added owns the relaxation.
    /// @return true when one of the terms owns rate and T_ref
    bool relaxation_owned() const {
        for(const std::unique_ptr<LevelTerm>& t : terms_) {
            if(t->owns_relaxation()) {
                return true;
            }
        }
        return false;
    }

    LBM& lbm_;                                   ///< the core, whose level buffer update() writes
    AtmosphereGrid grid_;                        ///< the box's cells and levels
    si::LatticeConverter converter_;             ///< the run's conversions
    std::vector<std::unique_ptr<LevelTerm>> terms_{}; ///< the contributions, in the order of add_term()
    LevelArrays arrays_{};                       ///< the arrays of the last update
};
