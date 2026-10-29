#pragma once

#include "setup/domain/level_grid.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"

#include <optional>
#include <string>
#include <vector>

/// @file sponge_profile.hpp
/// @brief The damping layer under the lid: SpongeProfile.

/// @brief The damping layer under the lid (Rayleigh damping; the CBL example's sponge): from a height up, the
/// vertical speed w is damped and the temperature T is relaxed to the start profile, so that gravity waves do not
/// reflect from the lid.
///
/// The rate per time step at a height h is dt / tau f², with f = (h - from) / (lid - from): 0 below the layer's
/// start, rising as the square of the height into the layer to dt / tau at the lid. SpongeTerm hands the rates to
/// the core (LBM::set_level_forcing()).
/// @code
/// using namespace si::symbols;
/// const SpongeProfile sponge = SpongeProfile::quadratic(3000.0 * m, 450.0 * s);
/// @endcode
class SpongeProfile {
public:
    /// @return no sponge: the rate 0 at every level
    static SpongeProfile none() { return SpongeProfile(); }
    /// @brief Makes a sponge whose rate rises as the square of the height into the layer.
    /// @param from the layer's lowest height above the floor's top, more than 0
    /// @param tau_at_lid the damping time scale at the lid, more than 0 (the CBL example's 450 s)
    /// @return the sponge
    /// @throws SetupError when the height or the time scale is not more than 0
    static SpongeProfile quadratic(si::Length from, si::Duration tau_at_lid);

    /// @return true when the sponge damps (made by quadratic()), false for none()
    bool active() const { return layer_.has_value(); }
    /// @brief Gives the layer's lowest height.
    /// @return the height above the floor's top, or none without a sponge
    std::optional<si::Length> start() const {
        std::optional<si::Length> from{};
        if(layer_) {
            from = layer_->from;
        }
        return from;
    }
    /// @brief Gives the damping time scale at the lid.
    /// @return the time scale, or none without a sponge
    std::optional<si::Duration> tau() const {
        std::optional<si::Duration> time_scale{};
        if(layer_) {
            time_scale = layer_->tau;
        }
        return time_scale;
    }
    /// @brief Gives the damping rate per time step at one height, for the core.
    /// @param h a height above the floor's top
    /// @param lid the lid's height: the centre of the top level (LevelGrid::lid_height())
    /// @param converter the run's conversions (the time step dt)
    /// @return dt / tau f² (a ratio per time step); 0 below the layer's start and without a sponge. Above the lid f
    /// is more than 1: the rate is not clamped.
    /// @throws SetupError when the sponge is active and the lid is not above the layer's start
    float lattice_rate(si::Length h, si::Length lid, const si::LatticeConverter& converter) const;
    /// @brief Gives the damping rate per time step at every level, for LBM::set_level_forcing().
    /// @param grid the column's levels (the lid's height is the centre of the top level)
    /// @param converter the run's conversions (the time step dt)
    /// @return one rate per level (index z, from 0 at the floor cell)
    /// @throws SetupError when the sponge is active and the lid is not above the layer's start
    std::vector<float> lattice_rates(const LevelGrid& grid, const si::LatticeConverter& converter) const;
    /// @brief Describes the sponge for the run's log.
    /// @return "no sponge", or the layer's start and the time scale at the lid
    /// @throws SetupError when the text does not fit its buffer
    std::string describe() const;

private:
    /// @brief The two values of the quadratic layer.
    struct Layer {
        si::Length from{ au::ZERO }; ///< the layer's lowest height above the floor's top
        si::Duration tau{ au::ZERO }; ///< the damping time scale at the lid
    };
    /// @brief Makes no sponge. Use none() or quadratic().
    SpongeProfile() = default;
    std::optional<Layer> layer_{}; ///< the layer, or none for no sponge
};
