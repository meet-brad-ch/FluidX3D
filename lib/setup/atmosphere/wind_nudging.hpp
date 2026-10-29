#pragma once

#include "setup/atmosphere/boundary_layer_diagnostics.hpp"
#include "setup/atmosphere/wind_target.hpp"
#include "setup/domain/level_grid.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"

#include <memory>
#include <vector>

/// @file wind_nudging.hpp
/// @brief The per-level forces that hold a column's mean wind to a target: WindNudging.

/// @brief The per-level horizontal forces that hold a column's layer-mean wind to a target wind (the CBL example's
/// nudge_wind()).
///
/// After each interval, every fluid level gets the acceleration (target - mean) / tau from the layer means of the
/// interval just run, so the mean wind relaxes to the target over the time scale tau. NudgingTerm hands the forces
/// to the core (LBM::set_level_forcing()).
class WindNudging {
public:
    /// @brief The forces of one interval, for the core: one value per level (index z, from 0 at the floor cell).
    struct Forces {
        std::vector<float> fx{}; ///< the lattice force along x per level: the change of the lattice speed per time step (0 on the floor and the lid)
        std::vector<float> fy{}; ///< the lattice force along y per level (0 on the floor and the lid)
    };

    /// @brief Makes the nudging toward a target.
    /// @param target the wind to hold, not null (shared: the start builder uses the same target)
    /// @param tau the nudging's time scale, more than 0 (the CBL example's 600 s)
    /// @throws SetupError when the target is null or the time scale is not more than 0
    WindNudging(std::shared_ptr<const WindTarget> target, si::Duration tau);

    /// @brief Gives the forces of one interval: (target - mean) dt / tau at each fluid level, as a lattice speed
    /// change per time step.
    /// @param means the layer means, one per level (index z)
    /// @param grid the column's levels (the floor and the lid get 0)
    /// @param converter the run's conversions (the time step and the cell)
    /// @return the forces, for LBM::set_level_forcing()
    /// @throws SetupError when means has fewer than grid.last_fluid() + 1 entries (one per level from 0 at the floor
    /// cell to the last fluid level)
    Forces forces(const std::vector<LayerMeans>& means, const LevelGrid& grid, const si::LatticeConverter& converter) const;

    /// @brief Tells how far the layer-mean wind is from the target below zi. This is information only: with a long
    /// tau the nudged wind does not reach the target.
    /// @param means the layer means, one per level (index z)
    /// @param grid the column's levels
    /// @param zi the mixed layer's depth: the fluid levels below it count
    /// @return the largest length of (mean wind - target wind) below zi, over the target's reference speed (a
    /// ratio); 0 when no fluid level is below zi
    /// @throws SetupError when means has fewer than grid.last_fluid() + 1 entries (one per level from 0 at the floor
    /// cell to the last fluid level)
    double deviation(const std::vector<LayerMeans>& means, const LevelGrid& grid, si::Length zi) const;

    /// @return the wind that the nudging holds
    const WindTarget& target() const { return *target_; }
    /// @return the nudging's time scale
    si::Duration tau() const { return tau_; }

private:
    /// @brief Checks that the layer means reach the last fluid level.
    /// @param means the layer means, one per level (index z)
    /// @param grid the column's levels
    /// @throws SetupError when means has fewer than grid.last_fluid() + 1 entries
    static void check_means(const std::vector<LayerMeans>& means, const LevelGrid& grid);

    std::shared_ptr<const WindTarget> target_{}; ///< the wind to hold, not null
    si::Duration tau_{ au::ZERO };               ///< the nudging's time scale
};
