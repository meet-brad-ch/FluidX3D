#pragma once

#include "setup/atmosphere/moist_column.hpp"
#include "setup/domain/level_grid.hpp"
#include "setup/units/units.hpp"

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

/// @file boundary_layer_diagnostics.hpp
/// @brief The horizontal layer means of an atmosphere's fields, and the boundary layer's heights from them.
///
/// Terms: theta is the potential temperature; theta_0 is the reference potential temperature (the lattice
/// temperature 1, Sounding::reference()); zi is the inversion height, the top of the convective boundary layer;
/// w'θ' is the resolved kinematic heat flux, the layer mean of w' θ' with w' and θ' the deviations from the layer
/// means.

/// @brief The error of a diagnostic that has no answer for the fields it was given (no level in its window, no cell,
/// no inversion). It is not a set-up error: the run's state, not the description, is the cause.
class DiagnosticsError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/// @brief The horizontal means of one layer of cells (the CBL example's LayerMeans). LayerMoments::means() makes it.
struct LayerMeans {
    si::TemperatureDifference theta{ au::ZERO };   ///< the layer mean of theta - theta_0
    si::SpeedVariance w_var{ au::ZERO };           ///< the variance of the vertical speed, mean(w'²)
    double w_skew = 0.0;                           ///< the skewness of the vertical speed, mean(w'³) / mean(w'²)^(3/2) (a ratio; 0 without variance)
    si::TemperatureVariance theta_var{ au::ZERO }; ///< the variance of theta, mean(θ'²)
    si::KinematicHeatFlux w_theta{ au::ZERO };     ///< the resolved heat flux w'θ', mean(w' θ')
    si::Speed u{ au::ZERO };                       ///< the layer mean of the speed toward +x (east)
    si::Speed v{ au::ZERO };                       ///< the layer mean of the speed toward +y (north)
    si::Speed w_max{ au::ZERO };                   ///< the largest vertical speed in the layer; not a number when a cell's w is not finite
    si::Speed w_min{ au::ZERO };                   ///< the smallest vertical speed in the layer (most negative); not a number when a cell's w is not finite
};

/// @brief Sums the cells of one layer, in the order they are added, and gives the layer's means from the sums.
///
/// The sums are the raw moments, as the CBL example sums them, so the means are the example's bit for bit. A variance
/// from raw moments, mean(x²) - mean(x)², loses digits where the variance is small against the mean's square (the
/// raw-moment floor, measured on the example's captures).
/// @code
/// using namespace si::symbols;
/// LayerMoments layer{};
/// layer.add(0.1 * K, 0.5 * m / s, 2.0 * m / s, 0.0 * m / s); // once per cell of the layer
/// const LayerMeans means = layer.means(1.0);
/// @endcode
class LayerMoments {
public:
    /// @brief Adds one cell to the sums.
    /// @param theta the cell's theta - theta_0
    /// @param w the cell's vertical speed (up is positive)
    /// @param u the cell's speed toward +x (east)
    /// @param v the cell's speed toward +y (north)
    void add(si::TemperatureDifference theta, si::Speed w, si::Speed u, si::Speed v);
    /// @brief Gives the layer's means: the sums over the number of cells, and the central moments from the raw ones.
    /// @param cells the divisor, more than 0: the layer's cells (nx ny for a whole layer)
    /// @return the means; the extremes are over the cells added, and both are not a number when a cell's w was not
    /// finite (so that a blow-up check sees it)
    /// @throws DiagnosticsError when no cell was added or cells is not more than 0
    LayerMeans means(double cells) const;

private:
    /// The extremes before the first cell (the example's start values: 1e9 m/s).
    static constexpr si::Speed start_extreme{ (au::meters / au::second)(1E9) };

    si::TemperatureDifference t_{ au::ZERO };  ///< the sum of theta - theta_0
    si::TemperatureVariance tt_{ au::ZERO };   ///< the sum of (theta - theta_0)²
    si::Speed w_{ au::ZERO };                  ///< the sum of w
    si::SpeedVariance ww_{ au::ZERO };         ///< the sum of w²
    si::SpeedCubed www_{ au::ZERO };           ///< the sum of w³
    si::KinematicHeatFlux wt_{ au::ZERO };     ///< the sum of w (theta - theta_0)
    si::Speed u_{ au::ZERO };                  ///< the sum of u
    si::Speed v_{ au::ZERO };                  ///< the sum of v
    si::Speed w_max_ = -start_extreme; ///< the largest w so far
    si::Speed w_min_ = start_extreme;  ///< the smallest w so far
    std::size_t added_ = 0u;                   ///< the number of cells added
    bool non_finite_w_ = false;                ///< true when a cell's w was not a number or infinite
};

/// @brief The convective boundary layer's heights from the layer means: the inversion height zi from the heat flux,
/// the mixed layer's top by the MicroHH reference runs' rule, and the condensation level of the mixed layer's air.
///
/// The means are one LayerMeans per level of the column (index z, the floor and the lid included), as
/// AtmosphereFields and LevelStatistics give them.
class BoundaryLayerDiagnostics {
public:
    /// The inversion is searched above this part of the column's height (a ratio): the surface layer's negative
    /// fluxes are not the entrainment flux.
    static constexpr double inversion_search_floor = 0.1;
    /// A level's resolved heat flux counts as entrainment only below this part of the floor's heat flux (a negative
    /// ratio): before the convection reaches the inversion, the most negative flux is rounding noise (about
    /// -1e-13 K m/s), and an entrainment flux is about -0.2 of the floor's.
    static constexpr double entrainment_floor = -1.0E-3;
    /// The bottom of the height window of the mixed layer's mean theta (MicroHH's rule).
    static constexpr si::Length mixed_window_bottom = au::meters(100.0);
    /// The top of the height window of the mixed layer's mean theta (MicroHH's rule).
    static constexpr si::Length mixed_window_top = au::meters(600.0);
    /// The excess over the mixed layer's mean theta that marks the mixed layer's top (MicroHH's rule).
    static constexpr si::TemperatureDifference mixed_top_excess = au::kelvins(0.5);
    /// The bottom of the window of the condensation level's mixed-layer air, as a ratio of zi.
    static constexpr double condensation_window_bottom = 0.25;
    /// The top of the window of the condensation level's mixed-layer air, as a ratio of zi.
    static constexpr double condensation_window_top = 0.75;

    /// @brief Makes the diagnostics of a column.
    /// @param grid the column's levels
    /// @param start_zi the inversion height at the start: inversion_height()'s answer before the convection reaches
    /// the inversion
    /// @param sponge_from the height of the damping layer's start, or none: the inversion is searched below it
    /// @param floor_flux the floor's kinematic heat flux, more than 0 (the scale of an entrainment flux: see
    /// entrainment_floor; zi is the inversion of a heated floor)
    /// @throws SetupError when the floor's heat flux is not more than 0
    BoundaryLayerDiagnostics(const LevelGrid& grid, si::Length start_zi, std::optional<si::Length> sponge_from, si::KinematicHeatFlux floor_flux);

    /// @brief Gives zi: the height of the most negative resolved heat flux w'θ' (the entrainment flux).
    ///
    /// The search runs over the levels z = 2 to levels - 3 (counted from 0 at the bottom cell, whatever the floor's
    /// and the lid's thickness), above inversion_search_floor of the top level's height and below the sponge. A flux counts only below entrainment_floor times the floor's
    /// flux. Before there is such a flux (the start, before the convection reaches the inversion), the answer is the
    /// start's zi.
    /// @param means the layer means, one per level (index z), at least as many as the column's levels minus 2
    /// @return zi, a level's centre height above the floor's top
    /// @throws std::out_of_range from std::vector::at() when means has too few levels
    si::Length inversion_height(const std::vector<LayerMeans>& means) const;

    /// @brief Gives the mixed layer's top: the lowest level above 100 m whose mean theta exceeds the mixed layer's
    /// mean theta (the mean between 100 m and 600 m) by more than 0.5 K (the MicroHH reference runs' rule).
    /// @param means the layer means, one per level (index z), one for each of the column's levels
    /// @return the level's centre height above the floor's top
    /// @throws DiagnosticsError when no level is between 100 m and 600 m, or no level below the lid is 0.5 K above
    /// the mixed layer
    /// @throws std::out_of_range from std::vector::at() when means has too few levels
    si::Length mixed_layer_top(const std::vector<LayerMeans>& means) const;

    /// @brief Gives the condensation level of the mixed layer's air: the mean theta_l and qt between 0.25 zi and
    /// 0.75 zi, lifted at constant theta_l and qt, at the lowest fluid level where the saturation adjustment at that
    /// level's pressure gives liquid water.
    /// @param means the layer means, one per level (index z); their theta is theta_l - theta_0
    /// @param total_water the layer mean of the total water qt, one per level (index z)
    /// @param column the moist column, for each level's pressure
    /// @param theta_0 the reference potential temperature of the means
    /// @param zi the mixed layer's depth for the window (the case's zi)
    /// @return the level's centre height above the floor's top, or none when the window holds no level or no fluid
    /// level condenses
    /// @throws std::out_of_range from std::vector::at() when means, total_water or the column has too few levels
    std::optional<si::Length> condensation_level(const std::vector<LayerMeans>& means, const std::vector<si::WaterContent>& total_water,
                                                 const MoistColumn& column, si::Temperature theta_0, si::Length zi) const;

private:
    /// @brief Gives the mixed layer's mean theta: the mean of the fluid levels' theta between 100 m and 600 m.
    /// @param means the layer means, one per level (index z)
    /// @return the mean theta - theta_0
    /// @throws DiagnosticsError when no fluid level is between 100 m and 600 m
    /// @throws std::out_of_range from std::vector::at() when means has too few levels
    si::TemperatureDifference mixed_layer_theta(const std::vector<LayerMeans>& means) const;

    /// @brief The mixed layer's air for the condensation level: the means over the window's levels.
    struct WindowAir {
        si::TemperatureDifference thl_excess{}; ///< the sum of the levels' theta_l - theta_0
        si::WaterContent qt{};                  ///< the sum of the levels' total water
        double levels{};                        ///< the number of levels in the window (0 for none)
    };

    /// @brief Sums theta_l and qt over the fluid levels between 0.25 zi and 0.75 zi.
    /// @param means the layer means, one per level (index z)
    /// @param total_water the layer mean of the total water qt, one per level (index z)
    /// @param zi the mixed layer's depth for the window
    /// @return the sums and the number of levels
    /// @throws std::out_of_range from std::vector::at() when means or total_water has too few levels
    WindowAir window_air(const std::vector<LayerMeans>& means, const std::vector<si::WaterContent>& total_water, si::Length zi) const;

    /// @brief Gives the lowest fluid level where air of one theta_l and qt holds liquid water.
    /// @param column the moist column, for each level's pressure
    /// @param thl the air's liquid water potential temperature
    /// @param qt the air's total water
    /// @return the level's centre height above the floor's top, or none when no fluid level condenses
    /// @throws std::out_of_range from MoistColumn::liquid() when the column has too few levels
    std::optional<si::Length> lowest_liquid_level(const MoistColumn& column, si::Temperature thl, si::WaterContent qt) const;

    LevelGrid grid_;                         ///< the column's levels
    si::Length start_zi_{ au::ZERO };         ///< the inversion height at the start
    std::optional<si::Length> sponge_from_{}; ///< the height of the damping layer's start, or none
    si::KinematicHeatFlux entrainment_threshold_{ au::ZERO }; ///< a flux counts as entrainment only below it (entrainment_floor times the floor's flux, negative)
};
