#pragma once

/// @file atmosphere_fields.hpp
/// @brief The middle layer's (L2) read-back of an atmosphere run: the core's fields on the host for one interval,
/// and their layer means. Header-side: it needs the build's defines and the core (lbm.hpp).

#include "setup/atmosphere/atmosphere_grid.hpp"
#include "setup/atmosphere/boundary_layer_diagnostics.hpp"
#include "setup/atmosphere/run_checks.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"

#include "lbm.hpp"
#include "utilities.hpp"

#include <optional>
#include <vector>

#ifdef MOISTURE
#include "setup/atmosphere/moist_column.hpp"
#endif // MOISTURE

#ifndef TEMPERATURE
#error "setup/atmosphere/atmosphere_fields.hpp needs TEMPERATURE in the EXTENSIONS"
#endif // TEMPERATURE

/// @brief The core's fields of one interval, read back to the host, with each cell's values as quantities.
///
/// Middle layer (L2). The constructor calls LBM::update_fields(), which recomputes rho, u and T on the device from
/// the DDFs (once per time step; the DDFs are not written), and then reads u, T and, in a moist run, Q from the
/// device into the core's host buffers, which it overwrites. The accessors read those host buffers and convert
/// through the run's LatticeConverter. An AtmosphereFields lives for one interval: it refers to the LBM, which must
/// outlive it, and a later run step makes its values stale.
class AtmosphereFields {
public:
    /// @brief Reads the fields of the current time step from the device to the host.
    /// @param lbm the core, after it ran; it must outlive this object
    /// @param grid the box's cells and levels (the layout of the host buffers)
    /// @param converter the run's conversions; theta() needs its temperature scale
    /// @param moist true to read Q too (a moist run; only with MOISTURE)
    AtmosphereFields(LBM& lbm, const AtmosphereGrid& grid, const si::LatticeConverter& converter, bool moist)
        : lbm_(lbm), grid_(grid), converter_(converter), moist_(moist) {
        lbm.update_fields();
        lbm.u.read_from_device();
        lbm.T.read_from_device();
#ifdef MOISTURE
        if(moist) {
            lbm.Q.read_from_device();
        }
#endif // MOISTURE
    }

    /// @brief Gives the position of a cell in the core's buffers: x varies fastest, then y, then z.
    /// @param x the cell's index along x, 0 to nx - 1
    /// @param y the cell's index along y, 0 to ny - 1
    /// @param z the cell's level, 0 (the floor) to levels - 1 (the lid)
    /// @return the linear index x + (y + z ny) nx
    ulong index(std::uint32_t x, std::uint32_t y, std::uint32_t z) const {
        const ulong z_wide{ static_cast<ulong>(z) };
        const ulong ny_wide{ static_cast<ulong>(grid_.ny()) };
        const ulong level_rows{ z_wide * ny_wide };
        const ulong y_wide{ static_cast<ulong>(y) };
        const ulong row{ y_wide + level_rows };
        const ulong nx_wide{ static_cast<ulong>(grid_.nx()) };
        const ulong row_start{ row * nx_wide };
        const ulong x_wide{ static_cast<ulong>(x) };
        const ulong n{ x_wide + row_start };
        return n;
    }
    /// @brief Gives a cell's vertical speed.
    /// @param n the cell's linear index (index())
    /// @return the speed toward +z, up
    si::Speed w(ulong n) const {
        const double lattice_w{ static_cast<double>(lbm_.u.z[n]) };
        const si::Speed speed = converter_.si_velocity(lattice_w);
        return speed;
    }
    /// @brief Gives a cell's speed toward the east.
    /// @param n the cell's linear index (index())
    /// @return the speed toward +x
    si::Speed u(ulong n) const {
        const double lattice_u{ static_cast<double>(lbm_.u.x[n]) };
        const si::Speed speed = converter_.si_velocity(lattice_u);
        return speed;
    }
    /// @brief Gives a cell's speed toward the north.
    /// @param n the cell's linear index (index())
    /// @return the speed toward +y
    si::Speed v(ulong n) const {
        const double lattice_v{ static_cast<double>(lbm_.u.y[n]) };
        const si::Speed speed = converter_.si_velocity(lattice_v);
        return speed;
    }
    /// @brief Gives a cell's potential temperature above the reference theta_0.
    /// @param n the cell's linear index (index())
    /// @return theta - theta_0 (theta_l - theta_0 in a moist run)
    si::TemperatureDifference theta_excess(ulong n) const {
        const double lattice_temperature{ static_cast<double>(lbm_.T[n]) };
        const si::TemperatureDifference excess = converter_.si_excess(lattice_temperature);
        return excess;
    }
    /// @brief Gives a cell's potential temperature.
    /// @param n the cell's linear index (index())
    /// @return theta (theta_l, the liquid water potential temperature, in a moist run)
    /// @throws SetupError when the converter has no temperature scale (from LatticeConverter::si_temperature())
    si::Temperature theta(ulong n) const {
        const double lattice_temperature{ static_cast<double>(lbm_.T[n]) };
        const si::Temperature temperature = converter_.si_temperature(lattice_temperature);
        return temperature;
    }
    /// @brief Gives a cell's lattice temperature as the core holds it (for a file of the raw fields).
    /// @param n the cell's linear index (index())
    /// @return the lattice temperature T, unconverted
    float lattice_t(ulong n) const { return lbm_.T[n]; }
    /// @brief Gives a cell's lattice velocity along x as the core holds it (for a file of the raw fields).
    /// @param n the cell's linear index (index())
    /// @return the velocity in cells per time step, unconverted
    float lattice_ux(ulong n) const { return lbm_.u.x[n]; }
    /// @brief Gives a cell's lattice velocity along y as the core holds it (for a file of the raw fields).
    /// @param n the cell's linear index (index())
    /// @return the velocity in cells per time step, unconverted
    float lattice_uy(ulong n) const { return lbm_.u.y[n]; }
    /// @brief Gives a cell's lattice velocity along z as the core holds it (for a file of the raw fields).
    /// @param n the cell's linear index (index())
    /// @return the velocity in cells per time step, unconverted
    float lattice_uz(ulong n) const { return lbm_.u.z[n]; }
#ifdef MOISTURE
    /// @brief Gives a cell's total water content (read only when the object was made with moist true).
    /// @param n the cell's linear index (index())
    /// @return qt, the total water (vapour and liquid)
    si::WaterContent qt(ulong n) const {
        const double lattice_q{ static_cast<double>(lbm_.Q[n]) };
        const si::WaterContent water = converter_.water(lattice_q);
        return water;
    }
    /// @brief Gives a cell's lattice moisture as the core holds it (for a file of the raw fields).
    /// @param n the cell's linear index (index())
    /// @return the lattice moisture Q, unconverted
    float lattice_q(ulong n) const { return lbm_.Q[n]; }
#endif // MOISTURE
    /// @return the box's cells and levels
    const AtmosphereGrid& grid() const { return grid_; }
    /// @return the run's conversions
    const si::LatticeConverter& converter() const { return converter_; }
    /// @return true when Q was read (a moist run)
    bool moist() const { return moist_; }

private:
    LBM& lbm_;                       ///< the core, whose host buffers the constructor filled
    AtmosphereGrid grid_;            ///< the box's cells and levels
    si::LatticeConverter converter_; ///< the run's conversions
    bool moist_{};                   ///< true when Q was read
};

/// @brief The level profiles of one interval: the layer means and, in a moist run, the cloud levels.
///
/// Both vectors are indexed by the level z, 0 (the floor) to levels - 1 (the lid). The floor and the lid are solid,
/// so their entries hold zeros.
struct LevelProfiles {
    std::vector<LayerMeans> means{};     ///< the layer means of each level
    std::optional<CloudLevels> clouds{}; ///< a moist run's mean qt, ql and wet share per level; none in a dry run
};

/// @brief Computes the level profiles of an interval's fields: the CBL example's layer_means() and cloud_levels().
///
/// Middle layer (L2), without state. The fluid levels are split into contiguous ranges over
/// std::thread::hardware_concurrency() threads (the core's parallel_for()); each level is summed over y, then x, by
/// one thread, so the sums and the means do not depend on the number of threads.
///
/// An exception inside a thread is not passed to the caller: it ends the program with std::terminate(). So clouds()
/// checks, before its parallel loop, the two values that can throw inside it: the converter's temperature scale
/// (AtmosphereFields::theta()) and the column's levels (MoistColumn::liquid()); the SetupError then arrives on the
/// caller's thread. In means() only LayerMoments::means() can throw (DiagnosticsError), which a grid of at least one
/// cell never gives.
class LevelStatistics {
public:
#ifdef MOISTURE
    /// @brief Computes the layer means and, with a moist column, the cloud levels.
    ///
    /// @param fields the interval's fields
    /// @param column the moist column (the saturation adjustment at each level), or nullptr for a dry run
    /// @return the profiles; clouds is set only when column is given
    /// @throws SetupError with a moist column, for fields whose converter has no temperature scale, or a column with
    /// fewer levels than the grid (checked before the parallel loop, see the class)
    LevelProfiles compute(const AtmosphereFields& fields, const MoistColumn* column) const {
        LevelProfiles p = means(fields);
        if(column != nullptr) {
            p.clouds = clouds(fields, *column);
        }
        return p;
    }
#else
    /// @brief Computes the layer means.
    /// @param fields the interval's fields
    /// @return the profiles, without cloud levels
    LevelProfiles compute(const AtmosphereFields& fields) const {
        LevelProfiles p = means(fields);
        return p;
    }
#endif // MOISTURE

private:
#ifdef MOISTURE
    /// @brief The water of one level: the means over the level's cells.
    struct LevelWater {
        si::WaterContent qt{};  ///< the mean total water
        si::WaterContent ql{};  ///< the mean liquid water after the saturation adjustment
        double fraction{};      ///< the share of the cells that hold liquid water (a ratio)
    };
#endif // MOISTURE

    /// @brief Gives the number of cells in one level, as the divisor of the level means.
    /// @param g the box's cells
    /// @return nx ny
    static double cells_per_level(const AtmosphereGrid& g) {
        const double nx{ static_cast<double>(g.nx()) };
        const double ny{ static_cast<double>(g.ny()) };
        const double cells{ nx * ny };
        return cells;
    }

    /// @brief Computes the layer means (theta excess, w variance and skewness, w'θ', u, v, the w extremes) of each
    /// fluid level.
    /// @param fields the interval's fields
    /// @return the profiles with the means of every fluid level (zeros at the floor and the lid)
    static LevelProfiles means(const AtmosphereFields& fields) {
        const LevelGrid& levels = fields.grid().levels();
        const double cells{ cells_per_level(fields.grid()) };
        const std::uint32_t first{ levels.first_fluid() };
        LevelProfiles p{};
        p.means.assign(levels.levels(), LayerMeans{});
        parallel_for(levels.fluid_levels(), [&](uint k) {
            const std::uint32_t z{ k + first };
            const LayerMeans means_of_z{ level_means(fields, z, cells) };
            p.means[z] = means_of_z;
        });
        return p;
    }

    /// @brief Computes the layer means of one level: the sums over y, then x, divided by the cell count.
    /// @param fields the interval's fields
    /// @param z the level, from 0 at the floor cell; a fluid level
    /// @param cells the cells of one level (cells_per_level())
    /// @return the level's means
    static LayerMeans level_means(const AtmosphereFields& fields, std::uint32_t z, double cells) {
        const std::uint32_t nx{ fields.grid().nx() };
        const std::uint32_t ny{ fields.grid().ny() };
        LayerMoments layer{};
        for(std::uint32_t y = 0u; y < ny; y++) {
            for(std::uint32_t x = 0u; x < nx; x++) {
                const ulong n = fields.index(x, y, z);
                const si::TemperatureDifference theta = fields.theta_excess(n);
                const si::Speed w = fields.w(n);
                const si::Speed u = fields.u(n);
                const si::Speed v = fields.v(n);
                layer.add(theta, w, u, v);
            }
        }
        const LayerMeans level = layer.means(cells);
        return level;
    }

#ifdef MOISTURE
    /// @brief Computes the cloud levels: the mean total water, the mean liquid water after the saturation adjustment
    /// at the level's pressure, and the share of cells that hold liquid water (the example's cloud_levels()).
    /// @param fields the interval's fields, read with Q
    /// @param column the moist column that gives each level's pressure
    /// @return the cloud levels, indexed by z (zeros at the floor and the lid)
    /// @throws SetupError when the converter has no temperature scale or the column has fewer levels than the grid
    static CloudLevels clouds(const AtmosphereFields& fields, const MoistColumn& column) {
        const LevelGrid& levels = fields.grid().levels();
        const si::Temperature theta_0 = fields.converter().theta_0(); // the scale of theta(): its SetupError here, not in a thread
        (void)theta_0;
        column.require_levels(levels.levels()); // liquid()'s levels: its std::out_of_range cannot happen in a thread
        const double cells{ cells_per_level(fields.grid()) };
        const std::uint32_t nz{ levels.levels() };
        const std::uint32_t first{ levels.first_fluid() };
        const si::WaterContent no_water{ au::ZERO };
        CloudLevels c{ .qt = std::vector<si::WaterContent>(nz, no_water),
                       .ql = std::vector<si::WaterContent>(nz, no_water),
                       .fraction = std::vector<double>(nz, 0.0) };
        parallel_for(levels.fluid_levels(), [&](uint k) {
            const std::uint32_t z{ k + first };
            const LevelWater water{ level_water(fields, column, z, cells) };
            c.qt[z] = water.qt;
            c.ql[z] = water.ql;
            c.fraction[z] = water.fraction;
        });
        return c;
    }

    /// @brief Computes the water of one level: the sums over y, then x, divided by the cell count.
    /// @param fields the interval's fields, read with Q
    /// @param column the moist column that gives the level's pressure; it has the level (checked by clouds())
    /// @param z the level, from 0 at the floor cell; a fluid level
    /// @param cells the cells of one level (cells_per_level())
    /// @return the level's mean qt, mean ql and wet share
    static LevelWater level_water(const AtmosphereFields& fields, const MoistColumn& column, std::uint32_t z, double cells) {
        const std::uint32_t nx{ fields.grid().nx() };
        const std::uint32_t ny{ fields.grid().ny() };
        si::WaterContent qt{ au::ZERO };
        si::WaterContent ql{ au::ZERO };
        double wet{ 0.0 };
        for(std::uint32_t y = 0u; y < ny; y++) {
            for(std::uint32_t x = 0u; x < nx; x++) {
                const ulong n = fields.index(x, y, z);
                const si::WaterContent q = fields.qt(n);
                const si::Temperature theta = fields.theta(n);
                const si::WaterContent l = column.liquid(theta, q, z);
                qt += q;
                ql += l;
                const bool liquid_cell{ l > au::ZERO };
                const double wet_count{ liquid_cell ? 1.0 : 0.0 };
                wet += wet_count;
            }
        }
        const si::WaterContent mean_qt = qt / cells;
        const si::WaterContent mean_ql = ql / cells;
        const double wet_share = wet / cells;
        const LevelWater water{ .qt = mean_qt, .ql = mean_ql, .fraction = wet_share };
        return water;
    }
#endif // MOISTURE
};
