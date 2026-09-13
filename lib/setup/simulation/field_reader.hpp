#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/unit_scale.hpp"
#include "lbm.hpp"
#include <algorithm>
#include <cmath>

/// @brief A snapshot of the simulation's fields in physical units (Simulation::fields()): the velocity, pressure and
/// density of every cell, or at a point.
///
/// Taking the snapshot reads the fields from the device once the simulation has started (before, the device holds
/// nothing yet and the host holds the setup). Positions are in metres from the domain's origin corner (cell i spans
/// i to i+1 cell sizes, its center at i+0.5). Pressures are relative to the fluid at rest.
/// @code
/// const FieldReader fields = sim.fields();
/// fields.for_each_cell([&](const FieldReader::Cell& c) { if(!c.solid) energy += magnitude(c.velocity).si(); });
/// const Speed u = magnitude(fields.velocity_at({ 0.5_m, 1.0_m, 0.2_m }));
/// @endcode
class FieldReader {
public:
    struct Cell {
        Position center;   ///< the cell's center
        Velocity velocity;
        Pressure pressure; ///< relative to the fluid at rest
        Density density;
        bool solid;        ///< a solid cell (its velocity is a wall's)
    };

    /// @param from_device read the velocity, density and flags from the device (once the simulation has started)
    FieldReader(LBM& lbm, const UnitScale& unit_scale, bool from_device) : lbm_(lbm), units_(unit_scale) {
        if(!from_device) return;
        lbm_.u.read_from_device();
        lbm_.rho.read_from_device();
        lbm_.flags.read_from_device();
    }

    /// The cell containing a point (the nearest one outside the domain).
    Cell cell_at(const Position& point) const {
        const float cell_size = units_.cell_size().si();
        const uint32_t x = clamp_index(point.x.si() / cell_size, lbm_.get_Nx());
        const uint32_t y = clamp_index(point.y.si() / cell_size, lbm_.get_Ny());
        const uint32_t z = clamp_index(point.z.si() / cell_size, lbm_.get_Nz());
        return cell(lbm_.index(x, y, z), x, y, z);
    }

    Velocity velocity_at(const Position& point) const { return cell_at(point).velocity; }
    Pressure pressure_at(const Position& point) const { return cell_at(point).pressure; }
    Density density_at(const Position& point) const { return cell_at(point).density; }

    /// Calls f(const Cell&) for every cell of the domain, on the host, in order of the cell index.
    template<typename F>
    void for_each_cell(F&& f) const {
        for(uint64_t n = 0ull; n < lbm_.get_N(); n++) {
            uint32_t x = 0u, y = 0u, z = 0u;
            lbm_.coordinates(n, x, y, z);
            f(cell(n, x, y, z));
        }
    }

private:
    LBM& lbm_;
    UnitScale units_;

    static uint32_t clamp_index(float cells, uint32_t count) {
        return (uint32_t)std::clamp(std::floor(cells), 0.0f, (float)(count - 1u));
    }

    Cell cell(uint64_t n, uint32_t x, uint32_t y, uint32_t z) const {
        const float cell_size = units_.cell_size().si();
        const float rho = lbm_.rho[n];
        return { { Length::from_si(((float)x + 0.5f) * cell_size), Length::from_si(((float)y + 0.5f) * cell_size), Length::from_si(((float)z + 0.5f) * cell_size) },
                 { units_.si_velocity(lbm_.u.x[n]), units_.si_velocity(lbm_.u.y[n]), units_.si_velocity(lbm_.u.z[n]) },
                 units_.si_pressure((rho - 1.0f) / 3.0f), // the lattice pressure is (rho-1)/3
                 units_.si_density(rho),
                 (lbm_.flags[n] & TYPE_S) != 0u };
    }
};
