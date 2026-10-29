#pragma once

/// @file heat_source_builder.hpp
/// @brief The middle layer's (L2) builder of an atmosphere's floor heating. Header-side: it needs the build's
/// defines and the core (lbm.hpp).

#include "setup/atmosphere/atmosphere_grid.hpp"
#include "setup/atmosphere/heated_patch.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"

#include "lbm.hpp"

#include <algorithm>
#include <cstdint>

#ifndef TEMPERATURE
#error "setup/atmosphere/heat_source_builder.hpp needs TEMPERATURE in the EXTENSIONS"
#endif // TEMPERATURE

/// @brief What HeatSourceBuilder::apply() gave the core: the count of patch cells and the heat rate.
struct SourceSummary {
    std::uint64_t patch_cells = 0u; ///< the number of cells of the first fluid level whose centre is in the patch
    float heat_rate = 0.0f;         ///< the lattice temperature that a source cell gains per time step (LBM::set_heat_rate())
};

/// @brief Sets the floor's heating of an atmosphere: the source flags and the core's heat rate.
///
/// Middle layer (L2). A temporary: make it with AtmosphereSimulation::heat_sources(), set the flux and the patch,
/// call apply() once, after the start state (AtmosphereStartBuilder writes the walls' flags, and apply() writes over
/// the flags of the source cells). It refers to the LBM, which must outlive it. apply() writes:
/// - the host flags: every cell of the first fluid level is a source (TYPE_Y) of the surface flux; under the patch
///   either the first level gets the core's fixed 4x rate (TYPE_X|TYPE_Y, PatchMode::SURFACE_LAYER) or the lowest N
///   fluid levels are sources (TYPE_Y, PatchMode::HEATED_LAYERS);
/// - the core's heat rate per time step (LBM::set_heat_rate()).
///
/// @code
/// using namespace si::symbols;
/// const SourceSummary s = atmosphere.heat_sources().surface_flux(0.262 * K * m / s).patch(HeatedPatch::disc(...)).apply();
/// @endcode
class HeatSourceBuilder {
public:
    /// @brief Makes a builder with a flux of 0 and no patch.
    /// @param lbm the core, before its first run; it must outlive the builder
    /// @param grid the box's cells and levels (the layout of the host buffers)
    /// @param converter the run's conversions
    /// @param record where apply() also writes its summary, or nullptr (AtmosphereSimulation keeps it for its
    /// fingerprint); it must outlive apply()
    HeatSourceBuilder(LBM& lbm, const AtmosphereGrid& grid, const si::LatticeConverter& converter, SourceSummary* record = nullptr)
        : lbm_(lbm), grid_(grid), converter_(converter), record_(record) {}

    /// @brief Sets the floor's uniform heat flux.
    /// @param flux the floor's kinematic heat flux (the sensible heat flux over ρ cp)
    /// @return this builder
    HeatSourceBuilder& surface_flux(si::KinematicHeatFlux flux) {
        flux_ = flux;
        return *this;
    }
    /// @brief Sets a round patch of stronger heating.
    /// @param patch the patch (default HeatedPatch::none())
    /// @return this builder
    HeatSourceBuilder& patch(HeatedPatch patch) {
        patch_ = patch;
        return *this;
    }

    /// @brief Writes the source flags and the core's heat rate (see the class description).
    /// @return the number of patch cells on the first fluid level and the lattice heat rate
    SourceSummary apply() {
        const LevelGrid& levels = grid_.levels();
        const std::uint32_t first = levels.first_fluid();
        const PatchLayers p = patch_layers();
        const std::uint32_t layers = std::max(p.heated_layers, 1u);
        const std::uint32_t wanted_top = first + layers;
        const std::uint32_t fluid_top = levels.last_fluid() + 1u;
        const std::uint32_t top = std::min(wanted_top, fluid_top); // the levels the loop sets
        SourceSummary s{};
        for(std::uint32_t z = first; z < top; z++) {
            const std::uint32_t level{ z - first };
            const std::uint64_t patch_cells{ flag_level(z, level, p) };
            if(level == 0u) {
                s.patch_cells = patch_cells;
            }
        }
        const float heat_rate = converter_.heat_rate(flux_);
        s.heat_rate = heat_rate;
        lbm_.set_heat_rate(s.heat_rate);
        if(record_ != nullptr) {
            *record_ = s;
        }
        return s;
    }

private:
    /// @brief How the patch heats, as the flags need it: the surface layer at the core's 4x rate, or heated layers.
    struct PatchLayers {
        bool surface = false;            ///< true: the patch's first level gets the core's 4x rate (SURFACE_LAYER)
        std::uint32_t heated_layers = 0u; ///< the number of fluid levels the patch heats (HEATED_LAYERS); 0 otherwise
    };

    /// @brief Gives the patch's mode as the flags need it.
    /// @return the mode; neither the surface layer nor a heated layer without a patch
    PatchLayers patch_layers() const {
        if(!patch_.active()) {
            return PatchLayers{};
        }
        const bool surface = patch_.mode() == PatchMode::SURFACE_LAYER;
        std::uint32_t heated_layers{ 0u };
        if(!surface) {
            heated_layers = patch_.layers();
        }
        const PatchLayers layers{ .surface = surface, .heated_layers = heated_layers };
        return layers;
    }

    /// @brief Writes the source flags of one level, in the order y, then x.
    /// @param z the level, from 0 at the floor cell
    /// @param level the level counted from the first fluid level (0: the first fluid level)
    /// @param p the patch's mode
    /// @return the number of the level's cells whose centre is in the patch
    std::uint64_t flag_level(std::uint32_t z, std::uint32_t level, const PatchLayers& p) {
        const std::uint32_t nx{ grid_.nx() };
        const std::uint32_t ny{ grid_.ny() };
        const si::Length cell{ grid_.cell() };
        const ulong z_wide{ static_cast<ulong>(z) };
        const ulong ny_wide{ static_cast<ulong>(ny) };
        const ulong nx_wide{ static_cast<ulong>(nx) };
        const ulong level_rows{ z_wide * ny_wide };
        std::uint64_t patch_cells{ 0u };
        for(std::uint32_t y = 0u; y < ny; y++) {
            const ulong y_wide{ static_cast<ulong>(y) };
            const ulong row{ y_wide + level_rows };
            const ulong row_start{ row * nx_wide };
            for(std::uint32_t x = 0u; x < nx; x++) {
                const ulong x_wide{ static_cast<ulong>(x) };
                const ulong n{ x_wide + row_start };
                const bool in_patch = patch_.covers(x, y, nx, ny, cell);
                flag_cell(n, level, in_patch, p);
                const std::uint64_t patch_cell{ in_patch ? 1u : 0u };
                patch_cells += patch_cell;
            }
        }
        return patch_cells;
    }

    /// @brief Writes a cell's source flag: every cell of the first fluid level is a source (TYPE_Y); under the patch
    /// the heated layers are sources too, or the first level gets the core's fixed 4x rate (TYPE_X|TYPE_Y). Other
    /// cells keep their flag.
    /// @param n the cell's linear index
    /// @param level the cell's level counted from the first fluid level (0: the first fluid level)
    /// @param in_patch true when the cell's centre is in the patch
    /// @param p the patch's mode
    void flag_cell(ulong n, std::uint32_t level, bool in_patch, const PatchLayers& p) {
        const bool first_level = level == 0u;
        const bool heated_layer = in_patch && level < p.heated_layers;
        if(first_level || heated_layer) {
            lbm_.flags[n] = TYPE_Y; // the example's z == 1 || heated layers
        }
        const bool surface_patch = in_patch && p.surface && first_level;
        if(surface_patch) {
            lbm_.flags[n] = TYPE_X | TYPE_Y; // the core's fixed 4x
        }
    }

    LBM& lbm_;                                                  ///< the core, whose flags and heat rate apply() writes
    AtmosphereGrid grid_;                                       ///< the box's cells and levels
    si::LatticeConverter converter_;                            ///< the run's conversions
    si::KinematicHeatFlux flux_{ au::ZERO };                    ///< the floor's kinematic heat flux
    HeatedPatch patch_ = HeatedPatch::none();                   ///< the patch of stronger heating, or none
    SourceSummary* record_{};                                   ///< where apply() also writes its summary, or nullptr
};
