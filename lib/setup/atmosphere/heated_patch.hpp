#pragma once

#include "setup/units/units.hpp"

#include <cstdint>
#include <optional>
#include <string>

/// @file heated_patch.hpp
/// @brief A round patch of stronger surface heating: HeatedPatch and PatchMode.

/// @brief How a heated patch gets its stronger heat flux.
enum class PatchMode {
    SURFACE_LAYER, ///< the floor's fluid cells under the patch get the core's fixed 4x rate (cell type TYPE_X|TYPE_Y)
    HEATED_LAYERS  ///< the lowest N fluid layers under the patch each get the floor's rate: N times the flux, spread over N cells
};

/// @brief A round patch of stronger surface heating on the floor, which anchors a thermal (the CBL example's patch).
///
/// The disc's centre is the box's middle along x, and a given distance north of the middle along y. A floor cell
/// belongs to the patch when its centre is within the disc's radius. All lengths are in metres, so the patch is the
/// same at every cell size. HeatSourceBuilder::patch() applies it; CblDescription::validate() checks that it is
/// inside the box.
/// @code
/// using namespace si::symbols;
/// const HeatedPatch patch = HeatedPatch::disc(400.0 * m, 2000.0 * m, PatchMode::SURFACE_LAYER);
/// @endcode
class HeatedPatch {
public:
    /// The core's fixed flux factor of the surface layer: a TYPE_X|TYPE_Y cell heats at 4 times the floor's rate.
    static constexpr std::uint32_t surface_layer_factor = 4u;
    /// The number of heated layers when disc() gets none (HEATED_LAYERS).
    static constexpr std::uint32_t default_layers = 4u;

    /// @return no patch: the floor heats uniformly
    static HeatedPatch none() { return HeatedPatch(); }
    /// @brief Makes a round patch.
    /// @param diameter the disc's diameter, more than 0
    /// @param north_of_middle the distance of the disc's centre north of the box's middle (negative: south)
    /// @param mode how the patch gets its stronger flux: the surface layer at the core's 4x rate, or N heated layers
    /// @param layers the number of heated layers N, at least 1 (used only by HEATED_LAYERS; SURFACE_LAYER heats one)
    /// @return the patch
    /// @throws SetupError when the diameter is not more than 0, or HEATED_LAYERS gets 0 layers
    static HeatedPatch disc(si::Length diameter, si::Length north_of_middle, PatchMode mode, std::uint32_t layers = default_layers);

    /// @return true when there is a patch (made by disc()), false for none()
    bool active() const { return disc_.has_value(); }
    /// @brief Gives how the patch gets its stronger flux.
    /// @return the mode
    /// @throws SetupError without a patch
    PatchMode mode() const;
    /// @brief Gives the number of layers that the patch heats.
    /// @return 1 for SURFACE_LAYER, N for HEATED_LAYERS
    /// @throws SetupError without a patch
    std::uint32_t layers() const;
    /// @brief Gives the patch's heat flux over the floor's (a ratio).
    /// @return 4 for SURFACE_LAYER (the core's fixed factor), N for HEATED_LAYERS
    /// @throws SetupError without a patch
    std::uint32_t flux_factor() const;
    /// @brief Gives the disc's diameter.
    /// @return the diameter
    /// @throws SetupError without a patch
    si::Length diameter() const { return disc().diameter; }
    /// @brief Gives the distance of the disc's centre north of the box's middle.
    /// @return the distance (negative: south)
    /// @throws SetupError without a patch
    si::Length north_of_middle() const { return disc().north; }

    /// @brief Tells whether a floor cell's centre is within the disc.
    /// @param x the cell's index along x, from 0 at the west edge
    /// @param y the cell's index along y, from 0 at the south edge
    /// @param nx the number of cells of the box along x
    /// @param ny the number of cells of the box along y
    /// @param cell the edge of one cubic cell, more than 0
    /// @return true when the centre, at (x + 1/2, y + 1/2) cells, is at most the radius from the disc's centre at
    /// (nx/2, ny/2 + north_of_middle / cell) cells; false without a patch
    bool covers(std::uint32_t x, std::uint32_t y, std::uint32_t nx, std::uint32_t ny, si::Length cell) const;
    /// @brief Describes the patch for the run's log.
    /// @return "no heated patch", or its diameter, position and flux factor
    /// @throws SetupError when the text does not fit its buffer
    std::string describe() const;

private:
    /// @brief The values of a disc.
    struct Disc {
        si::Length diameter{ au::ZERO };            ///< the disc's diameter
        si::Length north{ au::ZERO };               ///< the distance of the centre north of the box's middle
        PatchMode mode{ PatchMode::SURFACE_LAYER }; ///< how the flux is made stronger
        std::uint32_t layers{};                     ///< the number of heated layers (1 for SURFACE_LAYER)
    };
    /// @brief Makes no patch. Use none() or disc().
    HeatedPatch() = default;
    /// @brief Gives the disc's values.
    /// @return the values
    /// @throws SetupError without a patch
    const Disc& disc() const;
    std::optional<Disc> disc_{}; ///< the disc, or none for no patch
};
