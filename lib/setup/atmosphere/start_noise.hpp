#pragma once

#include "setup/core/setup_error.hpp"
#include "setup/units/units.hpp"

#include <cstdint>
#include <optional>
#include <random>
#include <string>

/// @file start_noise.hpp
/// @brief The random temperature perturbation of an atmosphere's start state: StartNoise and CellCentre.

/// @brief The position of a cell's centre: metres east and north of the box's south-west corner, and the height above
/// the floor's top. AtmosphereGrid::centre() makes it.
struct CellCentre {
    si::Length east{ au::ZERO };   ///< the distance east of the box's west edge
    si::Length north{ au::ZERO };  ///< the distance north of the box's south edge
    si::Length height{ au::ZERO }; ///< the height above the floor's top (negative inside the floor)
};

/// @brief The random temperature perturbation of the start state below a height. It breaks the symmetry, so that
/// the convection starts. The values are uniform in [-amplitude, amplitude). There are two kinds:
/// - per_cell: one draw per fluid cell below the top, as the CBL example draws. The draws come from std::mt19937
///   with the seed, through std::uniform_real_distribution<float> (float on purpose: the same draws as the verified
///   example), in the order in which AtmosphereStartBuilder visits the cells (z, then y, then x). The C++ standard
///   fixes mt19937's output, but not the distribution's algorithm: the same seed gives other draws with another
///   standard library (MSVC against libstdc++).
/// - blocks: one value per cube of a given size, from a hash of the seed and the cube's indices. Every cell size
///   that divides the cube gets the same physical field, with every standard library: for comparisons of runs at
///   different cell sizes (such as cells finer than 5 m).
/// @code
/// using namespace si::symbols;
/// const StartNoise noise = StartNoise::per_cell(0.1 * K, 300.0 * m, 2u);
/// @endcode
class StartNoise {
public:
    /// @brief The values of one start, cell by cell. StartNoise::sequence() makes it; AtmosphereStartBuilder asks it
    /// for one value per fluid cell, in its visiting order.
    class Sequence {
    public:
        /// @brief Gives the perturbation of the next cell.
        /// @param cell the cell's centre (per_cell: the cells must come in the visiting order; blocks: east, north and
        /// height 0 or more; a floor cell below the floor's top has a negative height)
        /// @return a value in [-amplitude, amplitude) below the top; 0 at and above the top, and without noise.
        /// per_cell uses a draw only below the top.
        /// @throws SetupError for block noise at a negative east, north or height below the top
        si::TemperatureDifference at(const CellCentre& cell);
        /// @return the number of draws used so far (per_cell; 0 for blocks, which use no draws)
        std::size_t draws() const { return draws_; }

    private:
        friend class StartNoise;
        /// @brief How the sequence makes its values.
        enum class Kind {
            NONE,     ///< every cell 0
            PER_CELL, ///< one draw per cell
            BLOCKS    ///< one hashed value per block
        };
        /// @brief Makes the sequence of a start. Use StartNoise::sequence().
        /// @param kind how the values are made
        /// @param amplitude the largest perturbation
        /// @param top the height at and above which every value is 0
        /// @param seed the seed of the engine (per_cell) or of the hash (blocks)
        /// @param block the edge of one cube (blocks only)
        Sequence(Kind kind, si::TemperatureDifference amplitude, si::Length top, unsigned seed, si::Length block);
        /// @brief Mixes a 64-bit key into a well-mixed 64-bit value (the finaliser of SplitMix64).
        /// @param k the key
        /// @return the mixed value
        static std::uint64_t mix(std::uint64_t k);
        /// @brief Gives the hashed value of the cube that holds a cell's centre.
        /// @param cell the cell's centre, with east, north and height 0 or more
        /// @return the cube's value in [-1, 1), from the seed and the cube's three indices
        /// @throws SetupError for a negative coordinate
        double block_value(const CellCentre& cell) const;

        Kind kind_{ Kind::NONE };                        ///< how the values are made
        si::TemperatureDifference amplitude_{ au::ZERO }; ///< the largest perturbation
        /// per_cell: the random engine
        std::mt19937 random_{}; // NOLINT(bugprone-random-generator-seed): the constructor seeds it with the noise's seed
        std::uniform_real_distribution<float> noise_{};  ///< per_cell: the draws in [-amplitude, amplitude) in K, as floats (the example's draws)
        si::Length top_{ au::ZERO };                     ///< the height at and above which every value is 0
        unsigned seed_{};                                ///< blocks: the hash's seed
        si::Length block_{ au::ZERO };                   ///< blocks: the edge of one cube
        std::size_t draws_{};                            ///< per_cell: the number of draws used
    };

    /// @return no noise: the start state is the sounding alone
    static StartNoise none() { return StartNoise(); }
    /// @brief Makes the per-cell noise: one draw per fluid cell below the top.
    /// @param amplitude the largest perturbation, more than 0 (the CBL example's 0.1 K)
    /// @param top the height above the floor's top below which the cells get a draw, more than 0
    /// @param seed the seed of the random engine
    /// @return the noise
    /// @throws SetupError when the amplitude or the top is not more than 0
    static StartNoise per_cell(si::TemperatureDifference amplitude, si::Length top, unsigned seed);
    /// @brief Makes the block noise: one value per cube below the top, the same at every cell size that divides
    /// the cube.
    /// @param amplitude the largest perturbation, more than 0
    /// @param top the height above the floor's top below which the cells get a value, more than 0 and a whole number
    /// of blocks
    /// @param seed the seed of the hash
    /// @param block the edge of one cube, more than 0
    /// @return the noise
    /// @throws SetupError when the amplitude, the top or the block is not more than 0, or the top is not a whole
    /// number of blocks
    static StartNoise blocks(si::TemperatureDifference amplitude, si::Length top, unsigned seed, si::Length block);

    /// @return true when the start gets noise (per_cell() or blocks()), false for none()
    bool active() const { return values_.has_value(); }
    /// @brief Gives the seed.
    /// @return the seed of the engine (per_cell) or of the hash (blocks)
    /// @throws SetupError without noise
    unsigned seed() const { return values().seed; }
    /// @brief Gives the largest perturbation.
    /// @return the amplitude
    /// @throws SetupError without noise
    si::TemperatureDifference amplitude() const { return values().amplitude; }
    /// @brief Gives the height below which the cells get a value.
    /// @return the top, above the floor's top
    /// @throws SetupError without noise
    si::Length top() const { return values().top; }
    /// @brief Makes the values of one start.
    /// @param cell the run's cell size (blocks: more than 0, and it must divide the block)
    /// @return the sequence; all values 0 without noise
    /// @throws SetupError for block noise when the cell size is not more than 0 or does not divide the block
    Sequence sequence(si::Length cell) const;
    /// @brief Describes the noise for the run's log.
    /// @return "no start noise", or the amplitude, the block or "per cell", the top and the seed
    /// @throws SetupError when the text does not fit its buffer
    std::string describe() const;

private:
    /// @brief The values of the noise.
    struct Values {
        si::TemperatureDifference amplitude{ au::ZERO }; ///< the largest perturbation
        si::Length top{ au::ZERO };                      ///< the height above the floor's top below which the cells get a value
        unsigned seed{};                                 ///< the seed of the engine or of the hash
        std::optional<si::Length> block{};               ///< the edge of one cube (blocks), or none (per_cell)
    };
    /// @brief Makes no noise. Use none(), per_cell() or blocks().
    StartNoise() = default;
    /// @brief Gives the noise's values.
    /// @return the values
    /// @throws SetupError without noise
    const Values& values() const {
        if(!values_) {
            throw SetupError("StartNoise: no noise");
        }
        return *values_;
    }
    /// @brief Tells whether a length is a whole number of blocks.
    /// @param length the length to test: the noise's top, or a block against a cell size
    /// @param block the edge of one cube (or the cell size), more than 0
    /// @return true when length / block is within 1e-4 times max(1, length / block) of a whole number
    static bool whole_blocks(si::Length length, si::Length block);
    std::optional<Values> values_{}; ///< the noise's values, or none for no noise
};
