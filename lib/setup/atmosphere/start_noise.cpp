#include "setup/atmosphere/start_noise.hpp"
#include "setup/core/setup_error.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>

namespace {

/// The tolerance of whole_blocks(), in blocks.
constexpr double block_tolerance = 1.0E-4;
/// The shift that keeps the top 24 bits of a block's 64-bit hash: they make the block's value.
constexpr unsigned value_shift = 40u;
/// The number of values of 24 bits (2^24): the kept bits over it are in [0, 1).
constexpr double value_range = 16777216.0; // 2^24

/// @brief Gives the index of the cube that holds a position along one axis.
/// @param position a position along the axis, 0 or more
/// @param block the edge of one cube, more than 0
/// @return floor(position / block), counted from 0
/// @throws SetupError for a negative position, which has no cube index
std::uint64_t block_index(si::Length position, si::Length block) {
    if(position < au::ZERO) {
        const double metres{ position.in(au::meters) };
        const std::string message{ "StartNoise: block noise needs coordinates of 0 m or more, not " + std::to_string(metres) + " m" };
        throw SetupError(message);
    }
    const auto ratio = position / block;
    const double blocks{ au::as_raw_number(ratio) };
    const double index_double{ std::floor(blocks) };
    const std::uint64_t index{ static_cast<std::uint64_t>(index_double) };
    return index;
}

} // namespace

StartNoise::Sequence::Sequence(Kind kind, si::TemperatureDifference amplitude, si::Length top, unsigned seed, si::Length block)
    : kind_(kind), amplitude_(amplitude), random_(seed), top_(top), seed_(seed), block_(block) {
    const double amplitude_k{ amplitude.in(au::kelvins) };
    const float a{ static_cast<float>(amplitude_k) }; // the example's float draws
    const float minus_a{ -a };
    noise_ = std::uniform_real_distribution<float>(minus_a, a);
}

std::uint64_t StartNoise::Sequence::mix(std::uint64_t k) {
    const std::uint64_t k1{ k + 0x9E3779B97F4A7C15ull };
    const std::uint64_t shifted1{ k1 >> 30 };
    const std::uint64_t mixed1{ k1 ^ shifted1 };
    const std::uint64_t k2{ mixed1 * 0xBF58476D1CE4E5B9ull };
    const std::uint64_t shifted2{ k2 >> 27 };
    const std::uint64_t mixed2{ k2 ^ shifted2 };
    const std::uint64_t k3{ mixed2 * 0x94D049BB133111EBull };
    const std::uint64_t shifted3{ k3 >> 31 };
    const std::uint64_t result{ k3 ^ shifted3 };
    return result;
}

bool StartNoise::whole_blocks(si::Length length, si::Length block) {
    const auto ratio = length / block;
    const double n{ au::as_raw_number(ratio) };
    const double nearest{ std::round(n) };
    const double off{ n - nearest };
    const double distance{ std::fabs(off) };
    const double scale{ std::fmax(1.0, n) };
    const double tolerance{ block_tolerance * scale };
    const bool whole{ distance <= tolerance };
    return whole;
}

double StartNoise::Sequence::block_value(const CellCentre& cell) const {
    // the block holding the cell's centre; the value from the seed and the block's three indices
    const std::uint64_t ix{ block_index(cell.east, block_) };
    const std::uint64_t iy{ block_index(cell.north, block_) };
    const std::uint64_t iz{ block_index(cell.height, block_) };
    const std::uint64_t seed_key{ static_cast<std::uint64_t>(seed_) };
    const std::uint64_t h0{ mix(seed_key) };
    const std::uint64_t h1{ mix(h0 ^ ix) };
    const std::uint64_t h2{ mix(h1 ^ iy) };
    const std::uint64_t h{ mix(h2 ^ iz) };
    const std::uint64_t value_bits{ h >> value_shift };
    const double value_bits_double{ static_cast<double>(value_bits) };
    const double unit{ value_bits_double / value_range }; // [0, 1)
    const double twice_unit{ 2.0 * unit };
    const double value{ twice_unit - 1.0 };
    return value;
}

si::TemperatureDifference StartNoise::Sequence::at(const CellCentre& cell) {
    si::TemperatureDifference value{ au::kelvins(0.0) };
    const bool none{ kind_ == Kind::NONE };
    const bool below_top{ cell.height < top_ };
    if(!none && below_top) {
        if(kind_ == Kind::PER_CELL) {
            draws_++;
            const float draw{ noise_(random_) };
            const double draw_k{ static_cast<double>(draw) };
            value = au::kelvins(draw_k);
        } else {
            const double block{ block_value(cell) };
            value = amplitude_ * block;
        }
    }
    return value;
}

StartNoise StartNoise::per_cell(si::TemperatureDifference amplitude, si::Length top, unsigned seed) {
    if(!(amplitude > au::ZERO)) {
        throw SetupError("StartNoise: the amplitude must be more than 0 K (no noise: StartNoise::none())");
    }
    if(!(top > au::ZERO)) {
        throw SetupError("StartNoise: the top must be more than 0 m");
    }
    StartNoise noise;
    noise.values_ = Values{ .amplitude = amplitude, .top = top, .seed = seed, .block = std::nullopt };
    return noise;
}

StartNoise StartNoise::blocks(si::TemperatureDifference amplitude, si::Length top, unsigned seed, si::Length block) {
    if(!(block > au::ZERO)) {
        throw SetupError("StartNoise::blocks(): the block must be more than 0 m");
    }
    StartNoise noise = per_cell(amplitude, top, seed);
    const bool whole{ whole_blocks(top, block) };
    if(!whole) {
        throw SetupError("StartNoise::blocks(): the top must be a whole number of blocks");
    }
    if(!noise.values_.has_value()) {
        throw SetupError("StartNoise::blocks(): per_cell() gave no values");
    }
    noise.values_->block = block;
    return noise;
}

StartNoise::Sequence StartNoise::sequence(si::Length cell) const {
    const si::Length no_block{ au::meters(0.0) };
    if(!values_.has_value()) {
        const si::TemperatureDifference no_amplitude{ au::kelvins(0.0) };
        const si::Length no_top{ au::meters(0.0) };
        Sequence none_sequence(Sequence::Kind::NONE, no_amplitude, no_top, 0u, no_block);
        return none_sequence;
    }
    const Values& v = *values_;
    if(!v.block.has_value()) {
        Sequence cell_sequence(Sequence::Kind::PER_CELL, v.amplitude, v.top, v.seed, no_block);
        return cell_sequence;
    }
    const si::Length block{ *v.block };
    const bool cell_positive{ cell > au::ZERO };
    const bool divides{ cell_positive && whole_blocks(block, cell) };
    if(!divides) {
        throw SetupError("StartNoise::blocks(): the cell size must divide the block (the same field at every such cell)");
    }
    Sequence block_sequence(Sequence::Kind::BLOCKS, v.amplitude, v.top, v.seed, block);
    return block_sequence;
}

std::string StartNoise::describe() const {
    std::string description{ "no start noise" };
    if(values_) {
        const double amplitude_k{ values_->amplitude.in(au::kelvins) };
        const double top_m{ values_->top.in(au::meters) };
        const unsigned noise_seed{ values_->seed };
        constexpr std::size_t text_capacity{ 160u };
        char text[text_capacity]{};
        int length{ 0 };
        if(values_->block) {
            const double block_m{ values_->block->in(au::meters) };
            length = std::snprintf(text, sizeof(text), "start noise +-%.3f K per %.1f m block below %.0f m, seed %u", amplitude_k, block_m, top_m, noise_seed);
        } else {
            length = std::snprintf(text, sizeof(text), "start noise +-%.3f K per cell below %.0f m, seed %u", amplitude_k, top_m, noise_seed);
        }
        const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
        if(cut) {
            throw SetupError("StartNoise::describe(): the text does not fit its buffer");
        }
        description = text;
    }
    return description;
}
