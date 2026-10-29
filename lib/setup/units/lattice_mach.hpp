#pragma once

#include "setup/core/setup_error.hpp"

/// @file lattice_mach.hpp
/// @brief The lattice Mach number: how fast the reference speed moves through the lattice.

namespace si {

/// @brief The lattice Mach number of a simulation's reference speed: the reference speed in cells per time step over
/// the lattice's speed of sound (1/√3 cells per time step).
///
/// It is a numerical choice, not a physical value: a smaller Mach number gives a smaller time step, so the lattice
/// flow is less compressible and the run is slower. TimeStepRule::from_mach() and the Mach constructor of Simulation
/// use it to set the time step. The default reference speed is 0.1 cells per time step (Mach about 0.17).
/// @code
/// const si::LatticeMach mach(0.13);         // the reference speed at 0.075 cells per time step
/// const double cells_per_step = mach.lattice_speed();
/// @endcode
class LatticeMach {
public:
    /// The lattice's speed of sound in cells per time step: 1/√3.
    static constexpr double sound_speed = 0.57735026918962576;
    /// The default reference speed in cells per time step.
    static constexpr double default_lattice_speed = 0.1;

    /// @brief Makes the default: a reference speed of 0.1 cells per time step.
    constexpr LatticeMach() = default;

    /// @brief Makes a lattice Mach number from its value.
    /// @param mach the reference speed over the lattice's speed of sound (a ratio), more than 0 and at most 1
    /// @throws SetupError when mach is not more than 0 or is more than 1 (a lattice flow faster than its sound speed)
    explicit LatticeMach(double mach) : lattice_speed_(mach * sound_speed) {
        if(!(mach > 0.0) || mach > 1.0) {
            throw SetupError("si::LatticeMach: the Mach number must be more than 0 and at most 1");
        }
    }

    /// @brief Gives the Mach number.
    /// @return the reference speed over the lattice's speed of sound (a ratio)
    constexpr double value() const {
        const double mach{ lattice_speed_ / sound_speed };
        return mach;
    }

    /// @brief Gives the reference speed in lattice units.
    /// @return the reference speed in cells per time step
    constexpr double lattice_speed() const { return lattice_speed_; }

private:
    double lattice_speed_{ default_lattice_speed }; ///< the reference speed in cells per time step
};

} // namespace si
