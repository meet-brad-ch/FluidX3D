#pragma once

#include "setup/units/lattice_mach.hpp"
#include "setup/units/units.hpp"

#include <cstddef>
#include <string>
#include <variant>

/// @brief The kinds of TimeStepRule, each a record of its values.
///
/// They are outside the class on purpose: a class nested in TimeStepRule is not complete for its default member
/// initializers until TimeStepRule is complete, so GCC (CWG 1397) cannot default-construct the std::variant of them in
/// TimeStepRule's own member initializer (rule_{}).
namespace time_step_rule_kinds {
/// @brief The step proportional to the cell.
struct ConstantLatticeSpeed {
    si::Duration step{ au::ZERO };  ///< the time step at the reference cell
    si::Length at_cell{ au::ZERO }; ///< the reference cell
};
/// @brief The same step at every cell.
struct Fixed {
    si::Duration step{ au::ZERO }; ///< the time step
};
/// @brief The step that puts the fastest speed at a lattice Mach number.
struct FromMach {
    si::Speed fastest{ au::ZERO }; ///< the fastest speed in the flow
    si::LatticeMach mach{};        ///< its lattice Mach number
};
} // namespace time_step_rule_kinds

/// @brief How a simulation chooses its time step from its cell size: a value, stated and printed, so that the same
/// rule gives the same lattice at every cell size.
///
/// - constant_lattice_speed(step, at_cell): dt = step * cell / at_cell, so the lattice speed dx/dt is the same at
///   every cell size (0.04 s at 2.5 m: 62.5 m/s). A speed of the flow is then the same number of cells per time step
///   at every resolution, and so is the compressibility error. This is the rule for atmospheric flows.
/// - fixed(step): the same time step at every cell size (tests in lattice-like units: 1 m cells, 1 s steps).
/// - from_mach(fastest, mach): the step that puts the fastest speed at the given lattice Mach number.
/// @code
/// using namespace si::symbols;
/// const TimeStepRule rule = TimeStepRule::constant_lattice_speed(0.04 * s, 2.5 * m);
/// const si::Duration dt = rule.step(10.0 * m); // 0.16 s
/// @endcode
class TimeStepRule {
public:
    /// @brief A time step proportional to the cell size: the lattice speed dx/dt stays the same.
    /// @param step the time step at the reference cell size, above 0
    /// @param at_cell the reference cell size, above 0
    /// @return the rule
    /// @throws SetupError when the step or the cell size is not above 0
    static TimeStepRule constant_lattice_speed(si::Duration step, si::Length at_cell);

    /// @brief The same time step at every cell size.
    /// @param step the time step, above 0
    /// @return the rule
    /// @throws SetupError when the step is not above 0
    static TimeStepRule fixed(si::Duration step);

    /// @brief The time step at which the fastest speed is the given lattice Mach number.
    /// @param fastest the fastest speed in the flow, above 0
    /// @param mach the lattice Mach number of that speed
    /// @return the rule
    /// @throws SetupError when the speed is not above 0
    static TimeStepRule from_mach(si::Speed fastest, si::LatticeMach mach);

    /// @brief Gives the time step that the rule sets at a cell size.
    /// @param cell the cell size, above 0
    /// @return the time step at that cell size (before the core rounds it to a float)
    /// @throws SetupError when the cell size is not above 0
    si::Duration step(si::Length cell) const;

    /// @brief Gives the lattice speed at a cell size: the speed of one cell per time step, dx/dt.
    /// @param cell the cell size, above 0
    /// @return the lattice speed at that cell size
    /// @throws SetupError when the cell size is not above 0 (from step())
    si::Speed lattice_speed(si::Length cell) const {
        const si::Duration time_step{ step(cell) };
        const si::Speed speed{ cell / time_step };
        return speed;
    }

    /// @brief Describes the rule in words, for the set-up line.
    /// @return for example "time step 0.04 s per 2.5 m of cell (lattice speed 62.5 m/s)"
    /// @throws SetupError when the description does not fit its text buffer (a programming error)
    std::string describe() const;

private:
    using ConstantLatticeSpeed = time_step_rule_kinds::ConstantLatticeSpeed; ///< the step proportional to the cell
    using Fixed = time_step_rule_kinds::Fixed;                               ///< the same step at every cell
    using FromMach = time_step_rule_kinds::FromMach;                         ///< the step from a lattice Mach number
    using Rule = std::variant<ConstantLatticeSpeed, Fixed, FromMach>;        ///< one of the rules

    /// @brief Makes the rule from one of its kinds. Use the static functions, which check the values.
    /// @param rule the kind of rule with its values
    explicit TimeStepRule(Rule rule) : rule_(rule) {}

    /// @brief Gives the step of the rule proportional to the cell: step times cell over the reference cell.
    /// @param r the rule proportional to the cell
    /// @param cell the cell size
    /// @return the time step at that cell size
    static si::Duration step_for(const ConstantLatticeSpeed& r, si::Length cell);
    /// @brief Gives the step of the rule of the same step at every cell.
    /// @param r the rule of the same step at every cell
    /// @param cell the cell size (not used)
    /// @return the rule's time step
    static si::Duration step_for(const Fixed& r, si::Length cell);
    /// @brief Gives the step at which the fastest speed moves the Mach number's lattice speed in cells per step.
    /// @param r the rule of the fastest speed at a lattice Mach number
    /// @param cell the cell size
    /// @return cell / fastest times the lattice speed
    static si::Duration step_for(const FromMach& r, si::Length cell);

    /// @brief Describes the rule proportional to the cell.
    /// @param r the rule proportional to the cell
    /// @return the step, the reference cell and the lattice speed in words
    /// @throws SetupError when the description does not fit its text buffer
    static std::string describe_rule(const ConstantLatticeSpeed& r);
    /// @brief Describes the rule of the same step at every cell.
    /// @param r the rule of the same step at every cell
    /// @return the step in words
    /// @throws SetupError when the description does not fit its text buffer
    static std::string describe_rule(const Fixed& r);
    /// @brief Describes the rule of the fastest speed at a lattice Mach number.
    /// @param r the rule of the fastest speed at a lattice Mach number
    /// @return the speed and the lattice speed in words
    /// @throws SetupError when the description does not fit its text buffer
    static std::string describe_rule(const FromMach& r);

    /// The size of the text buffer of a description, the end mark included.
    static constexpr std::size_t describe_capacity = 160;

    Rule rule_{}; ///< the kind of rule with its values
};
