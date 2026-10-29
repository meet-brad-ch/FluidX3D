#include "setup/core/time_step_rule.hpp"
#include "setup/core/setup_error.hpp"

#include <cstdio>

TimeStepRule TimeStepRule::constant_lattice_speed(si::Duration step, si::Length at_cell) {
    if(!(step > au::ZERO)) {
        throw SetupError("TimeStepRule::constant_lattice_speed(): the time step must be above 0 s");
    }
    if(!(at_cell > au::ZERO)) {
        throw SetupError("TimeStepRule::constant_lattice_speed(): the cell size must be above 0 m");
    }
    const ConstantLatticeSpeed rule{ .step = step, .at_cell = at_cell };
    return TimeStepRule(rule);
}

TimeStepRule TimeStepRule::fixed(si::Duration step) {
    if(!(step > au::ZERO)) {
        throw SetupError("TimeStepRule::fixed(): the time step must be above 0 s");
    }
    const Fixed rule{ .step = step };
    return TimeStepRule(rule);
}

TimeStepRule TimeStepRule::from_mach(si::Speed fastest, si::LatticeMach mach) {
    if(!(fastest > au::ZERO)) {
        throw SetupError("TimeStepRule::from_mach(): the fastest speed must be above 0 m/s");
    }
    const FromMach rule{ .fastest = fastest, .mach = mach };
    return TimeStepRule(rule);
}

si::Duration TimeStepRule::step(si::Length cell) const {
    if(!(cell > au::ZERO)) {
        throw SetupError("TimeStepRule::step(): the cell size must be above 0 m");
    }
    const auto step_of_rule = [cell](const auto& r) { return step_for(r, cell); };
    const si::Duration time_step{ std::visit(step_of_rule, rule_) };
    return time_step;
}

si::Duration TimeStepRule::step_for(const ConstantLatticeSpeed& r, si::Length cell) {
    const auto ratio = cell / r.at_cell;
    const double cell_ratio{ au::as_raw_number(ratio) };
    const si::Duration time_step{ r.step * cell_ratio };
    return time_step;
}

si::Duration TimeStepRule::step_for(const Fixed& r, si::Length /*cell*/) {
    const si::Duration time_step{ r.step };
    return time_step;
}

si::Duration TimeStepRule::step_for(const FromMach& r, si::Length cell) {
    const auto crossing = cell / r.fastest;
    const double mach_speed{ r.mach.lattice_speed() };
    const auto scaled = crossing * mach_speed;
    const si::Duration time_step{ si::Duration(scaled) };
    return time_step;
}

std::string TimeStepRule::describe() const {
    const auto describe_one = [](const auto& r) { return describe_rule(r); };
    const std::string description{ std::visit(describe_one, rule_) };
    return description;
}

std::string TimeStepRule::describe_rule(const ConstantLatticeSpeed& r) {
    const double step_s{ r.step.in(au::seconds) };
    const double at_cell_m{ r.at_cell.in(au::meters) };
    const auto speed = r.at_cell / r.step;
    const double speed_m_s{ speed.in(au::meters / au::second) };
    char text[describe_capacity]{};
    const int written{ std::snprintf(text, sizeof(text), "time step %g s per %g m of cell (lattice speed %g m/s)", step_s, at_cell_m, speed_m_s) };
    if(written < 0 || static_cast<std::size_t>(written) >= describe_capacity) {
        throw SetupError("TimeStepRule::describe(): the description does not fit its text buffer");
    }
    const std::string description{ text };
    return description;
}

std::string TimeStepRule::describe_rule(const Fixed& r) {
    const double step_s{ r.step.in(au::seconds) };
    char text[describe_capacity]{};
    const int written{ std::snprintf(text, sizeof(text), "time step %g s at every cell size", step_s) };
    if(written < 0 || static_cast<std::size_t>(written) >= describe_capacity) {
        throw SetupError("TimeStepRule::describe(): the description does not fit its text buffer");
    }
    const std::string description{ text };
    return description;
}

std::string TimeStepRule::describe_rule(const FromMach& r) {
    const double fastest_m_s{ r.fastest.in(au::meters / au::second) };
    const double mach_speed{ r.mach.lattice_speed() };
    char text[describe_capacity]{};
    const int written{ std::snprintf(text, sizeof(text), "time step for %g m/s at %g cells per step", fastest_m_s, mach_speed) };
    if(written < 0 || static_cast<std::size_t>(written) >= describe_capacity) {
        throw SetupError("TimeStepRule::describe(): the description does not fit its text buffer");
    }
    const std::string description{ text };
    return description;
}
