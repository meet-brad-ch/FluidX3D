#include "setup/simulation/step_schedule.hpp"

#include <algorithm>

std::size_t StepSchedule::add(std::uint64_t interval) {
    intervals_.push_back(std::max<std::uint64_t>(interval, 1u));
    return intervals_.size() - 1u;
}

bool StepSchedule::is_due(std::size_t task, std::uint64_t t) const {
    return t % intervals_.at(task) == 0u;
}

std::uint64_t StepSchedule::steps_to_next(std::uint64_t t, std::uint64_t end) const {
    std::uint64_t steps = end - t;
    for(const std::uint64_t interval : intervals_) steps = std::min(steps, interval - t % interval);
    return steps;
}
