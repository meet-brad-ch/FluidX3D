#include "setup/simulation/step_schedule.hpp"

#include "setup/core/setup_error.hpp"

#include <algorithm>

std::size_t StepSchedule::add(std::uint64_t interval, bool skip_zero) {
    if(interval == 0u) {
        throw SetupError("StepSchedule::add(): a task's interval must be at least 1 time step");
    }
    intervals_.push_back(interval);
    skip_zero_.push_back(skip_zero);
    const std::size_t index{ intervals_.size() - 1u };
    return index;
}

bool StepSchedule::is_due(std::size_t task, std::uint64_t t) const {
    if(t == 0u && skip_zero_.at(task)) {
        return false;
    }
    const std::uint64_t interval{ intervals_.at(task) };
    const bool due{ t % interval == 0u };
    return due;
}

std::uint64_t StepSchedule::steps_to_next(std::uint64_t t, std::uint64_t end) const {
    std::uint64_t steps{ end - t };
    for(const std::uint64_t interval : intervals_) {
        const std::uint64_t to_due{ interval - t % interval };
        steps = std::min(steps, to_due);
    }
    return steps;
}
