#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

/// @brief Periodic tasks on the LBM's clock of time steps, for Runner: which tasks are due at a time step, and how many
/// time steps to run until the next one is.
///
/// A task with the interval n is due at the time steps 0, n, 2n, ... (counted from the LBM's start).
class StepSchedule {
public:
    /// A task every interval time steps (at least 1); returns its index.
    std::size_t add(std::uint64_t interval);

    /// Whether the task is due at time step t.
    bool is_due(std::size_t task, std::uint64_t t) const;

    /// The time steps from t to the next time step at which a task is due, at most end - t (end > t).
    std::uint64_t steps_to_next(std::uint64_t t, std::uint64_t end) const;

    std::size_t size() const { return intervals_.size(); }

private:
    std::vector<std::uint64_t> intervals_;
};
