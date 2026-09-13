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
    /// @brief A task every interval time steps.
    /// @param interval the interval in time steps (at least 1)
    /// @return the task's index
    std::size_t add(std::uint64_t interval);

    /// @param task the task's index
    /// @param t a time step
    /// @return whether the task is due at that time step
    bool is_due(std::size_t task, std::uint64_t t) const;

    /// @brief The time steps until a task is next due.
    /// @param t the current time step
    /// @param end the time step the run ends at (end > t)
    /// @return the time steps from t to the next due time step, at most end - t
    std::uint64_t steps_to_next(std::uint64_t t, std::uint64_t end) const;

    /// @return the number of tasks
    std::size_t size() const { return intervals_.size(); }

private:
    std::vector<std::uint64_t> intervals_; ///< the tasks' intervals, in time steps
};
