#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

/// @brief Periodic tasks on the LBM's clock of time steps, for Runner: which tasks are due at a time step, and how many
/// time steps to run until the next one is.
///
/// A task with the interval n is due at the time steps 0, n, 2n, ... (counted from the LBM's start), or from n on when
/// it skips the time step 0.
class StepSchedule {
public:
    /// @brief Adds a task that is due every interval time steps.
    /// @param interval the interval in time steps, at least 1
    /// @param skip_zero true when the task is not due at the time step 0 (its first call comes after one interval)
    /// @return the task's index, counted from 0 in the order of add()
    /// @throws SetupError for an interval of 0
    std::size_t add(std::uint64_t interval, bool skip_zero = false);

    /// @brief Tells whether a task is due at a time step.
    /// @param task the task's index (from add())
    /// @param t a time step, counted from the LBM's start
    /// @return true when t is a multiple of the task's interval (and not 0 for a task that skips the time step 0)
    /// @throws std::out_of_range when there is no task with that index
    bool is_due(std::size_t task, std::uint64_t t) const;

    /// @brief Gives the time steps until the next time step at which a task is due, or until the end of the run.
    /// @param t the current time step
    /// @param end the time step at which the run ends, more than t
    /// @return the time steps from t to the next due time step of any task after t, at most end - t: a task due at t
    /// is next due one interval later, so the result is at least 1 for t below end. skip_zero is not used here.
    std::uint64_t steps_to_next(std::uint64_t t, std::uint64_t end) const;

    /// @return the number of tasks
    std::size_t size() const { return intervals_.size(); }

private:
    std::vector<std::uint64_t> intervals_; ///< the tasks' intervals in time steps, by task index
    std::vector<bool> skip_zero_{};        ///< per task index: true when the task is not due at the time step 0
};
