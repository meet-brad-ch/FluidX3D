#pragma once

#include "setup/core/types.hpp"
#include "setup/simulation/step_schedule.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/units.hpp"
#include "lbm.hpp"
#include <functional>
#include <utility>
#include <vector>

/// @file runner.hpp
/// @brief Runs the LBM for a simulated time and calls tasks at intervals of simulated time (Runner).

/// @brief Runs the LBM for a simulated time, calling tasks at regular intervals of simulated time (Simulation runs
/// through it: Simulation::every(), run_for(), run(), stop()).
///
/// A task gets the simulated time since the start. It is called from the start on (at 0 s) or first after one
/// interval (FirstCall::AFTER_ONE_INTERVAL), then after each interval; an interval is at least one time step. At the
/// end of run_for() a task is called only when its interval divides the run's length. The times become time steps
/// with the simulation's converter (rounded to the nearest step, as the core's to_ulong()).
/// @code
/// using namespace si::symbols;
/// Runner runner(lbm, converter);
/// runner.every(0.1 * s, [&](si::Duration t) { wave.update(t); })
///       .run_for(20.0 * s);
/// @endcode
class Runner {
public:
    /// A task, called with the simulated time since the start (the time steps so far times the time step). It is the
    /// user's own function, so it is a std::function: one call per due task and time step, never per cell.
    using Task = std::function<void(si::Duration time)>;

    /// When a task is called the first time.
    enum class FirstCall {
        AT_ZERO,           ///< at the start (0 s), then after each interval
        AFTER_ONE_INTERVAL ///< after the first interval, not at the start
    };

    /// @brief Makes the runner of an LBM. It keeps a reference to the LBM, which must outlive it.
    /// @param lbm the LBM to run
    /// @param converter the simulation's conversions, for the times and the time steps
    Runner(LBM& lbm, const si::LatticeConverter& converter) : lbm_(lbm), converter_(converter) {}

    /// @brief Calls a task every interval of simulated time.
    /// @param interval the interval; less than half a time step counts as one step
    /// @param task the function to call, with the simulated time since the start
    /// @param first when the task is called the first time
    /// @return this runner
    Runner& every(si::Duration interval, Task task, FirstCall first = FirstCall::AT_ZERO) {
        const std::uint64_t rounded_steps{ converter_.steps(interval) };
        // the contract above: an interval under half a time step rounds to 0 steps and runs every time step
        const std::uint64_t steps{ rounded_steps == 0u ? std::uint64_t{ 1u } : rounded_steps };
        const bool skip_zero{ first == FirstCall::AFTER_ONE_INTERVAL };
        Runner& self{ add(steps, std::move(task), skip_zero) };
        return self;
    }

    /// @brief Calls a task every time step.
    /// @param task the function to call, with the simulated time since the start
    /// @return this runner
    Runner& every_step(Task task) {
        Runner& self{ add(1u, std::move(task)) };
        return self;
    }

    /// @brief Runs a simulated time from now. The first run prints the time and its time steps.
    /// @param time the simulated time, rounded to the nearest time step
    void run_for(si::Duration time) {
        const std::uint64_t steps{ converter_.steps(time) };
        if(lbm_.get_t() == 0u) {
            const float seconds{ static_cast<float>(time.in(au::seconds)) };
            print_info("Simulated time " + to_string(seconds, 3u) + " s = " + to_string(steps) + " time steps");
        }
        const std::uint64_t end{ lbm_.get_t() + steps };
        run_until(end);
    }

    /// @brief Runs until a task calls stop() (with interactive graphics: until the window is closed).
    void run() { run_until(max_ulong); }

    /// @brief Ends run() or run_for() once the tasks of this time step are done; call it from a task.
    void stop() { stopped_ = true; }

private:
    LBM& lbm_;                            ///< the LBM to run (not owned)
    si::LatticeConverter converter_;      ///< the simulation's conversions
    StepSchedule schedule_{};             ///< the tasks' intervals in time steps
    std::vector<Task> tasks_{};           ///< the tasks, indexed as in schedule_
    uint64_t last_task_step_ = max_ulong; ///< the time step at which the tasks were last called
    bool stopped_ = false;                ///< true after stop() in this run

    /// @brief Adds a task.
    /// @param interval its interval in time steps, at least 1
    /// @param task the function to call when the task is due
    /// @param skip_zero true when the task is not called at the time step 0
    /// @return this runner
    Runner& add(uint64_t interval, Task task, bool skip_zero = false) {
        schedule_.add(interval, skip_zero);
        tasks_.push_back(std::move(task));
        return *this;
    }

    /// @brief Calls the tasks that are due at a time step.
    /// @param t the time step
    void call_due_tasks(uint64_t t) {
        const si::Duration time{ converter_.time(t) };
        for(std::size_t i = 0u; i < tasks_.size(); i++) {
            const bool due{ schedule_.is_due(i, t) };
            if(due) {
                tasks_[i](time);
            }
        }
    }

    /// @brief Runs until a time step, calling the tasks that are due, or until stop().
    /// @param end the time step to run until
    void run_until(uint64_t end) {
        stopped_ = false;
        lbm_.run(0u, end); // initializes the LBM at its first run
        // the bound: each pass runs at least one time step (steps_to_next() >= 1 while t < end), so the loop ends at
        // the time step end, or earlier at stop()
        while(true) {
            const uint64_t t{ lbm_.get_t() };
            if(t != last_task_step_) { // not again when run_for() continues where the last one ended
                last_task_step_ = t;
                call_due_tasks(t);
            }
            if(stopped_ || t >= end) {
                return;
            }
            const uint64_t next{ schedule_.steps_to_next(t, end) };
            lbm_.run(next, end);
        }
    }
};
