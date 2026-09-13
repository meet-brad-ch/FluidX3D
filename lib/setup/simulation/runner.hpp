#pragma once

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/unit_scale.hpp"
#include "setup/simulation/step_schedule.hpp"
#include "lbm.hpp"
#include <functional>
#include <utility>
#include <vector>

/// @brief Runs the LBM for a simulated time, calling tasks at regular intervals of simulated time (Simulation runs
/// through it: Simulation::every(), run_for(), run(), stop()).
///
/// A task gets the simulated time since the start and is called from the start on (at 0 s), then after each interval
/// (at least every time step); at the end of run_for() only when the interval divides its length.
/// @code
/// Runner runner(lbm, units);
/// runner.every(0.1_s, [&](Duration t) { wave.update(t); })
///       .run_for(20.0_s);
/// @endcode
class Runner {
public:
    using Task = std::function<void(Duration time)>;

    Runner(LBM& lbm, const UnitScale& unit_scale) : lbm_(lbm), units_(unit_scale) {}

    /// Calls the task every interval of simulated time.
    Runner& every(Duration interval, Task task) { return add(units_.time_steps(interval), std::move(task)); }

    /// Calls the task every time step.
    Runner& every_step(Task task) { return add(1u, std::move(task)); }

    /// Runs this much simulated time from now.
    void run_for(Duration time) {
        const uint64_t steps = units_.time_steps(time);
        if(lbm_.get_t() == 0u) print_info("Simulated time " + to_string(time.si(), 3u) + " s = " + to_string(steps) + " time steps");
        run_until(lbm_.get_t() + steps);
    }

    /// Runs until a task calls stop() (with interactive graphics: until the window is closed).
    void run() { run_until(max_ulong); }

    /// Ends run() or run_for() once the tasks of this time step are done; call it from a task.
    void stop() { stopped_ = true; }

private:
    LBM& lbm_;
    UnitScale units_;
    StepSchedule schedule_;
    std::vector<Task> tasks_; ///< indexed as in schedule_
    uint64_t last_task_step_ = max_ulong; ///< the time step at which the tasks were last called
    bool stopped_ = false;

    Runner& add(uint64_t interval, Task task) {
        schedule_.add(interval);
        tasks_.push_back(std::move(task));
        return *this;
    }

    void run_until(uint64_t end) {
        stopped_ = false;
        lbm_.run(0u, end); // initializes the LBM at its first run
        while(true) {
            const uint64_t t = lbm_.get_t();
            if(t != last_task_step_) { // not again when run_for() continues where the last one ended
                last_task_step_ = t;
                const Duration time = units_.si_time(t);
                for(std::size_t i = 0u; i < tasks_.size(); i++) {
                    if(schedule_.is_due(i, t)) tasks_[i](time);
                }
            }
            if(stopped_ || t >= end) return;
            lbm_.run(schedule_.steps_to_next(t, end), end);
        }
    }
};
