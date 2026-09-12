#include "setup/simulation/step_schedule.hpp"

#include <gtest/gtest.h>

TEST(StepSchedule, TasksAreDueFromTheStartAtMultiplesOfTheirInterval) {
    StepSchedule schedule;
    const std::size_t every_3 = schedule.add(3u);
    EXPECT_TRUE(schedule.is_due(every_3, 0u));
    EXPECT_FALSE(schedule.is_due(every_3, 1u));
    EXPECT_FALSE(schedule.is_due(every_3, 5u));
    EXPECT_TRUE(schedule.is_due(every_3, 6u));
}

TEST(StepSchedule, RunsToTheNextDueTask) {
    StepSchedule schedule;
    schedule.add(3u);
    schedule.add(5u);
    EXPECT_EQ(schedule.steps_to_next(0u, 100u), 3u);  // to 3
    EXPECT_EQ(schedule.steps_to_next(3u, 100u), 2u);  // to 5
    EXPECT_EQ(schedule.steps_to_next(5u, 100u), 1u);  // to 6
    EXPECT_EQ(schedule.steps_to_next(14u, 100u), 1u); // to 15, due for both
}

TEST(StepSchedule, StopsAtTheEnd) {
    StepSchedule schedule;
    schedule.add(10u);
    EXPECT_EQ(schedule.steps_to_next(95u, 98u), 3u);
}

TEST(StepSchedule, WithoutTasksRunsToTheEnd) {
    EXPECT_EQ(StepSchedule().steps_to_next(7u, 100u), 93u);
}

TEST(StepSchedule, AnIntervalBelowOneStepIsEveryStep) {
    StepSchedule schedule;
    const std::size_t task = schedule.add(0u);
    EXPECT_TRUE(schedule.is_due(task, 7u));
    EXPECT_EQ(schedule.steps_to_next(7u, 100u), 1u);
}

TEST(StepSchedule, VisitsEveryDueStepOfARun) { // as Runner loops: tasks at the due steps, then run to the next
    StepSchedule schedule;
    const std::size_t every_4 = schedule.add(4u);
    const std::size_t every_6 = schedule.add(6u);
    std::vector<std::uint64_t> visited, due_4, due_6;
    const std::uint64_t end = 13u;
    for(std::uint64_t t = 0u;; t += schedule.steps_to_next(t, end)) {
        visited.push_back(t);
        if(schedule.is_due(every_4, t)) due_4.push_back(t);
        if(schedule.is_due(every_6, t)) due_6.push_back(t);
        if(t >= end) break;
    }
    EXPECT_EQ(visited, (std::vector<std::uint64_t>{ 0u, 4u, 6u, 8u, 12u, 13u })); // the end too
    EXPECT_EQ(due_4, (std::vector<std::uint64_t>{ 0u, 4u, 8u, 12u }));
    EXPECT_EQ(due_6, (std::vector<std::uint64_t>{ 0u, 6u, 12u }));
}
