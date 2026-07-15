#include "twincat_talker/current_job_monitor.hpp"

#include <gtest/gtest.h>

TEST(CurrentJobMonitor, RequiresRunningConfirmationBeforeCompletion) {
    twincat_talker::CurrentJobMonitor monitor(114, 3);

    EXPECT_EQ(twincat_talker::JobState::WAITING_FOR_RUNNING, monitor.update(0));
    EXPECT_EQ(twincat_talker::JobState::WAITING_FOR_RUNNING, monitor.update(200));
    EXPECT_FALSE(monitor.runningConfirmed());
}

TEST(CurrentJobMonitor, CompletesAfterDebouncedNonTriggerState) {
    twincat_talker::CurrentJobMonitor monitor(114, 3);

    EXPECT_EQ(twincat_talker::JobState::RUNNING, monitor.update(114));
    EXPECT_EQ(twincat_talker::JobState::RUNNING, monitor.update(0));
    EXPECT_EQ(twincat_talker::JobState::RUNNING, monitor.update(0));
    EXPECT_EQ(twincat_talker::JobState::COMPLETED, monitor.update(0));
    EXPECT_TRUE(monitor.runningConfirmed());
    EXPECT_EQ(0, monitor.finalJob());
}

TEST(CurrentJobMonitor, TriggerStateResetsCompletionDebounce) {
    twincat_talker::CurrentJobMonitor monitor(114, 2);

    EXPECT_EQ(twincat_talker::JobState::RUNNING, monitor.update(114));
    EXPECT_EQ(twincat_talker::JobState::RUNNING, monitor.update(0));
    EXPECT_EQ(twincat_talker::JobState::RUNNING, monitor.update(114));
    EXPECT_EQ(0u, monitor.completionCount());
    EXPECT_EQ(twincat_talker::JobState::RUNNING, monitor.update(5));
    EXPECT_EQ(twincat_talker::JobState::COMPLETED, monitor.update(5));
}

TEST(CurrentJobMonitor, RejectsZeroDebounceCount) {
    EXPECT_THROW(twincat_talker::CurrentJobMonitor(114, 0), std::invalid_argument);
}
