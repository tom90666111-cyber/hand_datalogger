#ifndef TWINCAT_TALKER_CURRENT_JOB_MONITOR_HPP
#define TWINCAT_TALKER_CURRENT_JOB_MONITOR_HPP

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace twincat_talker {

enum class JobState {
    WAITING_FOR_RUNNING,
    RUNNING,
    COMPLETED
};

class CurrentJobMonitor {
public:
    CurrentJobMonitor(std::int16_t trigger_job, std::size_t completion_debounce_count)
        : trigger_job_(trigger_job),
          completion_debounce_count_(completion_debounce_count),
          running_confirmed_(false),
          completion_count_(0),
          final_job_(trigger_job) {
        if (completion_debounce_count_ == 0) {
            throw std::invalid_argument("completion_debounce_count must be greater than zero");
        }
    }

    JobState update(std::int16_t current_job) {
        final_job_ = current_job;

        if (current_job == trigger_job_) {
            running_confirmed_ = true;
            completion_count_ = 0;
            return JobState::RUNNING;
        }

        if (!running_confirmed_) {
            return JobState::WAITING_FOR_RUNNING;
        }

        ++completion_count_;
        if (completion_count_ >= completion_debounce_count_) {
            return JobState::COMPLETED;
        }
        return JobState::RUNNING;
    }

    bool runningConfirmed() const { return running_confirmed_; }
    std::size_t completionCount() const { return completion_count_; }
    std::int16_t finalJob() const { return final_job_; }

private:
    std::int16_t trigger_job_;
    std::size_t completion_debounce_count_;
    bool running_confirmed_;
    std::size_t completion_count_;
    std::int16_t final_job_;
};

}  // namespace twincat_talker

#endif  // TWINCAT_TALKER_CURRENT_JOB_MONITOR_HPP
