#ifndef TWINCAT_TALKER_TRAJECTORY_UPLOADER_HPP
#define TWINCAT_TALKER_TRAJECTORY_UPLOADER_HPP

#include "twincat_talker/trajectory_generator.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace twincat_talker {

const std::size_t kPlcAxisCount = 16;
const std::size_t kPlcMaximumTrajectoryRows = 60001;

struct AdsTrajectoryConfiguration {
    std::string remote_ip;
    std::string remote_ams_net_id;
    std::string local_ams_net_id;
    std::uint16_t port;
    std::string current_position_variable;
    std::string trajectory_data_variable;
    std::string trajectory_size_variable;
    std::string current_job_variable;

    AdsTrajectoryConfiguration();
};

class TrajectoryUploader {
public:
    explicit TrajectoryUploader(const AdsTrajectoryConfiguration& configuration);
    ~TrajectoryUploader();

    void connect();
    bool isConnected() const;

    std::array<double, kPlcAxisCount> readCurrentPositions() const;
    void upload(const std::vector<TrajectoryPoint>& trajectory,
                std::size_t target_axis,
                const std::array<double, kPlcAxisCount>& hold_positions);
    std::uint32_t readUploadedLastIndex() const;

    void trigger(std::int16_t trigger_job);
    std::int16_t readCurrentJob() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace twincat_talker

#endif  // TWINCAT_TALKER_TRAJECTORY_UPLOADER_HPP
