#ifndef TWINCAT_TALKER_HAND_STREAMER_HPP
#define TWINCAT_TALKER_HAND_STREAMER_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace twincat_talker {

// Number of hand target channels exposed by MAIN.ROS_A.
const std::size_t kHandChannelCount = 16;

struct AdsHandConfiguration {
    // Connection settings are intentionally empty: streaming to the PLC must
    // never fall back to a hardcoded address, so the operator has to provide
    // them explicitly (launch arguments or ROS parameters).
    std::string remote_ip;
    std::string remote_ams_net_id;
    std::string local_ams_net_id;
    std::uint16_t port;
    std::string ros_control_variable;
    std::string hand_targets_variable;
    std::string current_job_variable;
    std::int16_t stream_job;

    AdsHandConfiguration();
};

// Streams single-channel target values to the PLC hand interface
// (MAIN.ROS_A + hand job trigger) in real time.
//
// The streaming model differs from the trajectory upload path: the PLC only
// holds the current 16-channel target snapshot, so a trajectory must be
// written sample by sample from the ROS side. Every writeSample() call also
// writes the hand job value, mirroring the teleoperation node behaviour.
class HandTrajectoryStreamer {
public:
    explicit HandTrajectoryStreamer(const AdsHandConfiguration& configuration);
    ~HandTrajectoryStreamer();

    void connect();
    bool isConnected() const;

    // Writes MAIN.ROSControl = true so the PLC accepts ROS hand targets.
    void enableRosControl();

    // Reads MAIN.ROS_A. Used as the hold values for non-excited channels;
    // connect() also seeds the internal write buffer with this snapshot.
    std::array<float, kHandChannelCount> readCurrentTargets() const;

    // Overwrites one channel of the internal buffer, writes the full array to
    // MAIN.ROS_A and writes the hand job value (default 210).
    void writeSample(std::size_t channel, float value);

    std::int16_t readCurrentJob() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace twincat_talker

#endif  // TWINCAT_TALKER_HAND_STREAMER_HPP
