#include "twincat_talker/hand_streamer.hpp"

#include "AdsLib.h"
#include "AdsVariable.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace twincat_talker {
namespace {

void requireNonEmpty(const std::string& value, const char* field_name) {
    if (value.empty()) {
        throw std::invalid_argument(std::string(field_name) + " must not be empty");
    }
}

}  // namespace

AdsHandConfiguration::AdsHandConfiguration()
    : remote_ip(),
      remote_ams_net_id(),
      local_ams_net_id(),
      port(AMSPORT_R0_PLC_TC3),
      ros_control_variable("MAIN.ROSControl"),
      hand_targets_variable("MAIN.ROS_A"),
      current_job_variable("VariableMAIN.CurrentJob"),
      stream_job(210) {}

struct HandTrajectoryStreamer::Impl {
    explicit Impl(const AdsHandConfiguration& value)
        : configuration(value), connected(false), target_buffer() {}

    AdsHandConfiguration configuration;
    bool connected;
    std::unique_ptr<AdsDevice> route;
    std::unique_ptr<AdsVariable<bool> > ros_control;
    std::unique_ptr<AdsVariable<std::array<float, kHandChannelCount> > > hand_targets;
    std::unique_ptr<AdsVariable<std::int16_t> > current_job;
    std::array<float, kHandChannelCount> target_buffer;
};

HandTrajectoryStreamer::HandTrajectoryStreamer(const AdsHandConfiguration& configuration)
    : impl_(new Impl(configuration)) {
    requireNonEmpty(configuration.remote_ip, "remote_ip");
    requireNonEmpty(configuration.remote_ams_net_id, "remote_ams_net_id");
    requireNonEmpty(configuration.local_ams_net_id, "local_ams_net_id");
    requireNonEmpty(configuration.ros_control_variable, "ros_control_variable");
    requireNonEmpty(configuration.hand_targets_variable, "hand_targets_variable");
    requireNonEmpty(configuration.current_job_variable, "current_job_variable");
}

HandTrajectoryStreamer::~HandTrajectoryStreamer() {}

void HandTrajectoryStreamer::connect() {
    if (impl_->connected) {
        return;
    }

    const AmsNetId local_net_id(impl_->configuration.local_ams_net_id);
    const AmsNetId remote_net_id(impl_->configuration.remote_ams_net_id);

    AdsSetLocalAddress(local_net_id);
    const long route_result =
        AdsAddRoute(remote_net_id, impl_->configuration.remote_ip.c_str());
    if (route_result != 0) {
        throw std::runtime_error("AdsAddRoute failed with error code " +
                                 std::to_string(route_result));
    }

    impl_->route.reset(new AdsDevice(impl_->configuration.remote_ip,
                                     remote_net_id,
                                     impl_->configuration.port));
    impl_->ros_control.reset(new AdsVariable<bool>(
        *impl_->route, impl_->configuration.ros_control_variable));
    impl_->hand_targets.reset(
        new AdsVariable<std::array<float, kHandChannelCount> >(
            *impl_->route, impl_->configuration.hand_targets_variable));
    impl_->current_job.reset(new AdsVariable<std::int16_t>(
        *impl_->route, impl_->configuration.current_job_variable));
    impl_->target_buffer = *impl_->hand_targets;
    impl_->connected = true;
}

bool HandTrajectoryStreamer::isConnected() const {
    return impl_->connected;
}

void HandTrajectoryStreamer::enableRosControl() {
    if (!impl_->connected) {
        throw std::logic_error("ADS hand streamer is not connected");
    }
    *impl_->ros_control = true;
}

std::array<float, kHandChannelCount> HandTrajectoryStreamer::readCurrentTargets() const {
    if (!impl_->connected) {
        throw std::logic_error("ADS hand streamer is not connected");
    }
    return *impl_->hand_targets;
}

void HandTrajectoryStreamer::writeSample(std::size_t channel, float value) {
    if (!impl_->connected) {
        throw std::logic_error("ADS hand streamer is not connected");
    }
    if (channel >= kHandChannelCount) {
        throw std::invalid_argument("hand channel is outside the 0-15 range");
    }
    if (!std::isfinite(value)) {
        throw std::invalid_argument("hand target value is not finite");
    }

    impl_->target_buffer[channel] = value;
    *impl_->hand_targets = impl_->target_buffer;
    *impl_->current_job = impl_->configuration.stream_job;
}

std::int16_t HandTrajectoryStreamer::readCurrentJob() const {
    if (!impl_->connected) {
        throw std::logic_error("ADS hand streamer is not connected");
    }
    return *impl_->current_job;
}

}  // namespace twincat_talker
