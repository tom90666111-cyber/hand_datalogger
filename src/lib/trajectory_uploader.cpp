#include "twincat_talker/trajectory_uploader.hpp"

#include "AdsLib.h"
#include "AdsVariable.h"

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

namespace twincat_talker {
namespace {

const std::size_t kPlcTrajectoryValueCount =
    kPlcMaximumTrajectoryRows * kPlcAxisCount;

void requireNonEmpty(const std::string& value, const char* field_name) {
    if (value.empty()) {
        throw std::invalid_argument(std::string(field_name) + " must not be empty");
    }
}

}  // namespace

AdsTrajectoryConfiguration::AdsTrajectoryConfiguration()
    : remote_ip("192.168.203.111"),
      remote_ams_net_id("127.0.0.1.2.2"),
      local_ams_net_id("192.168.10.1.1.3"),
      port(AMSPORT_R0_PLC_TC3),
      current_position_variable("VariableMAIN.CurrentPositionReal"),
      trajectory_data_variable("MyTRAJ.database_read_LREAL2"),
      trajectory_size_variable("MyTRAJ.SIZE_POS_TRAJ"),
      current_job_variable("VariableMAIN.CurrentJob") {}

struct TrajectoryUploader::Impl {
    explicit Impl(const AdsTrajectoryConfiguration& value)
        : configuration(value), connected(false) {}

    AdsTrajectoryConfiguration configuration;
    bool connected;
    std::unique_ptr<AdsDevice> route;
    std::unique_ptr<AdsVariable<std::array<double, kPlcAxisCount> > > current_positions;
    std::unique_ptr<AdsVariable<std::array<double, kPlcTrajectoryValueCount> > >
        trajectory_data;
    std::unique_ptr<AdsVariable<std::uint32_t> > trajectory_size;
    std::unique_ptr<AdsVariable<std::int16_t> > current_job;
};

TrajectoryUploader::TrajectoryUploader(const AdsTrajectoryConfiguration& configuration)
    : impl_(new Impl(configuration)) {
    requireNonEmpty(configuration.remote_ip, "remote_ip");
    requireNonEmpty(configuration.remote_ams_net_id, "remote_ams_net_id");
    requireNonEmpty(configuration.local_ams_net_id, "local_ams_net_id");
    requireNonEmpty(configuration.current_position_variable,
                    "current_position_variable");
    requireNonEmpty(configuration.trajectory_data_variable, "trajectory_data_variable");
    requireNonEmpty(configuration.trajectory_size_variable, "trajectory_size_variable");
    requireNonEmpty(configuration.current_job_variable, "current_job_variable");
}

TrajectoryUploader::~TrajectoryUploader() {}

void TrajectoryUploader::connect() {
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
    impl_->current_positions.reset(
        new AdsVariable<std::array<double, kPlcAxisCount> >(
            *impl_->route, impl_->configuration.current_position_variable));
    impl_->trajectory_data.reset(
        new AdsVariable<std::array<double, kPlcTrajectoryValueCount> >(
            *impl_->route, impl_->configuration.trajectory_data_variable));
    impl_->trajectory_size.reset(new AdsVariable<std::uint32_t>(
        *impl_->route, impl_->configuration.trajectory_size_variable));
    impl_->current_job.reset(new AdsVariable<std::int16_t>(
        *impl_->route, impl_->configuration.current_job_variable));
    impl_->connected = true;
}

bool TrajectoryUploader::isConnected() const {
    return impl_->connected;
}

std::array<double, kPlcAxisCount> TrajectoryUploader::readCurrentPositions() const {
    if (!impl_->connected) {
        throw std::logic_error("ADS uploader is not connected");
    }
    return *impl_->current_positions;
}

void TrajectoryUploader::upload(
    const std::vector<TrajectoryPoint>& trajectory,
    std::size_t target_axis,
    const std::array<double, kPlcAxisCount>& hold_positions) {
    if (!impl_->connected) {
        throw std::logic_error("ADS uploader is not connected");
    }
    if (trajectory.empty() || trajectory.size() > kPlcMaximumTrajectoryRows) {
        throw std::invalid_argument("trajectory size is outside PLC capacity");
    }
    if (target_axis >= kPlcAxisCount) {
        throw std::invalid_argument("target_axis is outside PLC axis range");
    }
    for (std::size_t axis = 0; axis < kPlcAxisCount; ++axis) {
        if (!std::isfinite(hold_positions[axis])) {
            throw std::invalid_argument("hold position contains a non-finite value");
        }
    }

    std::unique_ptr<std::array<double, kPlcTrajectoryValueCount> > buffer(
        new std::array<double, kPlcTrajectoryValueCount>());

    for (std::size_t row = 0; row < trajectory.size(); ++row) {
        for (std::size_t axis = 0; axis < kPlcAxisCount; ++axis) {
            (*buffer)[row * kPlcAxisCount + axis] = hold_positions[axis];
        }
        (*buffer)[row * kPlcAxisCount + target_axis] = trajectory[row].target;
    }

    *impl_->trajectory_data = *buffer;
    *impl_->trajectory_size = static_cast<std::uint32_t>(trajectory.size() - 1);

    const std::uint32_t uploaded_last_index = readUploadedLastIndex();
    if (uploaded_last_index != trajectory.size() - 1) {
        throw std::runtime_error("PLC trajectory size readback does not match upload");
    }
}

std::uint32_t TrajectoryUploader::readUploadedLastIndex() const {
    if (!impl_->connected) {
        throw std::logic_error("ADS uploader is not connected");
    }
    return *impl_->trajectory_size;
}

void TrajectoryUploader::trigger(std::int16_t trigger_job) {
    if (!impl_->connected) {
        throw std::logic_error("ADS uploader is not connected");
    }
    *impl_->current_job = trigger_job;
}

std::int16_t TrajectoryUploader::readCurrentJob() const {
    if (!impl_->connected) {
        throw std::logic_error("ADS uploader is not connected");
    }
    return *impl_->current_job;
}

}  // namespace twincat_talker
