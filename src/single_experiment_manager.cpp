#include "twincat_talker/hand_streamer.hpp"
#include "twincat_talker/trajectory_generator.hpp"

#include <boost/bind.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

namespace twincat_talker {
namespace {

std::string expandUserPath(const std::string& path) {
    if (path.size() >= 2 && path[0] == '~' && path[1] == '/') {
        const char* home = std::getenv("HOME");
        if (home == NULL || std::string(home).empty()) {
            throw std::runtime_error("HOME is not set; cannot expand output path");
        }
        return std::string(home) + path.substr(1);
    }
    return path;
}

bool isDirectory(const std::string& path) {
    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

void createDirectories(const std::string& raw_path) {
    const std::string path = expandUserPath(raw_path);
    if (path.empty()) {
        throw std::invalid_argument("output directory must not be empty");
    }
    if (isDirectory(path)) {
        return;
    }

    std::string current;
    if (path[0] == '/') {
        current = "/";
    }

    std::stringstream stream(path);
    std::string component;
    while (std::getline(stream, component, '/')) {
        if (component.empty()) {
            continue;
        }
        if (!current.empty() && current[current.size() - 1] != '/') {
            current += '/';
        }
        current += component;

        if (!isDirectory(current) && mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
            throw std::runtime_error("cannot create directory: " + current);
        }
    }
}

std::string timestampString() {
    const std::time_t now = std::time(NULL);
    struct tm local_time;
    localtime_r(&now, &local_time);
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &local_time);
    return std::string(buffer);
}

std::string sanitizeName(const std::string& name) {
    std::string result;
    for (std::size_t i = 0; i < name.size(); ++i) {
        const unsigned char character = static_cast<unsigned char>(name[i]);
        if (std::isalnum(character) || character == '_' || character == '-') {
            result.push_back(static_cast<char>(character));
        } else {
            result.push_back('_');
        }
    }
    return result.empty() ? "experiment" : result;
}

std::string createUniqueExperimentDirectory(const std::string& root,
                                            const std::string& experiment_name) {
    const std::string expanded_root = expandUserPath(root);
    createDirectories(expanded_root);

    const std::string base = expanded_root + "/" + timestampString() + "_" +
                             sanitizeName(experiment_name);
    std::string candidate = base;
    for (int suffix = 1; isDirectory(candidate); ++suffix) {
        candidate = base + "_" + std::to_string(suffix);
    }
    createDirectories(candidate);
    return candidate;
}

void writeExperimentYaml(const std::string& path,
                         const TrajectoryParameters& trajectory,
                         int target_joint,
                         bool stream_to_plc,
                         double write_rate_hz,
                         bool ros_control_enabled,
                         const AdsHandConfiguration& ads) {
    std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        throw std::runtime_error("cannot write experiment metadata: " + path);
    }
    output << std::boolalpha << std::setprecision(12);
    output << "trajectory:\n";
    output << "  waveform: \"" << trajectory.waveform << "\"\n";
    output << "  frequency_hz: " << trajectory.frequency_hz << '\n';
    output << "  amplitude: " << trajectory.amplitude << '\n';
    output << "  offset: " << trajectory.offset << '\n';
    output << "  phase_rad: " << trajectory.phase_rad << '\n';
    output << "  duration_s: " << trajectory.duration_s << '\n';
    output << "  sample_rate_hz: " << trajectory.sample_rate_hz << '\n';
    output << "  target_joint: " << target_joint << '\n';
    output << "safety:\n";
    output << "  limits_enabled: " << trajectory.limits_enabled << '\n';
    output << "  minimum_value: " << trajectory.minimum_value << '\n';
    output << "  maximum_value: " << trajectory.maximum_value << '\n';
    output << "  maximum_step: " << trajectory.maximum_step << '\n';
    output << "execution:\n";
    output << "  stream_to_plc: " << stream_to_plc << '\n';
    output << "  write_rate_hz: " << write_rate_hz << '\n';
    output << "  ros_control: " << ros_control_enabled << '\n';
    output << "  stream_job: " << ads.stream_job << '\n';
    output << "ads:\n";
    output << "  remote_ip: \"" << ads.remote_ip << "\"\n";
    output << "  remote_ams_net_id: \"" << ads.remote_ams_net_id << "\"\n";
    output << "  local_ams_net_id: \"" << ads.local_ams_net_id << "\"\n";
    output << "  port: " << ads.port << '\n';
}

struct ExperimentResult {
    std::string status;
    std::string failure_reason;
    std::size_t generated_points;
    std::size_t written_points;
    std::size_t recorded_rows;
    std::int16_t final_current_job;
    double maximum_sync_error_s;

    ExperimentResult()
        : status("failed"),
          generated_points(0),
          written_points(0),
          recorded_rows(0),
          final_current_job(0),
          maximum_sync_error_s(0.0) {}
};

void writeResultYaml(const std::string& path, const ExperimentResult& result) {
    std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        ROS_ERROR_STREAM("Cannot write result file: " << path);
        return;
    }
    output << std::boolalpha << std::setprecision(12);
    output << "status: \"" << result.status << "\"\n";
    output << "failure_reason: \"" << result.failure_reason << "\"\n";
    output << "generated_points: " << result.generated_points << '\n';
    output << "written_points: " << result.written_points << '\n';
    output << "recorded_rows: " << result.recorded_rows << '\n';
    output << "final_current_job: " << result.final_current_job << '\n';
    output << "maximum_sync_error_s: " << result.maximum_sync_error_s << '\n';
}

class RuntimeLog {
public:
    explicit RuntimeLog(const std::string& path) : output_(path.c_str()) {
        if (!output_.is_open()) {
            throw std::runtime_error("cannot open runtime log: " + path);
        }
    }

    void write(const std::string& message) {
        output_ << std::fixed << std::setprecision(6) << ros::Time::now().toSec() << ','
                << message << '\n';
        output_.flush();
        ROS_INFO_STREAM(message);
    }

private:
    std::ofstream output_;
};

class SynchronizedRecorder {
public:
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped,
                                                            sensor_msgs::JointState>
        SyncPolicy;

    SynchronizedRecorder(ros::NodeHandle& node,
                         const std::string& pose_topic,
                         const std::string& joint_topic,
                         int subscriber_queue_size,
                         int sync_queue_size,
                         double sync_tolerance_s,
                         const std::vector<int>& joint_indices,
                         int flush_interval)
        : pose_subscriber_(node, pose_topic, subscriber_queue_size),
          joint_subscriber_(node, joint_topic, subscriber_queue_size),
          synchronizer_(SyncPolicy(sync_queue_size), pose_subscriber_, joint_subscriber_),
          sync_tolerance_s_(sync_tolerance_s),
          joint_indices_(joint_indices),
          flush_interval_(flush_interval),
          ready_(false),
          write_failed_(false),
          last_sync_wall_time_s_(0.0),
          recording_(false),
          state_("WAITING_FOR_DATA"),
          current_job_(0),
          trigger_time_valid_(false),
          command_target_valid_(false),
          command_target_(std::numeric_limits<double>::quiet_NaN()),
          row_count_(0),
          maximum_sync_error_s_(0.0) {
        if (sync_tolerance_s_ <= 0.0) {
            throw std::invalid_argument("sync_tolerance_s must be greater than zero");
        }
        if (joint_indices_.size() != 2 || joint_indices_[0] < 0 || joint_indices_[1] < 0) {
            throw std::invalid_argument("exactly two non-negative joint_indices are required");
        }
        if (flush_interval_ <= 0) {
            throw std::invalid_argument("flush_interval must be greater than zero");
        }
        synchronizer_.registerCallback(
            boost::bind(&SynchronizedRecorder::callback, this, _1, _2));
    }

    ~SynchronizedRecorder() { stop(); }

    bool ready() const { return ready_.load(); }

    bool writeFailed() const { return write_failed_.load(); }

    bool dataStale(double timeout_s) const {
        if (!ready_.load()) {
            return true;
        }
        return ros::WallTime::now().toSec() - last_sync_wall_time_s_.load() > timeout_s;
    }

    void start(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        output_.open(path.c_str(), std::ios::out | std::ios::trunc);
        if (!output_.is_open()) {
            throw std::runtime_error("cannot open recorded data file: " + path);
        }
        output_ << "record_index,record_time,pose_time,joint_time,sync_error,"
                   "experiment_elapsed,experiment_state,current_job,x,y,z,qx,qy,qz,qw,"
                   "mcp,pip,cmd_target\n";
        recording_ = true;
    }

    void stop() {
        std::lock_guard<std::mutex> lock(mutex_);
        recording_ = false;
        if (output_.is_open()) {
            output_.flush();
            output_.close();
        }
    }

    void setState(const std::string& state) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = state;
    }

    void setCurrentJob(std::int16_t current_job) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_job_ = current_job;
    }

    void setCommandTarget(double command_target) {
        std::lock_guard<std::mutex> lock(mutex_);
        command_target_ = command_target;
        command_target_valid_ = true;
    }

    void setTriggerTime(const ros::Time& trigger_time) {
        std::lock_guard<std::mutex> lock(mutex_);
        trigger_time_ = trigger_time;
        trigger_time_valid_ = true;
    }

    std::size_t rowCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return row_count_;
    }

    double maximumSyncError() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return maximum_sync_error_s_;
    }

private:
    void callback(const geometry_msgs::PoseStamped::ConstPtr& pose,
                  const sensor_msgs::JointState::ConstPtr& joint) {
        const std::size_t first_index = static_cast<std::size_t>(joint_indices_[0]);
        const std::size_t second_index = static_cast<std::size_t>(joint_indices_[1]);
        if (joint->position.size() <= std::max(first_index, second_index)) {
            ROS_ERROR_THROTTLE(1.0, "JointState position array is shorter than joint_indices");
            return;
        }

        const double sync_error = std::fabs((pose->header.stamp - joint->header.stamp).toSec());
        if (sync_error > sync_tolerance_s_) {
            ROS_WARN_THROTTLE(1.0, "Skipping messages outside synchronization tolerance");
            return;
        }

        last_sync_wall_time_s_.store(ros::WallTime::now().toSec());
        ready_.store(true);
        std::lock_guard<std::mutex> lock(mutex_);
        if (!recording_ || !output_.is_open()) {
            return;
        }

        const ros::Time record_time = ros::Time::now();
        const double elapsed = trigger_time_valid_
                                   ? (record_time - trigger_time_).toSec()
                                   : std::numeric_limits<double>::quiet_NaN();
        maximum_sync_error_s_ = std::max(maximum_sync_error_s_, sync_error);

        output_ << std::fixed << std::setprecision(9) << row_count_ << ','
                << record_time.toSec() << ',' << pose->header.stamp.toSec() << ','
                << joint->header.stamp.toSec() << ',' << sync_error << ',' << elapsed << ','
                << state_ << ',' << current_job_ << ',' << pose->pose.position.x << ','
                << pose->pose.position.y << ',' << pose->pose.position.z << ','
                << pose->pose.orientation.x << ',' << pose->pose.orientation.y << ','
                << pose->pose.orientation.z << ',' << pose->pose.orientation.w << ','
                << joint->position[first_index] << ',' << joint->position[second_index] << ','
                << (command_target_valid_ ? command_target_
                                          : std::numeric_limits<double>::quiet_NaN())
                << '\n';

        ++row_count_;
        if (row_count_ % static_cast<std::size_t>(flush_interval_) == 0) {
            output_.flush();
        }
        if (!output_.good()) {
            write_failed_.store(true);
            recording_ = false;
            ROS_ERROR("Recorded data output stream failed; logging has been disabled");
        }
    }

    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_subscriber_;
    message_filters::Subscriber<sensor_msgs::JointState> joint_subscriber_;
    message_filters::Synchronizer<SyncPolicy> synchronizer_;
    double sync_tolerance_s_;
    std::vector<int> joint_indices_;
    int flush_interval_;
    std::atomic<bool> ready_;
    std::atomic<bool> write_failed_;
    std::atomic<double> last_sync_wall_time_s_;
    mutable std::mutex mutex_;
    std::ofstream output_;
    bool recording_;
    std::string state_;
    std::int16_t current_job_;
    ros::Time trigger_time_;
    bool trigger_time_valid_;
    bool command_target_valid_;
    double command_target_;
    std::size_t row_count_;
    double maximum_sync_error_s_;
};

void requireRecorderHealthy(const SynchronizedRecorder& recorder,
                            double data_stale_timeout_s) {
    if (recorder.writeFailed()) {
        throw std::runtime_error("recorded data output stream failed");
    }
    if (recorder.dataStale(data_stale_timeout_s)) {
        throw std::runtime_error("synchronized input data became stale");
    }
}

bool waitForDuration(double duration_s,
                     const SynchronizedRecorder& recorder,
                     double data_stale_timeout_s) {
    if (duration_s <= 0.0) {
        requireRecorderHealthy(recorder, data_stale_timeout_s);
        return ros::ok();
    }
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(duration_s);
    ros::WallRate rate(100.0);
    while (ros::ok() && ros::WallTime::now() < deadline) {
        requireRecorderHealthy(recorder, data_stale_timeout_s);
        rate.sleep();
    }
    return ros::ok();
}

bool waitForRecorderReady(const SynchronizedRecorder& recorder, double timeout_s) {
    const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout_s);
    ros::WallRate rate(100.0);
    while (ros::ok() && !recorder.ready() && ros::WallTime::now() < deadline) {
        rate.sleep();
    }
    return recorder.ready();
}

void loadParameters(ros::NodeHandle& node,
                    TrajectoryParameters& trajectory,
                    int& target_joint,
                    bool& stream_to_plc,
                    double& write_rate_hz,
                    bool& ros_control_enabled,
                    AdsHandConfiguration& ads) {
    node.param<std::string>("trajectory/waveform", trajectory.waveform, trajectory.waveform);
    node.param("trajectory/frequency_hz", trajectory.frequency_hz, trajectory.frequency_hz);
    node.param("trajectory/amplitude", trajectory.amplitude, trajectory.amplitude);
    node.param("trajectory/offset", trajectory.offset, trajectory.offset);
    node.param("trajectory/phase_rad", trajectory.phase_rad, trajectory.phase_rad);
    node.param("trajectory/duration_s", trajectory.duration_s, trajectory.duration_s);
    node.param("trajectory/sample_rate_hz", trajectory.sample_rate_hz,
               trajectory.sample_rate_hz);
    node.param("trajectory/target_joint", target_joint, 0);

    int max_points = static_cast<int>(trajectory.max_points);
    node.param("trajectory/max_points", max_points, max_points);
    if (max_points < 2) {
        throw std::invalid_argument("trajectory/max_points must be at least two");
    }
    trajectory.max_points = static_cast<std::size_t>(max_points);

    node.param("safety/limits_enabled", trajectory.limits_enabled, false);
    node.param("safety/minimum_value", trajectory.minimum_value, 0.0);
    node.param("safety/maximum_value", trajectory.maximum_value, 0.0);
    node.param("safety/maximum_step", trajectory.maximum_step, 0.0);

    node.param("execution/stream_to_plc", stream_to_plc, false);
    node.param("execution/write_rate_hz", write_rate_hz, 50.0);
    node.param("execution/ros_control", ros_control_enabled, true);
    int stream_job_value = ads.stream_job;
    node.param("execution/stream_job", stream_job_value, stream_job_value);
    if (stream_job_value < std::numeric_limits<std::int16_t>::min() ||
        stream_job_value > std::numeric_limits<std::int16_t>::max()) {
        throw std::invalid_argument("execution/stream_job is outside int16 range");
    }
    ads.stream_job = static_cast<std::int16_t>(stream_job_value);

    node.param<std::string>("ads/remote_ip", ads.remote_ip, ads.remote_ip);
    node.param<std::string>("ads/remote_ams_net_id", ads.remote_ams_net_id,
                            ads.remote_ams_net_id);
    node.param<std::string>("ads/local_ams_net_id", ads.local_ams_net_id,
                            ads.local_ams_net_id);
    int ads_port = ads.port;
    node.param("ads/port", ads_port, ads_port);
    if (ads_port <= 0 || ads_port > std::numeric_limits<std::uint16_t>::max()) {
        throw std::invalid_argument("ads/port is outside uint16 range");
    }
    ads.port = static_cast<std::uint16_t>(ads_port);

    node.param<std::string>("plc_variables/ros_control",
                            ads.ros_control_variable,
                            ads.ros_control_variable);
    node.param<std::string>("plc_variables/hand_targets",
                            ads.hand_targets_variable,
                            ads.hand_targets_variable);
    node.param<std::string>("plc_variables/current_job",
                            ads.current_job_variable,
                            ads.current_job_variable);
}

}  // namespace
}  // namespace twincat_talker

int main(int argc, char** argv) {
    ros::init(argc, argv, "single_experiment_manager");
    ros::NodeHandle private_node("~");

    using namespace twincat_talker;

    std::string experiment_directory;
    std::unique_ptr<SynchronizedRecorder> recorder;
    std::unique_ptr<ros::AsyncSpinner> spinner;
    ExperimentResult result;

    try {
        TrajectoryParameters trajectory_parameters;
        AdsHandConfiguration ads_configuration;
        int target_joint = 0;
        bool stream_to_plc = false;
        double write_rate_hz = 50.0;
        bool ros_control_enabled = true;
        loadParameters(private_node,
                       trajectory_parameters,
                       target_joint,
                       stream_to_plc,
                       write_rate_hz,
                       ros_control_enabled,
                       ads_configuration);

        if (target_joint < 0 || target_joint >= static_cast<int>(kHandChannelCount)) {
            throw std::invalid_argument("trajectory/target_joint must be between 0 and 15");
        }
        if (write_rate_hz <= 0.0) {
            throw std::invalid_argument("execution/write_rate_hz must be greater than zero");
        }
        if (stream_to_plc && !trajectory_parameters.limits_enabled) {
            throw std::invalid_argument("PLC streaming requires safety/limits_enabled=true");
        }

        std::string output_root;
        std::string experiment_name;
        private_node.param<std::string>("output/root_directory",
                                        output_root,
                                        "~/.ros/hand_datalogger");
        private_node.param<std::string>("output/experiment_name",
                                        experiment_name,
                                        "single_experiment");
        experiment_directory =
            createUniqueExperimentDirectory(output_root, experiment_name);
        RuntimeLog runtime_log(experiment_directory + "/runtime.log");
        runtime_log.write("GENERATING trajectory");

        const std::vector<TrajectoryPoint> trajectory =
            TrajectoryGenerator::generate(trajectory_parameters);
        result.generated_points = trajectory.size();
        TrajectoryGenerator::writeCsv(experiment_directory + "/generated_trajectory.csv",
                                      trajectory);
        writeExperimentYaml(experiment_directory + "/experiment.yaml",
                            trajectory_parameters,
                            target_joint,
                            stream_to_plc,
                            write_rate_hz,
                            ros_control_enabled,
                            ads_configuration);
        runtime_log.write("VALIDATED " + std::to_string(trajectory.size()) + " points");

        if (!stream_to_plc) {
            result.status = "validated";
            writeResultYaml(experiment_directory + "/result.yaml", result);
            runtime_log.write("COMPLETED dry-run without ADS connection");
            return 0;
        }

        runtime_log.write("CONNECTING to ADS");
        HandTrajectoryStreamer streamer(ads_configuration);
        streamer.connect();
        if (ros_control_enabled) {
            streamer.enableRosControl();
            runtime_log.write("ROSControl enabled on the PLC");
        }
        const std::array<float, kHandChannelCount> hold_targets =
            streamer.readCurrentTargets();
        for (std::size_t channel = 0; channel < kHandChannelCount; ++channel) {
            if (!std::isfinite(hold_targets[channel])) {
                throw std::runtime_error(
                    "PLC reported a non-finite value in the hand target array");
            }
        }
        runtime_log.write(
            "CONNECTED; non-excited hand channels hold the current MAIN.ROS_A values");

        std::string pose_topic;
        std::string joint_topic;
        private_node.param<std::string>("topics/pose", pose_topic, "/optitrack/pose1");
        private_node.param<std::string>("topics/joint_state",
                                        joint_topic,
                                        "/twincat/joint_states");
        int subscriber_queue_size = 20;
        int sync_queue_size = 50;
        int flush_interval = 100;
        double sync_tolerance_s = 0.02;
        double data_ready_timeout_s = 10.0;
        double data_stale_timeout_s = 1.0;
        double pre_roll_s = 2.0;
        double post_roll_s = 2.0;
        double maximum_runtime_s = 600.0;
        private_node.param("logging/subscriber_queue_size",
                           subscriber_queue_size,
                           subscriber_queue_size);
        private_node.param("logging/sync_queue_size", sync_queue_size, sync_queue_size);
        private_node.param("logging/sync_tolerance_s", sync_tolerance_s, sync_tolerance_s);
        private_node.param("logging/data_ready_timeout_s",
                           data_ready_timeout_s,
                           data_ready_timeout_s);
        private_node.param("logging/data_stale_timeout_s",
                           data_stale_timeout_s,
                           data_stale_timeout_s);
        private_node.param("logging/flush_interval", flush_interval, flush_interval);
        private_node.param("execution/pre_roll_s", pre_roll_s, pre_roll_s);
        private_node.param("execution/post_roll_s", post_roll_s, post_roll_s);
        private_node.param("execution/maximum_runtime_s",
                           maximum_runtime_s,
                           maximum_runtime_s);

        if (subscriber_queue_size <= 0 || sync_queue_size <= 0 ||
            data_ready_timeout_s <= 0.0 || data_stale_timeout_s <= 0.0 ||
            maximum_runtime_s <= 0.0 || pre_roll_s < 0.0 || post_roll_s < 0.0) {
            throw std::invalid_argument("logging or execution timing parameters are invalid");
        }

        std::vector<int> joint_indices;
        if (!private_node.getParam("logging/joint_indices", joint_indices)) {
            joint_indices.push_back(0);
            joint_indices.push_back(1);
        }

        ros::NodeHandle node;
        recorder.reset(new SynchronizedRecorder(node,
                                                 pose_topic,
                                                 joint_topic,
                                                 subscriber_queue_size,
                                                 sync_queue_size,
                                                 sync_tolerance_s,
                                                 joint_indices,
                                                 flush_interval));
        spinner.reset(new ros::AsyncSpinner(1));
        spinner->start();

        runtime_log.write("WAITING_FOR_DATA on " + pose_topic + " and " + joint_topic);
        if (!waitForRecorderReady(*recorder, data_ready_timeout_s)) {
            throw std::runtime_error("timed out waiting for synchronized input data");
        }

        recorder->start(experiment_directory + "/recorded_data.csv");
        recorder->setState("PRE_ROLL");
        recorder->setCurrentJob(streamer.readCurrentJob());
        runtime_log.write("PRE_ROLL recording started");
        if (!waitForDuration(pre_roll_s, *recorder, data_stale_timeout_s)) {
            throw std::runtime_error("ROS shutdown during pre-roll");
        }

        const std::size_t point_count = trajectory.size();
        const std::size_t stream_channel = static_cast<std::size_t>(target_joint);
        recorder->setState("STREAMING");
        const ros::Time stream_start = ros::Time::now();
        recorder->setTriggerTime(stream_start);
        streamer.writeSample(stream_channel, static_cast<float>(trajectory[0].target));
        recorder->setCommandTarget(trajectory[0].target);
        std::size_t last_written_index = 0;
        result.written_points = 1;
        runtime_log.write("STREAMING " + std::to_string(point_count) + " points at " +
                          std::to_string(write_rate_hz) + " Hz");

        const ros::WallTime runtime_deadline =
            ros::WallTime::now() + ros::WallDuration(maximum_runtime_s);
        ros::WallRate write_rate(write_rate_hz);
        const double sample_period_s = 1.0 / trajectory_parameters.sample_rate_hz;

        while (ros::ok()) {
            requireRecorderHealthy(*recorder, data_stale_timeout_s);

            // Sample-and-hold playback: derive the trajectory index from the
            // elapsed time so write-rate jitter never accumulates phase error.
            double elapsed_s = (ros::Time::now() - stream_start).toSec();
            if (elapsed_s < 0.0) {
                // Guard against wall-clock steps backwards (e.g. NTP sync).
                elapsed_s = 0.0;
            }
            std::size_t target_index =
                static_cast<std::size_t>(elapsed_s / sample_period_s + 0.5);
            if (target_index >= point_count) {
                target_index = point_count - 1;
            }
            if (target_index != last_written_index) {
                streamer.writeSample(stream_channel,
                                     static_cast<float>(trajectory[target_index].target));
                recorder->setCommandTarget(trajectory[target_index].target);
                last_written_index = target_index;
                result.written_points = last_written_index + 1;
            }

            const std::int16_t current_job = streamer.readCurrentJob();
            recorder->setCurrentJob(current_job);
            result.final_current_job = current_job;

            if (last_written_index == point_count - 1) {
                break;
            }
            if (ros::WallTime::now() >= runtime_deadline) {
                throw std::runtime_error(
                    "maximum runtime exceeded before the stream finished");
            }
            write_rate.sleep();
        }

        recorder->setState("POST_ROLL");
        runtime_log.write("POST_ROLL started after the last trajectory point was written");
        if (!waitForDuration(post_roll_s, *recorder, data_stale_timeout_s)) {
            throw std::runtime_error("ROS shutdown during post-roll");
        }

        recorder->stop();
        result.recorded_rows = recorder->rowCount();
        result.maximum_sync_error_s = recorder->maximumSyncError();
        result.final_current_job = streamer.readCurrentJob();
        result.status = "completed";
        writeResultYaml(experiment_directory + "/result.yaml", result);
        runtime_log.write("COMPLETED one-shot experiment");
        spinner->stop();
        return 0;
    } catch (const std::exception& exception) {
        ROS_ERROR_STREAM("Single experiment failed: " << exception.what());
        result.failure_reason = exception.what();
        if (recorder.get() != NULL) {
            recorder->stop();
            result.recorded_rows = recorder->rowCount();
            result.maximum_sync_error_s = recorder->maximumSyncError();
        }
        if (spinner.get() != NULL) {
            spinner->stop();
        }
        if (!experiment_directory.empty()) {
            writeResultYaml(experiment_directory + "/result.yaml", result);
        }
        return 1;
    }
}
