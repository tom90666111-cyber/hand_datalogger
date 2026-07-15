#include "twincat_talker/trajectory_generator.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>

namespace twincat_talker {
namespace {

const double kPi = 3.14159265358979323846;

bool isFinite(double value) {
    return std::isfinite(value);
}

}  // namespace

TrajectoryParameters::TrajectoryParameters()
    : waveform("sine"),
      frequency_hz(0.5),
      amplitude(1.0),
      offset(0.0),
      phase_rad(0.0),
      duration_s(10.0),
      sample_rate_hz(200.0),
      max_points(60001),
      limits_enabled(false),
      minimum_value(0.0),
      maximum_value(0.0),
      maximum_step(0.0) {}

void TrajectoryGenerator::validateParameters(const TrajectoryParameters& parameters) {
    if (parameters.waveform != "sine" && parameters.waveform != "triangle" &&
        parameters.waveform != "square") {
        throw std::invalid_argument("waveform must be sine, triangle, or square");
    }

    if (!isFinite(parameters.frequency_hz) || parameters.frequency_hz <= 0.0) {
        throw std::invalid_argument("frequency_hz must be finite and greater than zero");
    }
    if (!isFinite(parameters.amplitude) || parameters.amplitude < 0.0) {
        throw std::invalid_argument("amplitude must be finite and non-negative");
    }
    if (!isFinite(parameters.offset) || !isFinite(parameters.phase_rad)) {
        throw std::invalid_argument("offset and phase_rad must be finite");
    }
    if (!isFinite(parameters.duration_s) || parameters.duration_s <= 0.0) {
        throw std::invalid_argument("duration_s must be finite and greater than zero");
    }
    if (!isFinite(parameters.sample_rate_hz) || parameters.sample_rate_hz <= 0.0) {
        throw std::invalid_argument("sample_rate_hz must be finite and greater than zero");
    }
    if (parameters.max_points < 2) {
        throw std::invalid_argument("max_points must be at least two");
    }

    const double sample_intervals = parameters.duration_s * parameters.sample_rate_hz;
    if (!isFinite(sample_intervals) || sample_intervals >
                                           static_cast<double>(parameters.max_points - 1)) {
        throw std::invalid_argument("trajectory exceeds the configured point capacity");
    }

    if (parameters.limits_enabled) {
        if (!isFinite(parameters.minimum_value) || !isFinite(parameters.maximum_value) ||
            parameters.minimum_value > parameters.maximum_value) {
            throw std::invalid_argument("trajectory limits are invalid");
        }
        if (!isFinite(parameters.maximum_step) || parameters.maximum_step <= 0.0) {
            throw std::invalid_argument("maximum_step must be finite and greater than zero");
        }
    }
}

double TrajectoryGenerator::sampleWaveform(const TrajectoryParameters& parameters,
                                           double time_s) {
    const double phase = 2.0 * kPi * parameters.frequency_hz * time_s +
                         parameters.phase_rad;
    double normalized = 0.0;

    if (parameters.waveform == "sine") {
        normalized = std::sin(phase);
    } else if (parameters.waveform == "triangle") {
        normalized = (2.0 / kPi) * std::asin(std::sin(phase));
    } else {
        normalized = std::sin(phase) >= 0.0 ? 1.0 : -1.0;
    }

    return parameters.offset + parameters.amplitude * normalized;
}

std::vector<TrajectoryPoint> TrajectoryGenerator::generate(
    const TrajectoryParameters& parameters) {
    validateParameters(parameters);

    const std::size_t interval_count = static_cast<std::size_t>(
        std::llround(parameters.duration_s * parameters.sample_rate_hz));
    const std::size_t point_count = interval_count + 1;
    if (point_count > parameters.max_points) {
        throw std::invalid_argument("rounded trajectory point count exceeds capacity");
    }

    std::vector<TrajectoryPoint> trajectory;
    trajectory.reserve(point_count);

    for (std::size_t index = 0; index < point_count; ++index) {
        const double time_s = static_cast<double>(index) / parameters.sample_rate_hz;
        const double target = sampleWaveform(parameters, time_s);

        if (!isFinite(target)) {
            throw std::runtime_error("generated trajectory contains a non-finite value");
        }
        if (parameters.limits_enabled &&
            (target < parameters.minimum_value || target > parameters.maximum_value)) {
            throw std::runtime_error("generated trajectory exceeds the configured limits");
        }
        if (parameters.limits_enabled && !trajectory.empty() &&
            std::fabs(target - trajectory.back().target) > parameters.maximum_step) {
            throw std::runtime_error("generated trajectory exceeds maximum_step");
        }

        TrajectoryPoint point;
        point.index = index;
        point.time_s = time_s;
        point.target = target;
        trajectory.push_back(point);
    }

    return trajectory;
}

void TrajectoryGenerator::writeCsv(const std::string& path,
                                   const std::vector<TrajectoryPoint>& trajectory) {
    if (trajectory.empty()) {
        throw std::invalid_argument("cannot write an empty trajectory");
    }

    std::ofstream output(path.c_str(), std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
        throw std::runtime_error("cannot open trajectory output file: " + path);
    }

    output << "index,time,target\n";
    output << std::fixed << std::setprecision(9);
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
        output << trajectory[i].index << ',' << trajectory[i].time_s << ','
               << trajectory[i].target << '\n';
    }

    if (!output.good()) {
        throw std::runtime_error("failed while writing trajectory output file: " + path);
    }
}

}  // namespace twincat_talker
