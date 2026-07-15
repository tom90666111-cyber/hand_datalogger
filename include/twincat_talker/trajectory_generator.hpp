#ifndef TWINCAT_TALKER_TRAJECTORY_GENERATOR_HPP
#define TWINCAT_TALKER_TRAJECTORY_GENERATOR_HPP

#include <cstddef>
#include <string>
#include <vector>

namespace twincat_talker {

struct TrajectoryParameters {
    std::string waveform;
    double frequency_hz;
    double amplitude;
    double offset;
    double phase_rad;
    double duration_s;
    double sample_rate_hz;
    std::size_t max_points;
    bool limits_enabled;
    double minimum_value;
    double maximum_value;
    double maximum_step;

    TrajectoryParameters();
};

struct TrajectoryPoint {
    std::size_t index;
    double time_s;
    double target;
};

class TrajectoryGenerator {
public:
    static std::vector<TrajectoryPoint> generate(const TrajectoryParameters& parameters);
    static void writeCsv(const std::string& path,
                         const std::vector<TrajectoryPoint>& trajectory);

private:
    static void validateParameters(const TrajectoryParameters& parameters);
    static double sampleWaveform(const TrajectoryParameters& parameters, double time_s);
};

}  // namespace twincat_talker

#endif  // TWINCAT_TALKER_TRAJECTORY_GENERATOR_HPP
