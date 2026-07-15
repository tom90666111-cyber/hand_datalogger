#include "twincat_talker/trajectory_generator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace {

twincat_talker::TrajectoryParameters safeParameters() {
    twincat_talker::TrajectoryParameters parameters;
    parameters.waveform = "sine";
    parameters.frequency_hz = 1.0;
    parameters.amplitude = 2.0;
    parameters.offset = 5.0;
    parameters.duration_s = 1.0;
    parameters.sample_rate_hz = 100.0;
    parameters.limits_enabled = true;
    parameters.minimum_value = 2.0;
    parameters.maximum_value = 8.0;
    parameters.maximum_step = 1.0;
    return parameters;
}

}  // namespace

TEST(TrajectoryGenerator, GeneratesExpectedSinePointCountAndValues) {
    const std::vector<twincat_talker::TrajectoryPoint> trajectory =
        twincat_talker::TrajectoryGenerator::generate(safeParameters());

    ASSERT_EQ(101u, trajectory.size());
    EXPECT_DOUBLE_EQ(0.0, trajectory.front().time_s);
    EXPECT_NEAR(5.0, trajectory.front().target, 1e-12);
    EXPECT_NEAR(7.0, trajectory[25].target, 1e-12);
    EXPECT_NEAR(5.0, trajectory.back().target, 1e-12);
}

TEST(TrajectoryGenerator, SupportsTriangleAndSquareWaveforms) {
    twincat_talker::TrajectoryParameters parameters = safeParameters();
    parameters.waveform = "triangle";
    EXPECT_NO_THROW(twincat_talker::TrajectoryGenerator::generate(parameters));

    parameters.waveform = "square";
    EXPECT_NO_THROW(twincat_talker::TrajectoryGenerator::generate(parameters));
}

TEST(TrajectoryGenerator, RejectsCapacityOverflow) {
    twincat_talker::TrajectoryParameters parameters = safeParameters();
    parameters.duration_s = 601.0;
    parameters.sample_rate_hz = 100.0;
    parameters.max_points = 60001;
    EXPECT_THROW(twincat_talker::TrajectoryGenerator::generate(parameters),
                 std::invalid_argument);
}

TEST(TrajectoryGenerator, RejectsUnsafeLimitsAndSteps) {
    twincat_talker::TrajectoryParameters parameters = safeParameters();
    parameters.maximum_value = 6.0;
    EXPECT_THROW(twincat_talker::TrajectoryGenerator::generate(parameters),
                 std::runtime_error);

    parameters = safeParameters();
    parameters.maximum_step = 0.001;
    EXPECT_THROW(twincat_talker::TrajectoryGenerator::generate(parameters),
                 std::runtime_error);
}
