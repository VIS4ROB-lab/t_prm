#pragma once

#include <benchmarking/benchmark.h>

#include <chrono>
#include <string>
#include <vector>
#include <memory>

namespace benchmarking {

// Individual time measurement.
struct TimingResult {
    std::string description;
    double duration_micros;
};

struct BenchmarkResult {
    std::string description = "unset";
    double duration_micros = -1.;
    std::vector<bool> success = {};
    std::vector<std::vector<Eigen::Vector3d>> path = {};
    std::vector<std::vector<double>> path_durations = {}; // for T-PRM
    std::vector<TimingResult> timing_results = {};
};

class BasePlannerBenchmark {
public:
    BasePlannerBenchmark();

    virtual std::string getName() const = 0;

    virtual BenchmarkResult runBenchmark(std::shared_ptr<Benchmark> benchmark, int benchmark_idx, int run_idx);

    void startBenchmark();
    void stopBenchmark();

    void startMeasurement();
    TimingResult stopMeasurement(std::string what);

    // in micro seconds
    double getTotalDuration() const;

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_start_time;

    std::chrono::time_point<std::chrono::high_resolution_clock> m_total_start_time;
    double m_total_duration;
};

}  // namespace benchmarking