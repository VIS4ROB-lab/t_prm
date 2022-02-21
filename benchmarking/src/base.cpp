#include <benchmarking/base.h>

namespace benchmarking {

BasePlannerBenchmark::BasePlannerBenchmark() {}

void BasePlannerBenchmark::startMeasurement() {
    m_start_time = std::chrono::high_resolution_clock::now();
}

BenchmarkResult BasePlannerBenchmark::runBenchmark(Benchmark*, int, int) {
    return BenchmarkResult();
};

void BasePlannerBenchmark::startBenchmark() {
    m_total_start_time = std::chrono::high_resolution_clock::now();
}

void BasePlannerBenchmark::stopBenchmark() {
    m_total_duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - m_total_start_time).count();
}

TimingResult BasePlannerBenchmark::stopMeasurement(std::string what) {
    auto end_time = std::chrono::high_resolution_clock::now();
    double duration_micros = std::chrono::duration_cast<std::chrono::microseconds>(end_time - m_start_time).count();
    TimingResult tr;
    tr.description = what;
    tr.duration_micros = duration_micros;
    return tr;
}

double BasePlannerBenchmark::getTotalDuration() const {
    return m_total_duration;
}

}  // namespace benchmarking