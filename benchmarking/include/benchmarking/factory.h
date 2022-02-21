#pragma once

#include <benchmarking/base.h>
#include <benchmarking/benchmark.h>
#include <benchmarking/result_handler.h>

#include <vector>
#include <memory>

namespace benchmarking {

class BenchmarkFactory {
public:
    BenchmarkFactory();
    ~BenchmarkFactory();

    void register_benchmark(std::shared_ptr<Benchmark> benchmark);
    void register_benchmarks(std::vector<std::shared_ptr<Benchmark>> benchmarks);
    void register_planner(std::shared_ptr<BasePlannerBenchmark> planner);
    void register_result_handler(std::shared_ptr<ResultHandler> result_handler);

    void run();

private:
    void run_benchmark(std::shared_ptr<BasePlannerBenchmark> planner, std::shared_ptr<Benchmark> benchmark);

    void writeStaticObstacles(size_t bm_idx) const;

    void log(std::string message) const;
    void log(std::string tag, std::string message) const;

private:
    std::vector<std::shared_ptr<BasePlannerBenchmark>> m_planners;
    std::vector<std::shared_ptr<Benchmark>> m_benchmarks;
    std::vector<std::shared_ptr<ResultHandler>> m_result_handlers;

    int current_benchmark_index = 0;
};

}  // namespace benchmarking
