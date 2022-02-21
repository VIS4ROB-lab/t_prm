#include <benchmarking/factory.h>

#include <iostream>

namespace benchmarking {

BenchmarkFactory::BenchmarkFactory() {}

BenchmarkFactory::~BenchmarkFactory() {}

void BenchmarkFactory::run() {
    // Run all registered benchmarks for all planners
    for (size_t i = 0; i < m_benchmarks.size(); ++i) {
        current_benchmark_index = i;
        log("********************************************************");
        log("Running benchmark: " + m_benchmarks[i]->name);

        for (size_t j = 0; j < m_planners.size(); ++j) {
            log("Running planner: " + m_planners[j]->getName());
            run_benchmark(m_planners[j], m_benchmarks[i]);
        }
    }
    for (std::shared_ptr<ResultHandler> handler : m_result_handlers) {
        handler->printSummary();
    }
}

void BenchmarkFactory::run_benchmark(std::shared_ptr<BasePlannerBenchmark> planner, std::shared_ptr<Benchmark> benchmark) {
    // Run the benchmark for the planner n times
    int n = benchmark->numRuns();
    std::vector<double> durations(n);
    std::vector<bool> success(n);

    // Tell the result handler which benchmark we use.
    for (std::shared_ptr<ResultHandler> handler : m_result_handlers) {
        handler->nextBenchmarkIs(benchmark);
    }

    for (int i = 0; i < n; ++i) {
        BenchmarkResult result = planner->runBenchmark(benchmark, current_benchmark_index, i);
        for (std::shared_ptr<ResultHandler> handler : m_result_handlers) {
            handler->handleResult(result, planner->getName(), current_benchmark_index, i);
        }
    }

    for (std::shared_ptr<ResultHandler> handler : m_result_handlers) {
        handler->printSummary(planner->getName(), current_benchmark_index);
    }
}

void BenchmarkFactory::register_planner(std::shared_ptr<BasePlannerBenchmark> planner) {
    m_planners.push_back(planner);
    log("Registered planner '" + planner->getName() + "'");
}

void BenchmarkFactory::register_benchmark(std::shared_ptr<Benchmark> benchmark) {
    m_benchmarks.push_back(benchmark);
    log("Registered benchmark '" + benchmark->name + "'");

    if (benchmark->is_2d) {
        for (auto& q : benchmark->start) {
            q.z() = 0.;
        }
        for (auto& q : benchmark->goal) {
            q.z() = 0.;
        }
    }
}

void BenchmarkFactory::register_benchmarks(std::vector<std::shared_ptr<Benchmark>> benchmarks) {
    for (std::shared_ptr<Benchmark> benchmark : benchmarks) {
        register_benchmark(benchmark);
    }
}

void BenchmarkFactory::register_result_handler(std::shared_ptr<ResultHandler> handler) {
    m_result_handlers.push_back(handler);
    log("Registered result handler '" + handler->getName() + "'");
}

void BenchmarkFactory::log(std::string message) const {
    log("Factory", message);
}

void BenchmarkFactory::log(std::string tag, std::string message) const {
    std::cout << "[" << tag << "] " << message << std::endl;
}

}  // namespace benchmarking