#pragma once

#include <benchmarking/base.h>

namespace benchmarking {

class OMPLPlannerBenchmark : public BasePlannerBenchmark {
public:
    OMPLPlannerBenchmark(std::string planner = "RRTstar");
    std::string getName() const override {
        return "OMPL " + m_planner;
    }

    BenchmarkResult runBenchmark(std::shared_ptr<Benchmark> benchmark, int benchmark_idx, int run_idx) override;

private:
    std::string m_planner;
};
}  // namespace benchmarking