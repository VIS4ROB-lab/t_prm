#include <benchmarking/base.h>
#include <tprm/temporal_prm.h>

namespace benchmarking {

// Wrapper around T-PRM
class TPRMPlannerBenchmark : public BasePlannerBenchmark {
public:
    TPRMPlannerBenchmark() {}
    std::string getName() const override {
        return "T-PRM";
    }

    BenchmarkResult runBenchmark(std::shared_ptr<Benchmark> benchmark, int benchmark_idx, int run_idx) override;

private:
    // path (pos x, pos y, pos z), time (t)
    std::pair<std::vector<tprm::Vector3d>, std::vector<double>> computePath(const std::vector<tprm::PathResultEntry>& path) const;
};

}  // namespace benchmarking