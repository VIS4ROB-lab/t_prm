#include <benchmarking/TPRMPlanner.h>
#include <tprm/config.h>
#include <tprm/obstacle_impl.h>

#include <fstream>
#include <iostream>

namespace benchmarking {

// path (pos x, pos y), time (t)
std::pair<std::vector<tprm::Vector3d>, std::vector<double>> TPRMPlannerBenchmark::computePath(const std::vector<tprm::PathResultEntry>& path) const {
    std::vector<tprm::Vector3d> path_;
    std::vector<double> times;
    for (size_t i = 0; i < path.size(); ++i) {
        path_.push_back(path[i].position);
        times.push_back(path[i].time);
    }

    return {path_, times};
}

BenchmarkResult TPRMPlannerBenchmark::runBenchmark(std::shared_ptr<Benchmark> benchmark, int benchmark_idx, int run_idx) {
    BasePlannerBenchmark::startBenchmark();

    using namespace tprm;

    BenchmarkResult result;

    TemporalPRM tprm;

    Vector3d roomMax;
    if (benchmark->is_2d) {
        roomMax = Vector3d(benchmark->domain_size, benchmark->domain_size, 0.);
    } else {
        roomMax = Vector3d::Constant(benchmark->domain_size);
    }

    tprm.setEnvironmentMin(Vector3d::Constant(0.));
    tprm.setEnvironmentMax(roomMax);

    for (const auto& c : benchmark->circles) {
        tprm.addStaticObstacle(std::make_shared<tprm::StaticSphereObstacle>(c.center, c.radius));
    }
    for (const auto& c : benchmark->moving_circles) {
        tprm.addDynamicObstacle(std::make_shared<tprm::DynamicSphereObstacle>(c.center, c.velocity, c.radius));
    }

    startMeasurement();
    tprm.placeSamples(benchmark->numNodes);
    result.timing_results.push_back(stopMeasurement("Place Samples"));

    startMeasurement();
    tprm.buildPRM(benchmark->tprm_cost_edge_threshold);
    result.timing_results.push_back(stopMeasurement("Build PRM"));

    for (size_t query_idx = 0; query_idx < benchmark->start.size(); query_idx++) {
        startMeasurement();
        auto path = tprm.getShortestPath(benchmark->start[query_idx], benchmark->goal[query_idx]);
        result.timing_results.push_back(stopMeasurement("A* " + std::to_string(query_idx)));

        result.success.push_back(path.size() > 0);

        auto [path_pos, path_time] = computePath(path);
        result.path.push_back(path_pos);
        result.path_durations.push_back(path_time);
    }

    BasePlannerBenchmark::stopBenchmark();
    result.description = "T-PRM - " + benchmark->name;
    result.duration_micros = getTotalDuration();

    return result;
}

} /* namespace benchmarking */