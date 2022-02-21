#include <benchmarking/OMPLPlanner.h>
#include <benchmarking/TPRMPlanner.h>
#include <benchmarking/factory.h>

#include <iostream>

std::shared_ptr<benchmarking::Benchmark> create_basic_bm(int iters) {
    using namespace benchmarking;

    // Register benchmarks
    auto b1 = std::make_shared<Benchmark>("Simple Comparison", iters);
    b1->numNodes = 1000;

    for (size_t i = 0; i < 10; i++) {
        b1->start.push_back(tprm::Vector3d::Zero());
        b1->goal.push_back(tprm::Vector3d::Constant(10.));
    }

    b1->domain_size = 10.;

    // Specific
    b1->rrt_growIterations = 5000;
    b1->rrt_edge_length = 1.5;

    b1->tprm_max_time = 40.0;
    b1->tprm_cost_edge_threshold = 10;

    b1->ompl_path_length_threshold = std::numeric_limits<double>::infinity();
    b1->ompl_time_limit = 1.;

    return b1;
}

int main(int argc, char const* argv[]) {
    const int NUM_ITERS = 1;

    srand(time(NULL));
    using namespace benchmarking;

    BenchmarkFactory factory = BenchmarkFactory();

    // Register planners
    factory.register_planner(std::make_shared<TPRMPlannerBenchmark>());
    factory.register_planner(std::make_shared<OMPLPlannerBenchmark>("RRTstar"));
    factory.register_planner(std::make_shared<OMPLPlannerBenchmark>("PRM"));

    // Register result handlers
    factory.register_result_handler(std::make_shared<ResultHandlerPathLength>());
    factory.register_result_handler(std::make_shared<ResultHandlerSuccessRate>());
    factory.register_result_handler(std::make_shared<ResultHandlerComputingTime>());
    factory.register_result_handler(std::make_shared<ResultHandlerThetaChanges>());
    factory.register_result_handler(std::make_shared<ResultHandlerPathWriter>());
    factory.register_result_handler(std::make_shared<ResultHandlerAllWriter>());

    // Register benchmarks
    factory.register_benchmark(create_basic_bm(NUM_ITERS));

    // Run all benchmarks on all planners
    factory.run();

    return 0;
}
