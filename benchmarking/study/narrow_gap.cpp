/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022,
 *  ETH Zurich - V4RL, Department of Mechanical and Process Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Matthias Busenhart
 *********************************************************************/

#include <benchmarking/OMPLPlanner.h>
#include <benchmarking/TPRMPlanner.h>
#include <benchmarking/factory.h>

#include <iostream>

void write_info_about_bm(std::shared_ptr<benchmarking::Benchmark> bm) {
    std::ofstream file("info_about_bm.csv", std::ios::app);
    file << "==========" << std::endl;
    file << "Benchmark: " << bm->name << std::endl;
    file << "Num. of obstacles: " << bm->circles.size() << std::endl;
    file << "Num. of queries: " << bm->start.size() << std::endl;
    file << "Num. of runs: " << bm->numRuns() << std::endl;
    file << "OBSTACLES:" << std::endl;
    for (size_t i = 0; i < bm->circles.size(); ++i) {
        file << "  " << bm->circles[i].center.x() << ", " << bm->circles[i].center.y() << ", " << bm->circles[i].radius << std::endl;
    }
    file << "QUERIES:" << std::endl;
    for (size_t i = 0; i < bm->start.size(); ++i) {
        file << "  " << bm->start[i].x() << ", " << bm->start[i].y() << ", " << bm->start[i].z() << ", " << bm->goal[i].x() << ", " << bm->goal[i].y() << ", "
             << bm->goal[i].z() << std::endl;
    }
}

std::shared_ptr<benchmarking::Benchmark> create_basic_bm(int iters) {
    using namespace benchmarking;

    // Register benchmarks
    auto b1 = std::make_shared<Benchmark>("Narrow Gap", iters);
    b1->numNodes = 1000; // CHANGE HERE (number of nodes of T-PRM)

    b1->domain_size = 10.;

    // Specific

    b1->tprm_cost_edge_threshold = 1.0; // CHANGE HERE (edge connection radius of T-PRM)

    b1->ompl_path_length_threshold = std::numeric_limits<double>::infinity();
    b1->ompl_edge_length = 1.0; // CHANGE HERE (edge connection radius of PRM)
    b1->ompl_time_limit = 1.0;

    b1->start.push_back(tprm::Vector3d::Zero());
    b1->goal.push_back(tprm::Vector3d::Constant(10.));

    b1->circles.push_back(benchmarking::Circle(tprm::Vector3d(5., 1.5, 0.), 2.));
    b1->circles.push_back(benchmarking::Circle(tprm::Vector3d(5., 3., 0.), 1.95));

    // and other side
    b1->circles.push_back(benchmarking::Circle(tprm::Vector3d(5., 10. - 1.5, 0.), 2.));
    b1->circles.push_back(benchmarking::Circle(tprm::Vector3d(5., 10. - 3., 0.), 1.95));

    write_info_about_bm(b1);

    return b1;
}

int main(int argc, char const* argv[]) {
    const int NUM_ITERS = 1;  // 100 for real runs, 1 for path print.

    std::ofstream file("info_about_bm.csv");
    file.close();  // clear the file

    srand(42);
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

    // Remove for benchmark runs because slow
    factory.register_result_handler(std::make_shared<ResultHandlerPathWriter>());

    //factory.register_result_handler(std::make_shared<ResultHandlerAllWriter>());

    // Register benchmarks
    factory.register_benchmark(create_basic_bm(NUM_ITERS));

    // Run all benchmarks on all planners
    factory.run();

    return 0;
}
