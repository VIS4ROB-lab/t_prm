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