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
