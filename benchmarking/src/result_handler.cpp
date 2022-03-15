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

#include <benchmarking/result_handler.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>

namespace benchmarking {

void ResultHandlerPathLength::nextBenchmarkIs(std::shared_ptr<Benchmark> benchmark) {
    m_current_benchmark = benchmark;
}

double std_dev(double mean, const std::vector<double>& data) {
    double sum = 0;
    for (double d : data) {
        sum += (d - mean) * (d - mean);
    }
    return std::sqrt(sum / data.size());
}

// Path Length
void ResultHandlerPathLength::handleResult(const BenchmarkResult& result, std::string, int, int) {
    last_description = result.description;

    for (size_t query_idx = 0; query_idx < result.success.size(); ++query_idx) {
        if (!result.success[query_idx] || result.path[query_idx].size() == 0)
            return;

        double path_length = 0.0;
        path_length += (m_current_benchmark->start[query_idx] - result.path[query_idx][0]).norm();
        for (size_t i = 1; i < result.path[query_idx].size(); ++i) {
            path_length += (result.path[query_idx][i - 1] - result.path[query_idx][i]).norm();
        }
        path_length += (m_current_benchmark->goal[query_idx] - result.path[query_idx].back()).norm();

        m_path_lengths_for_idx.push_back(path_length);
    }
}

void ResultHandlerPathLength::printSummary(std::string planner, int benchmark_idx) {
    // Print the average, min and max path length
    if (m_path_lengths_for_idx.size() == 0) {
        std::cout << "Path length: not any valid measurements." << std::endl;
        return;
    }
    std::cout << "Path length: " << std::endl;
    double avg = std::accumulate(m_path_lengths_for_idx.begin(), m_path_lengths_for_idx.end(), 0.0) / m_path_lengths_for_idx.size();
    std::cout << "  Average: " << avg << std::endl;
    std::cout << "  Std. Dev: " << std_dev(avg, m_path_lengths_for_idx) << std::endl;
    std::cout << "  Min: " << *std::min_element(m_path_lengths_for_idx.begin(), m_path_lengths_for_idx.end()) << std::endl;
    std::cout << "  Max: " << *std::max_element(m_path_lengths_for_idx.begin(), m_path_lengths_for_idx.end()) << std::endl;
    std::cout << "  " << m_path_lengths_for_idx.size() << " measurements." << std::endl;

    m_file << last_description << ", " << benchmark_idx << ", ";
    for (size_t i = 0; i < m_path_lengths_for_idx.size(); ++i) {
        m_file << m_path_lengths_for_idx[i];
        if (i < m_path_lengths_for_idx.size() - 1) {
            m_file << ", ";
        }
    }
    m_file << std::endl;

    m_path_lengths_for_idx.clear();
}

// Success Rate
void ResultHandlerSuccessRate::handleResult(const BenchmarkResult& result, std::string, int, int) {
    for (size_t query_idx = 0; query_idx < result.success.size(); ++query_idx) {
        m_results_for_idx.push_back(result.success[query_idx]);
    }
    last_description = result.description;
}

void ResultHandlerSuccessRate::printSummary(std::string planner, int benchmark_idx) {
    if (m_results_for_idx.size() == 0) {
        std::cout << "Success rate: not any valid measurements." << std::endl;
        return;
    }
    // print the average success rate
    std::cout << "Success rate: " << std::accumulate(m_results_for_idx.begin(), m_results_for_idx.end(), 0.0) / m_results_for_idx.size() << std::endl;
    std::cout << "  " << m_results_for_idx.size() << " measurements." << std::endl;
    m_file << last_description << ", " << benchmark_idx << ", ";
    for (size_t i = 0; i < m_results_for_idx.size(); ++i) {
        m_file << m_results_for_idx[i];
        if (i < m_results_for_idx.size() - 1) {
            m_file << ", ";
        }
    }
    m_file << std::endl;

    m_results_for_idx.clear();
}

// Computation Time
void ResultHandlerComputingTime::handleResult(const BenchmarkResult& result, std::string, int, int) {
    // if not all are true, return
    if (std::all_of(result.success.begin(), result.success.end(), [](bool b) { return !b; }))
        return;
    m_results_for_idx.push_back(result.duration_micros);
    last_description = result.description;
}

void ResultHandlerComputingTime::printSummary(std::string planner, int benchmark_idx) {
    if (m_results_for_idx.size() == 0) {
        std::cout << "Computation time: not any valid measurements." << std::endl;
        return;
    }
    // print the average computation time, min and max
    std::cout << "Computation time: " << std::endl;
    double avg = std::accumulate(m_results_for_idx.begin(), m_results_for_idx.end(), 0.0) / m_results_for_idx.size();
    std::cout << "  Average: " << avg << " microseconds" << std::endl;
    std::cout << "  Std. Dev.: " << std_dev(avg, m_results_for_idx) << std::endl;
    std::cout << "  Min: " << *std::min_element(m_results_for_idx.begin(), m_results_for_idx.end()) << " microseconds" << std::endl;
    std::cout << "  Max: " << *std::max_element(m_results_for_idx.begin(), m_results_for_idx.end()) << " microseconds" << std::endl;
    std::cout << "  " << m_results_for_idx.size() << " measurements." << std::endl;

    m_file << last_description << ", " << benchmark_idx << ", ";
    for (size_t i = 0; i < m_results_for_idx.size(); ++i) {
        m_file << m_results_for_idx[i];
        if (i < m_results_for_idx.size() - 1) {
            m_file << ", ";
        }
    }
    m_file << std::endl;

    m_results_for_idx.clear();
}

// Theta Changes
void ResultHandlerThetaChanges::handleResult(const BenchmarkResult& result, std::string, int, int) {
    // iterate over result.path, compute the angle for each line
    // and add it to the vector

    last_description = result.description;
    for (size_t query_idx = 0; query_idx < result.success.size(); query_idx++) {
        if (!result.success[query_idx] || result.path[query_idx].size() < 2)
            return;

        double total_diff = 0.0;
        for (size_t i = 2; i < result.path[query_idx].size(); ++i) {
            double angle1 = std::atan2(result.path[query_idx][i - 1].y() - result.path[query_idx][i - 2].y(),
                                       result.path[query_idx][i - 1].x() - result.path[query_idx][i - 2].x());
            double angle2 = std::atan2(result.path[query_idx][i].y() - result.path[query_idx][i - 1].y(),
                                       result.path[query_idx][i].x() - result.path[query_idx][i - 1].x());
            total_diff += std::abs(angle1 - angle2);
        }

        m_results_for_idx.push_back(total_diff);
    }
}

void ResultHandlerThetaChanges::printSummary(std::string planner, int benchmark_idx) {
    if (m_results_for_idx.size() == 0) {
        std::cout << "Theta changes: not any valid measurements." << std::endl;
        return;
    }

    // print average, min and max theta changes
    std::cout << "Theta changes: " << std::endl;
    double avg = std::accumulate(m_results_for_idx.begin(), m_results_for_idx.end(), 0.0) / m_results_for_idx.size();
    std::cout << "  Average: " << avg << " rad" << std::endl;
    std::cout << "  Std. Dev.: " << std_dev(avg, m_results_for_idx) << std::endl;
    std::cout << "  Min: " << *std::min_element(m_results_for_idx.begin(), m_results_for_idx.end()) << " rad" << std::endl;
    std::cout << "  Max: " << *std::max_element(m_results_for_idx.begin(), m_results_for_idx.end()) << " rad" << std::endl;
    std::cout << "  " << m_results_for_idx.size() << " measurements." << std::endl;

    m_file << last_description << ", " << benchmark_idx << ", ";
    for (size_t i = 0; i < m_results_for_idx.size(); ++i) {
        m_file << m_results_for_idx[i];
        if (i < m_results_for_idx.size() - 1) {
            m_file << ", ";
        }
    }
    m_file << std::endl;

    m_results_for_idx.clear();
}

// Summary of waiting moves performed by T-PRM
void ResultHandlerTPRMWaiting::handleResult(const BenchmarkResult& result, std::string planner, int, int) {
    for (size_t query_idx = 0; query_idx < result.success.size(); query_idx++) {
        if (!result.success[query_idx] || result.path[query_idx].size() == 0)
            return;
        if (planner == "T-PRM") {
            // iterate over the path, check if two successors are close to each other, if so, increment m_waiting
            for (size_t i = 0; i < result.path.size() - 1; ++i) {
                ;
                if ((result.path[query_idx][i] - result.path[query_idx][i + 1]).squaredNorm() < 0.01) {
                    ++m_waiting;
                }
            }
        }
    }
}

void ResultHandlerTPRMWaiting::printSummary(std::string planner, int benchmark_idx) {
    if (planner == "T-PRM") {
        std::cout << "Waiting moves: " << m_waiting << std::endl;
    }
    m_waiting = 0;
}

// Path Writer
void ResultHandlerPathWriter::handleResult(const BenchmarkResult& result, std::string planner, int benchmark_idx, int run_idx) {
    for (int query_idx = 0; query_idx < result.success.size(); query_idx++) {
        std::ofstream path_file("path_" + planner + "_" + std::to_string(benchmark_idx) + "_" + std::to_string(run_idx) + "_" + std::to_string(query_idx) +
                                ".path");
        if (result.success[query_idx] && result.path[query_idx].size() > 1)
            for (size_t i = 1; i < result.path[query_idx].size(); i++) {
                path_file << result.path[query_idx][i - 1].x() << " " << result.path[query_idx][i - 1].y() << " " << result.path[query_idx][i - 1].z() << " "
                          << result.path[query_idx][i].x() << " " << result.path[query_idx][i].y() << " " << result.path[query_idx][i].z();
                if (result.path_durations.size() && result.timing_results.size() > 0) {
                    path_file << " " << result.path_durations[query_idx][i];
                }
                path_file << std::endl;
            }
        path_file.close();
    }
}

// All writer
ResultHandlerAllWriter::ResultHandlerAllWriter() : ResultHandler("All Writer"), m_file("all_data.csv") {
    m_file << "Planner,Benchmark Index,Run Index,Description,Duration\n";
}
void ResultHandlerAllWriter::handleResult(const BenchmarkResult& result, std::string planner, int benchmark_idx, int run_idx) {
    for (const auto& r : result.timing_results) {
        m_file << planner << "," << benchmark_idx << "," << run_idx << ",";
        m_file << r.description << "," << r.duration_micros << "\n";
    }
    m_file << planner << "," << benchmark_idx << "," << run_idx << ",FULL DURATION," << result.duration_micros << std::endl;
}

}  // namespace benchmarking
