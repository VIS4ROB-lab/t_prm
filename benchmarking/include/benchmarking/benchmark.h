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
#include <benchmarking/obstacle.h>

#include <string>
#include <utility>

namespace benchmarking {

class Benchmark {
public:
    Benchmark(const std::string& name, int num_runs);
    ~Benchmark();

    int numRuns() const {
        return m_num_runs;
    }

    int numNodes = 500;

    // Domain size
    double domain_size = 10.;

    bool is_2d = false;

    std::vector<Eigen::Vector3d> start = {};
    std::vector<Eigen::Vector3d> goal = {};

    // Obstacles
    std::vector<Circle> circles = {};               // static
    std::vector<MovingCircle> moving_circles = {};  // dynamic

    bool writeMovingIntermediatePaths = false;

    // Individual
    // **********
    // RRT*-FND
    double ompl_edge_length = 1.;

    // T-PRM
    double tprm_cost_edge_threshold = 1.;

    // OMPL RRT*
    double ompl_time_limit = 1.0;
    double ompl_path_length_threshold = 1.0;

    std::string name;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    int m_num_runs;
};
}  // namespace benchmarking