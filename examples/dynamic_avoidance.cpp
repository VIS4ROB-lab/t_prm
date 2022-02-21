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

#include <tprm/obstacle_impl.h>
#include <tprm/temporal_prm.h>

#include <iostream>

int main(int argc, char const* argv[]) {
    tprm::HolonomicRobot::movement_speed = 0.1;  // m/s

    std::srand(3);

    tprm::TemporalPRM tprm;

    // add a moving sphere, towards 0/0/0
    tprm.addDynamicObstacle(std::make_shared<tprm::DynamicSphereObstacle>(tprm::Vector3d::Constant(1.), tprm::Vector3d::Constant(-0.1), 0.25));

    tprm.placeSamples(150);

    tprm.buildPRM(0.25);

    std::cout << "Number of nodes: " << tprm.getTemporalGraph().getNumNodes() << std::endl;
    std::cout << "Number of edges: " << tprm.getTemporalGraph().getNumEdges() << std::endl;

    auto path = tprm.getShortestPath(tprm::Vector3d(0, 0, 0), tprm::Vector3d(1., 1., 1.), 0.5);

    if (path.empty()) {
        std::cout << "No path found" << std::endl;
    } else {
        for (auto& node : path) {
            std::cout << "Node: " << node.position.transpose() << " at time " << node.time << std::endl;
        }
    }

    return 0;
}