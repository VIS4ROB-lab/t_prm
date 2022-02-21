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

#include <omp.h>
#include <tprm/temporal_prm.h>

#include <random>

namespace tprm {

TemporalPRM::TemporalPRM(const Vector3d& environmentMin, const Vector3d& environmentMax) : m_environmentMin(environmentMin), m_environmentMax(environmentMax) {}

void TemporalPRM::setEnvironmentMin(const Vector3d& environmentMin) {
    m_environmentMin = environmentMin;
}
void TemporalPRM::setEnvironmentMax(const Vector3d& environmentMax) {
    m_environmentMax = environmentMax;
}

void TemporalPRM::addStaticObstacle(const std::shared_ptr<StaticObstacle>& obstacle) {
    m_staticObstacles.push_back(obstacle);
}

void TemporalPRM::addDynamicObstacle(const std::shared_ptr<DynamicObstacle>& obstacle) {
    m_dynamicObstacles.push_back(obstacle);
}

void TemporalPRM::placeSamples(int numNodes) {
    const Vector3d half = Vector3d::Constant(0.5);
    const Vector3d envSize = m_environmentMax - m_environmentMin;
    for (int i = 0; i < numNodes; i++) {
        bool is_blocked = true;

        Vector3d sample = (Vector3d::Random() * 0.5 + half).cwiseProduct(envSize) + m_environmentMin;
        while (is_blocked) {
            is_blocked = false;
            for (const auto& obstacle : m_staticObstacles) {
                if (obstacle->isColliding(sample)) {
                    is_blocked = true;
                    sample = (Vector3d::Random() * 0.5 + half).cwiseProduct(envSize) + m_environmentMin;
                    break;
                }
            }
        }
        m_graph.addNode(TemporalGraphNode(sample, getTimeAvailability(sample)));
    }
}

void TemporalPRM::buildPRM(double costEdgeThreshold) {
    const int loop_end = m_graph.getNumNodes();
#pragma omp parallel for schedule(dynamic, 1) shared(m_graph) firstprivate(costEdgeThreshold, m_staticObstacles, loop_end) default(none)
    for (int i = 0; i < loop_end; i++) {
        for (int j = i + 1; j < loop_end; j++) {
            const TemporalGraphNode& node_i = m_graph.getNode(i);
            const TemporalGraphNode& node_j = m_graph.getNode(j);

            double cost = m_graph.getEdgeCost(TemporalGraphEdge(i, j));
            if (cost > costEdgeThreshold) {
                continue;
            }

            bool is_blocked = false;
            for (const auto& obstacle : m_staticObstacles) {
                if (obstacle->isColliding(node_i.position, node_j.position)) {
                    is_blocked = true;
                    break;
                }
            }
            if (is_blocked) {
                continue;
            }

#pragma omp critical
            m_graph.addEdge(TemporalGraphEdge(i, j));
        }
    }
}

void TemporalPRM::recomputeTimeAvailabilities() {
    for (int i = 0; i < m_graph.getNumNodes(); i++) {
        m_graph.getNode(i).time_availabilities = getTimeAvailability(m_graph.getNode(i).position);
    }
}

TemporalGraph& TemporalPRM::getTemporalGraph() {
    return m_graph;
}

std::vector<TimeAvailability> TemporalPRM::getTimeAvailability(const Vector3d& position) const {
    if (m_dynamicObstacles.empty()) {
        return {TimeAvailability(0, std::numeric_limits<double>::infinity())};
    }

    std::vector<TimeAvailability> time_availabilities;
    struct HitInfo {
        double t_start, t_end;
    };

    std::vector<HitInfo> hit_infos;
    for (const auto& obstacle : m_dynamicObstacles) {
        double t_start, t_end;
        if (obstacle->isColliding(position, t_start, t_end)) {
            hit_infos.push_back({t_start, t_end});
        }
    }

    time_availabilities.push_back({0, std::numeric_limits<double>::infinity()});

    if (hit_infos.empty()) {
        return time_availabilities;
    }

    auto apply_obstacle_hit_to_interval = [&time_availabilities](const HitInfo& hi, TimeAvailability& interval) {
        if (hi.t_start > interval.start && hi.t_end < interval.end) {
            // create two intervals from this
            // this: slice.from_time, hi.t_start
            // new: hi.t_end, slice.to_time
            TimeAvailability new_interval(hi.t_end, interval.end);
            interval.end = hi.t_start;
            time_availabilities.push_back(new_interval);
        } else if (hi.t_start > interval.start && hi.t_start < interval.end) {
            // create an interval from this
            // this: slice.from_time, hi.t_start
            interval.end = hi.t_start;
        } else if (hi.t_end > interval.start && hi.t_end < interval.end) {
            // create an interval from this
            // this: hi.t_end, slice.to_time
            interval.start = hi.t_end;
        } else if (hi.t_start < interval.start && hi.t_end > interval.end) {
            // do nothing
        } else if (fabs(hi.t_start - interval.start) < 1e-6 && fabs(hi.t_end - interval.end) < 1e-6) {
            // this point is blocked any time by the moving obstacle.
            return false;
        }
        return true;
    };

    for (size_t i = 0; i < time_availabilities.size(); i++) {  // will be increased in this loop, therefore i indexed
        for (const auto& hi : hit_infos) {                     // does not change, therefore range loop
            if (!apply_obstacle_hit_to_interval(hi, time_availabilities[i])) {
                return {};
            }
        }
    }
    return time_availabilities;
}

std::vector<PathResultEntry> TemporalPRM::getShortestPath(const Vector3d& start, const Vector3d& goal, double timeStart) const {
    int closest_start_id = m_graph.getClosestNode(start);
    int closest_goal_id = m_graph.getClosestNode(goal);

    const TemporalGraphNode& start_node = m_graph.getNode(closest_start_id);
    const TemporalGraphNode& goal_node = m_graph.getNode(closest_goal_id);

    // try to connect start and goal to their closest nodes
    // check static obstacles
    bool is_blocked = false;
    for (const auto& obstacle : m_staticObstacles) {
        if (obstacle->isColliding(start, start_node.position)) {
            is_blocked = true;
            break;
        }
    }
    if (is_blocked) {
        return {};
    }
    is_blocked = false;
    for (const auto& obstacle : m_staticObstacles) {
        if (obstacle->isColliding(goal, goal_node.position)) {
            is_blocked = true;
            break;
        }
    }
    if (is_blocked) {
        return {};
    }

    // check dynamic obstacles
    TemporalGraphNode tmp_start(start, getTimeAvailability(start));
    if (!tmp_start.isActiveAt(timeStart)) {
        return {};
    }

    double time_to_closest_start = (start - start_node.position).norm() / HolonomicRobot::movement_speed;

    auto path = m_graph.getShortestPath(closest_start_id, closest_goal_id, timeStart + time_to_closest_start);
    if (path.empty()) {
        return {};
    }

    TemporalGraphNode tmp_goal(goal, getTimeAvailability(goal));
    double time_from_closest_goal = (goal - goal_node.position).norm() / HolonomicRobot::movement_speed;
    if (!tmp_goal.isActiveAt(path.back().time + time_from_closest_goal)) {
        return {};
    }

    std::vector<PathResultEntry> result;
    // push back custom start
    result.push_back(PathResultEntry(start, timeStart));
    for (const auto& entry : path) {
        result.push_back(PathResultEntry(m_graph.getNode(entry.node_id).position, entry.time));
    }
    result.push_back(PathResultEntry(goal, path.back().time + time_from_closest_goal));
    return result;
}

} /* namespace tprm */