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

#include <tprm/temporal_graph.h>

#include <deque>
#include <unordered_map>
#include <unordered_set>

namespace tprm {

TimeAvailability::TimeAvailability(double start_time, double end_time) : start(start_time), end(end_time) {
    if (start > end) {
        throw std::invalid_argument("start_time > end_time");
    }
    if (start < 0.0) {
        throw std::invalid_argument("start_time < 0.0");
    }
    if (end < 0.0) {
        throw std::invalid_argument("end_time < 0.0");
    }
}

bool TemporalGraphNode::isActiveAt(double time) const {
    if (time < 0.0) {
        throw std::invalid_argument("time < 0.0");
    }
    for (const TimeAvailability& time_availability : time_availabilities) {
        if (time >= time_availability.start && time <= time_availability.end) {
            return true;
        }
    }
    return false;
}

int TemporalGraphEdge::getOtherNodeId(int node_id) const {
    if (node_id == node_a_id) {
        return node_b_id;
    } else if (node_id == node_b_id) {
        return node_a_id;
    } else {
        throw std::runtime_error("Node id not found in edge");
    }
}

TemporalGraph::TemporalGraph() {
    m_edge_cost_function = std::bind(&TemporalGraph::defaultEdgeCostFunction, this, std::placeholders::_1);
}

void TemporalGraph::clear() {
    m_nodes.clear();
    m_edges.clear();
}

int TemporalGraph::addNode(const TemporalGraphNode& node) {
    m_nodes.push_back(node);
    return m_nodes.size() - 1;
}

int TemporalGraph::addEdge(const TemporalGraphEdge& edge) {
    m_edges.push_back(edge);
    return m_edges.size() - 1;
}

const TemporalGraphNode& TemporalGraph::getNode(int node_id) const {
    if (node_id < 0 || node_id >= m_nodes.size()) {
        throw std::runtime_error("Node id not found");
    }
    return m_nodes[node_id];
}

TemporalGraphNode& TemporalGraph::getNode(int node_id) {
    if (node_id < 0 || node_id >= m_nodes.size()) {
        throw std::runtime_error("Node id not found");
    }
    return m_nodes[node_id];
}

const TemporalGraphEdge& TemporalGraph::getEdge(int edge_id) const {
    if (edge_id < 0 || edge_id >= m_edges.size()) {
        throw std::runtime_error("Edge id not found");
    }
    return m_edges[edge_id];
}

int TemporalGraph::getNumNodes() const {
    return m_nodes.size();
}

int TemporalGraph::getNumEdges() const {
    return m_edges.size();
}

int TemporalGraph::getClosestNode(const Vector3d& position) const {
    int closest_node_id = -1;
    double closest_node_distance = std::numeric_limits<double>::max();
    for (int node_id = 0; node_id < m_nodes.size(); node_id++) {
        double distance = (m_nodes[node_id].position - position).squaredNorm();  // use squared distance to avoid sqrt
        if (distance < closest_node_distance) {
            closest_node_distance = distance;
            closest_node_id = node_id;
        }
    }
    return closest_node_id;
}

/**
 * @brief Helper struct for special double.
 * 
 * This struct allows to initialize an array / list / queue with infinite values.
 * Used in TemporalGraph::getShortestPath
 */
struct CustomDouble {
    double value = std::numeric_limits<double>::infinity();
};

std::vector<GraphPathResultEntry> TemporalGraph::getShortestPath(int start_node_id, int end_node_id, double start_time) const {
    if (start_node_id < 0 || start_node_id >= m_nodes.size()) {
        throw std::runtime_error("Start node id not found");
    }
    if (end_node_id < 0 || end_node_id >= m_nodes.size()) {
        throw std::runtime_error("End node id not found");
    }

    std::vector<std::vector<TemporalGraphEdge>> edge_buckets(
        m_nodes.size(), std::vector<TemporalGraphEdge>());  // bucket for each node. Each bucket contains all edges that start at the node
    for (const TemporalGraphEdge& edge : m_edges) {
        edge_buckets[edge.node_a_id].push_back(edge);
        edge_buckets[edge.node_b_id].push_back(edge);
    }

    std::unordered_map<int, int> predecessor_map;   // predecessor of each node
    std::unordered_map<int, CustomDouble> g_costs;  // g-cost of each node
    std::unordered_map<int, CustomDouble> f_costs;  // f-cost of each node

    const TemporalGraphNode& goal_node = m_nodes[end_node_id];

    std::function<double(int)> heuristic_function = [&](int node_id) { return m_edge_cost_function(TemporalGraphEdge(node_id, end_node_id)); };

    // sort function for the open queue
    auto compare = [&](int a, int b) { return f_costs[a].value < f_costs[b].value; };

    std::deque<int> open_queue;
    std::unordered_set<int> open_queue_set;
    std::unordered_set<int> closed_list_set;

    std::unordered_map<int, CustomDouble> arrival_time;

    g_costs[start_node_id].value = 0.;  // init this node by access (--> inf value) and set 0
    f_costs[start_node_id].value = heuristic_function(start_node_id);

    open_queue.push_back(start_node_id);
    open_queue_set.insert(start_node_id);
    arrival_time[start_node_id].value = start_time;

    auto handle_successor = [&](int current_id, int successor_id, double current_time, const TemporalGraphEdge& edge, double edge_time) {
        if (closed_list_set.find(successor_id) != closed_list_set.end()) {
            return;
        }

        double tentative_g_cost = g_costs[current_id].value + getEdgeCost(edge);

        auto open_queue_set_it = open_queue_set.find(successor_id);

        if (open_queue_set_it != open_queue_set.end()) {
            if (tentative_g_cost >= g_costs[successor_id].value) {
                return;
            }
        }

        predecessor_map[successor_id] = current_id;
        g_costs[successor_id].value = tentative_g_cost;
        f_costs[successor_id].value = g_costs[successor_id].value + heuristic_function(successor_id);
        arrival_time[successor_id].value = current_time + edge_time;

        if (open_queue_set_it == open_queue_set.end()) {
            open_queue.push_back(successor_id);
            open_queue_set.insert(successor_id);
        }
    };

    while (!open_queue.empty()) {
        int current_id = open_queue.front();
        open_queue.pop_front();
        open_queue_set.erase(current_id);

        if (current_id == end_node_id) {
            break;
        }

        closed_list_set.insert(current_id);
        for (const TemporalGraphEdge& edge : edge_buckets[current_id]) {
            int successor_id = edge.getOtherNodeId(current_id);

            double current_time = arrival_time[current_id].value;
            double edge_time = (getNode(current_id).position - getNode(successor_id).position).norm() / HolonomicRobot::movement_speed;

            // check TA
            if (m_nodes[successor_id].isActiveAt(current_time + edge_time)) {
                handle_successor(current_id, successor_id, current_time, edge, edge_time);
            }
        }

        std::sort(open_queue.begin(), open_queue.end(), compare);
    }

    // check if goal is reached (arrival time < INF)
    if (arrival_time[end_node_id].value == std::numeric_limits<double>::infinity()) {
        return std::vector<GraphPathResultEntry>();
    }

    // reconstruct path by backtracking
    std::vector<GraphPathResultEntry> path;
    int current_id = end_node_id;
    while (current_id != start_node_id) {
        path.push_back(GraphPathResultEntry(current_id, arrival_time[current_id].value));
        current_id = predecessor_map[current_id];
    }
    path.push_back(GraphPathResultEntry(current_id, arrival_time[current_id].value));

    std::reverse(path.begin(), path.end());
    return path;
}

void TemporalGraph::setEdgeCostFunction(std::function<double(const TemporalGraphEdge&)> edge_cost_function) {
    m_edge_cost_function = edge_cost_function;
}

double TemporalGraph::defaultEdgeCostFunction(const TemporalGraphEdge& edge) const {
    return (getNode(edge.node_a_id).position - getNode(edge.node_b_id).position).norm();
};

double TemporalGraph::getEdgeCost(const TemporalGraphEdge& edge) const {
    return m_edge_cost_function(edge);
}

} /* namespace tprm */