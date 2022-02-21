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

#ifndef __TPRM_TEMPORAL_GRAPH_H__
#define __TPRM_TEMPORAL_GRAPH_H__

#include <tprm/config.h>

#include <functional>  // for std::function
#include <vector>      // for std::vector

namespace tprm {

/**
 * @brief Struct to store time availability of a node.
 */
struct TimeAvailability {
    /**
     * @brief Constructor.
     * @param start_time The start time of the node.
     * @param end_time The end time of the node.
     * @throw std::invalid_argument if start_time > end_time or any is below 0.
     */
    TimeAvailability(double start_time, double end_time);
    double start;  ///< The start time of the interval.
    double end;    ///< The end time of the interval.
};

/**
 * @brief Class to represent a temporal graph node.
 */
struct TemporalGraphNode {
    /**
     * @brief Constructor.
     * @param position The position of the node.
     * @param time_availability The time availability of the node (a vector of intervals)
     */
    TemporalGraphNode(const Vector3d& position, const std::vector<TimeAvailability>& time_availabilities)
        : position(position), time_availabilities(time_availabilities) {}

    Vector3d position;  ///< The position of the node.

    std::vector<TimeAvailability> time_availabilities;  ///< The time availability of the node.

    /**
     * @brief Checks if the node is available at a given time.
     * @param time The time to check.
     * @return True if the node is available at the given time.
     * @throw std::invalid_argument if time is below 0.
     */
    bool isActiveAt(double time) const;
};

/**
 * @brief Class to represent a temporal graph edge.
 * 
 * The edge is undirected.
 */
struct TemporalGraphEdge {
    /**
     * @brief Constructor.
     * @param node_a The first node of the edge.
     * @param node_b The second node of the edge.
     */
    TemporalGraphEdge(int node_a, int node_b) : node_a_id(node_a), node_b_id(node_b) {}
    int node_a_id = -1;  ///< The id of the first node of the edge.
    int node_b_id = -1;  ///< The id of the second node of the edge.

    /**
     * @brief Helper function to get the other end.
     * @param node_id The id of the node.
     * @return The id of the other end of the edge.
     * @throw std::runtime_error if the edge is not connected to the given node.
     */
    int getOtherNodeId(int node_id) const;
};

/**
 * @brief Class to represent a path entry for the found path.
 * 
 * An entry consists of a node and the time at which the node is reached.
 */
struct GraphPathResultEntry {
    /**
     * @brief Constructor.
     * @param node_id The node of the entry.
     * @param time The time at which the node is reached.
     */
    GraphPathResultEntry(int node_id, double time) : node_id(node_id), time(time){};
    int node_id = -1;  ///< The id of the node.
    double time = -1;  ///< The time at which the node is reached.
};

/**
 * @brief Class to represent a T-PRM graph.
 */
class TemporalGraph {
public:
    /**
     * @brief Constructor.
     */
    TemporalGraph();

    /**
     * @brief Destructor.
     */
    virtual ~TemporalGraph() = default;

    /**
     * @brief Clear the graph (deletes all nodes & edges).
     */
    void clear();

    /**
     * @brief Add a node to the graph.
     * @param node The node to add.
     * @return The id of the added node.
     */
    int addNode(const TemporalGraphNode& node);

    /**
     * @brief Add an edge to the graph.
     * @param edge The edge to add.
     * @return The id of the added edge.
     */
    int addEdge(const TemporalGraphEdge& edge);

    /**
     * @brief Get the node with the given id.
     * @param id The id of the node.
     * @return The node with the given id.
     * @throw std::runtime_error if no node with the given id exists.
     */
    const TemporalGraphNode& getNode(int id) const;

    /**
     * @brief Get a mutable reference to the node with the given id.
     * @param id The id of the node.
     * @return The node with the given id.
     * @throw std::runtime_error if no node with the given id exists.
     */
    TemporalGraphNode& getNode(int id);

    /**
     * @brief Get the edge with the given id.
     * @param id The id of the edge.
     * @return The edge with the given id.
     * @throw std::runtime_error if no edge with the given id exists.
     */
    const TemporalGraphEdge& getEdge(int id) const;

    /**
     * @brief Get the number of nodes in the graph.
     * @return The number of nodes in the graph.
     */
    int getNumNodes() const;

    /**
     * @brief Get the number of edges in the graph.
     * @return The number of edges in the graph.
     */
    int getNumEdges() const;

    /**
     * @brief Get the id of the node which is closest to the given position.
     * @param position The position to get the closest node to.
     * @return The id of the node with the given position.
     * @throw std::runtime_error if no node with the given position exists.
     */
    int getClosestNode(const Vector3d& position) const;

    /**
     * @brief Get the shortest path from the given start node to the given end node.
     * 
     * The first node is visited with the given start time (used for dynamic obstacles).
     * @param start_node_id The id of the start node.
     * @param end_node_id The id of the end node.
     * 
     * @throws std::runtime_error if either the start or end node does not exist.
     * 
     * @return The shortest path from the given start node to the given end node (empty if no path exists).
     */
    std::vector<GraphPathResultEntry> getShortestPath(int start_node_id, int end_node_id, double start_time = 0.) const;

    /**
     * @brief Set the edge cost function.
     * 
     * The edge cost function is used to calculate the cost of an edge.
     * The default edge cost function is the Euclidean distance between the two nodes.
     * @param edge_cost_function The edge cost function.
     */
    void setEdgeCostFunction(std::function<double(const TemporalGraphEdge&)> cost_function);

    /**
     * @brief Get the edge cost for an edge.
     * @param edge The edge to get the cost for.
     * @return The edge cost for the given edge.
     */
    double getEdgeCost(const TemporalGraphEdge& edge) const;

private:
    /**
     * @brief Get the edge cost between two nodes.
     * 
     * This is the default cost, which is the Euclidean distance between the two nodes.
     * @param edge The edge to get the cost for.
     * @return The cost of the edge.
     */
    double defaultEdgeCostFunction(const TemporalGraphEdge& edge) const;

private:
    /**
     * @brief Get the cost of the given edge.
     * default: distance between the two nodes.
     */
    std::function<double(const TemporalGraphEdge&)> m_edge_cost_function;

    /**
     * @brief The nodes of the graph.
     */
    std::vector<TemporalGraphNode> m_nodes;
    /**
     * @brief The edges of the graph.
     */
    std::vector<TemporalGraphEdge> m_edges;
};

} /* namespace tprm */

#endif /* __TPRM_TEMPORAL_GRAPH_H__ */