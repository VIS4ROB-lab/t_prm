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
    int rrt_growIterations = 1000;
    double rrt_edge_length = 1.;
    double rrt_max_time = 5.;
    double rrt_timestep = 0.1;

    // T-PRM
    double tprm_max_time = 20.0;
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