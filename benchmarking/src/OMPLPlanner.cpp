#include <benchmarking/OMPLPlanner.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <fstream>

//#define OMPL_USE_NUM_NODES
#define OMPL_USE_EDGE_LENGTH

namespace ompl::geometric {

class MyPRM : public PRM {
public:
    MyPRM(ompl::base::SpaceInformationPtr si, double edgeThreshold) : PRM(si), m_edgeThreshold(edgeThreshold) {
        setConnectionFilter(std::bind(&MyPRM::connectionFilter, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    bool connectionFilter(const Vertex& v1, const Vertex& v2) const {
        return PRM::distanceFunction(v1, v2) < m_edgeThreshold;
    }
    double m_edgeThreshold;
};
}  // namespace ompl::geometric

namespace benchmarking {

class ValidityChecker : public ompl::base::StateValidityChecker {
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr& si, std::shared_ptr<benchmarking::Benchmark> bm) : ompl::base::StateValidityChecker(si), benchmark(bm) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ompl::base::State* state) const {
        return this->clearance(state) > 0.0;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ompl::base::State* state) const {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ompl::base::RealVectorStateSpace::StateType* state2D = state->as<ompl::base::RealVectorStateSpace::StateType>();

        double min_dist = 1e4;
        if (benchmark->is_2d) {
            double x = state2D->values[0];
            double y = state2D->values[1];
            Eigen::Vector2d pos(x, y);
            double min_dist = 1e4;
            for (const benchmarking::Circle& c : benchmark->circles) {
                // get minimum distance to circle
                double dist = (pos - c.center.head(2)).norm() - c.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }
        } else {  // 3D case
            // Extract the robot's (x,y) position from its state
            double x = state2D->values[0];
            double y = state2D->values[1];
            double z = state2D->values[2];
            Eigen::Vector3d pos(x, y, z);

            // static obstacles:
            double min_dist = 1e4;
            for (const benchmarking::Circle& c : benchmark->circles) {
                // get minimum distance to circle
                double dist = (pos - c.center).norm() - c.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }
        }

        return min_dist;
    }

private:
    std::shared_ptr<benchmarking::Benchmark> benchmark;
};

class ValidityCheckerDynObst : public ompl::base::StateValidityChecker {
public:
    ValidityCheckerDynObst(const ompl::base::SpaceInformationPtr& si, std::shared_ptr<benchmarking::Benchmark> bm, double time)
        : ompl::base::StateValidityChecker(si), benchmark(bm), m_time(time) {}

    // Returns whether the given state's position overlaps the
    // circular obstacle
    bool isValid(const ompl::base::State* state) const {
        return this->clearance(state) > 0.0;
    }

    void set_time(double time) {
        m_time = time;
    }

    // Returns the distance from the given state's position to the
    // boundary of the circular obstacle.
    double clearance(const ompl::base::State* state) const {
        // We know we're working with a RealVectorStateSpace in this
        // example, so we downcast state into the specific type.
        const ompl::base::RealVectorStateSpace::StateType* state2D = state->as<ompl::base::RealVectorStateSpace::StateType>();

        double min_dist = 1e4;
        if (benchmark->is_2d) {
            // Extract the robot's (x,y) position from its state
            double x = state2D->values[0];
            double y = state2D->values[1];
            Eigen::Vector2d pos(x, y);

            // static obstacles:

            for (const benchmarking::Circle& c : benchmark->circles) {
                // get minimum distance to circle
                double dist = (pos - c.center.head(2)).norm() - c.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }

            for (const benchmarking::MovingCircle& mc : benchmark->moving_circles) {
                Eigen::Vector3d pos_mc = mc.center + mc.velocity * m_time;
                double dist = (pos - pos_mc.head(2)).norm() - mc.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }
        } else {  // 3D
            // Extract the robot's (x,y) position from its state
            double x = state2D->values[0];
            double y = state2D->values[1];
            double z = state2D->values[2];
            Eigen::Vector3d pos(x, y, z);

            // static obstacles:

            for (const benchmarking::Circle& c : benchmark->circles) {
                // get minimum distance to circle
                double dist = (pos - c.center).norm() - c.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }

            for (const benchmarking::MovingCircle& mc : benchmark->moving_circles) {
                Eigen::Vector3d pos_mc = mc.center + mc.velocity * m_time;
                double dist = (pos - pos_mc).norm() - mc.radius;
                if (dist < min_dist)
                    min_dist = dist;
            }
        }

        return min_dist;
    }

private:
    std::shared_ptr<benchmarking::Benchmark> benchmark;
    double m_time;
};

OMPLPlannerBenchmark::OMPLPlannerBenchmark(std::string planner) : m_planner(planner) {
    ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_NONE);
}

ompl::base::OptimizationObjectivePtr getThresholdPathLengthObj(const ompl::base::SpaceInformationPtr& si, double cost) {
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ompl::base::Cost(cost));
    return obj;
}

void write_intermediate_path(std::shared_ptr<Benchmark> benchmark, const std::vector<Eigen::Vector3d>& path, double time, std::string name) {
    if (!benchmark->writeMovingIntermediatePaths)
        return;

    std::ofstream of("INTERMEDIATE_PATH_" + name + "_AT_" + std::to_string(time) + ".txt");
    if (path.size() == 0) {
        return;
    }
    for (int i = 0; i < path.size() - 1; i++) {
        of << path[i].x() << " " << path[i].y() << " " << path[i].z() << " " << path[i + 1].x() << " " << path[i + 1].y() << " " << path[i + 1].z()
           << std::endl;
    }
}

BenchmarkResult OMPLPlannerBenchmark::runBenchmark(std::shared_ptr<Benchmark> benchmark, int benchmark_idx, int run_idx) {
    if (benchmark->moving_circles.size() == 0) {
        BasePlannerBenchmark::startBenchmark();

        BenchmarkResult result;

        ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2 + !benchmark->is_2d));
        space->as<ompl::base::RealVectorStateSpace>()->setBounds(0, benchmark->domain_size);

        ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

        ValidityChecker* validityChecker = new ValidityChecker(si, benchmark);

        si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(validityChecker));
        si->setup();

        ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
        pdef->setOptimizationObjective(getThresholdPathLengthObj(si, benchmark->ompl_path_length_threshold));

        ompl::base::PlannerPtr planner;
        if (m_planner == "RRTstar") {
            planner = std::make_shared<ompl::geometric::RRTstar>(si);
            dynamic_cast<ompl::geometric::RRTstar*>(planner.get())->setRange(benchmark->rrt_edge_length);
        } else if (m_planner == "PRM") {
#ifdef OMPL_USE_EDGE_LENGTH
            planner = std::make_shared<ompl::geometric::MyPRM>(si, benchmark->rrt_edge_length);
#else
            planner = std::make_shared<ompl::geometric::PRM>(si);
#endif
        } else if (m_planner != "") {
            std::cerr << "Unknown planner: " << m_planner << std::endl;
            exit(1);
        }
        ompl::base::ScopedState<> start(space);
        ompl::base::ScopedState<> goal(space);
        for (size_t query_idx = 0; query_idx < benchmark->start.size(); query_idx++) {
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = benchmark->start[query_idx].x();
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = benchmark->start[query_idx].y();
            if (!benchmark->is_2d)
                start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = benchmark->start[query_idx].z();

            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = benchmark->goal[query_idx].x();
            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = benchmark->goal[query_idx].y();
            if (!benchmark->is_2d)
                goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = benchmark->goal[query_idx].z();

            if (query_idx > 0) {
                pdef->clearSolutionPaths();
                pdef->clearStartStates();
                pdef->clearGoal();

                if (m_planner == "PRM") {
                    // cast planner
                    static_cast<ompl::geometric::PRM*>(planner.get())->clearQuery();
                } else if (

                    m_planner == "RRTstar") {
                    static_cast<ompl::geometric::RRTstar*>(planner.get())->clear();
                }
            }
            pdef->setStartAndGoalStates(start, goal);
            planner->setProblemDefinition(pdef);
            planner->setup();

            startMeasurement();
#ifndef OMPL_USE_NUM_NODES
            ompl::base::PlannerStatus status = planner->solve(benchmark->ompl_time_limit);
#else
            // if we want to limit the number of nodes: (hacky!)
            ompl::base::PlannerStatus status;
            if (m_planner == "PRM") {
                status = planner->solve(ompl::base::PlannerTerminationCondition([&]() -> bool {
                    // return true if we should stop
                    auto planner_ = static_cast<ompl::geometric::PRM*>(planner.get());
                    return planner_->milestoneCount() >= benchmark->numNodes;
                }));
            } else {
                status = planner->solve(ompl::base::PlannerTerminationCondition([&]() -> bool {
                    // return true if we should stop
                    auto planner_ = static_cast<ompl::geometric::RRTstar*>(planner.get());
                    return planner_->numIterations() >= benchmark->numNodes;
                }));
            }
#endif
            result.success.push_back(status == ompl::base::PlannerStatus::EXACT_SOLUTION);

            result.timing_results.push_back(stopMeasurement("Solve query " + std::to_string(query_idx)));

            ompl::base::PathPtr path = pdef->getSolutionPath();
            if (path != nullptr) {
                ompl::geometric::PathGeometric& pathGeometric = dynamic_cast<ompl::geometric::PathGeometric&>(*path);
                auto states = pathGeometric.getStates();
                result.path.push_back({});
                for (auto state : states) {
                    if (!benchmark->is_2d) {
                        result.path.back().push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                                     state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                                                     state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]));
                    } else {  // pad with 0
                        result.path.back().push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                                     state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], 0.));
                    }
                }
            }
        }

        BasePlannerBenchmark::stopBenchmark();

        result.description = "OMPL " + m_planner + " - " + benchmark->name;
        result.duration_micros = getTotalDuration();

        return result;
    } else {
        // moving circles --> run OMPL iteratively
        BasePlannerBenchmark::startBenchmark();

        BenchmarkResult result;

        ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(2 + !benchmark->is_2d));
        space->as<ompl::base::RealVectorStateSpace>()->setBounds(0, benchmark->domain_size);

        ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

        ompl::base::StateValidityCheckerPtr validityChecker = std::make_shared<ValidityCheckerDynObst>(si, benchmark, 0.);
        si->setStateValidityChecker(validityChecker);

        si->setup();
        ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
        pdef->setOptimizationObjective(getThresholdPathLengthObj(si, benchmark->ompl_path_length_threshold));

        ompl::base::PlannerPtr planner;
        if (m_planner == "RRTstar") {
            planner = std::make_shared<ompl::geometric::RRTstar>(si);
            dynamic_cast<ompl::geometric::RRTstar*>(planner.get())->setRange(benchmark->rrt_edge_length);
        } else if (m_planner == "PRM") {
#ifdef OMPL_USE_EDGE_LENGTH
            planner = std::make_shared<ompl::geometric::MyPRM>(si, benchmark->rrt_edge_length);
#else
            planner = std::make_shared<ompl::geometric::PRM>(si);
#endif
        } else if (m_planner != "") {
            std::cerr << "Unknown planner: " << m_planner << std::endl;
            exit(1);
        }

        ompl::base::ScopedState<> start(space);
        ompl::base::ScopedState<> goal(space);
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = benchmark->start[0].x();
        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = benchmark->start[0].y();
        if (!benchmark->is_2d)
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = benchmark->start[0].z();

        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = benchmark->goal[0].x();
        goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = benchmark->goal[0].y();
        if (!benchmark->is_2d)
            goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = benchmark->goal[0].z();

        pdef->setStartAndGoalStates(start, goal);
        planner->setProblemDefinition(pdef);
        planner->setup();

        startMeasurement();

        ompl::base::PlannerStatus status = planner->solve(benchmark->ompl_time_limit);
        result.success.push_back(status == ompl::base::PlannerStatus::EXACT_SOLUTION);

        result.timing_results.push_back(stopMeasurement("Solve query " + std::to_string(0)));

        ompl::base::PathPtr path = pdef->getSolutionPath();
        std::vector<Eigen::Vector3d> path_;
        if (path != nullptr) {
            ompl::geometric::PathGeometric& pathGeometric = dynamic_cast<ompl::geometric::PathGeometric&>(*path);
            auto states = pathGeometric.getStates();

            for (auto state : states) {
                if (!benchmark->is_2d) {
                    path_.push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]));
                } else {  // pad with 0
                    path_.push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                    state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], 0.));
                }
            }
        }

        write_intermediate_path(benchmark, path_, 0., OMPLPlannerBenchmark::getName());

        double total_time = 0.;
        bool has_reached_end = false;
        std::vector<Eigen::Vector3d> full_path;
        bool in_collision = false;
        full_path.push_back(benchmark->start[0]);
        while (!has_reached_end) {
            in_collision = false;  // reset
            double time_on_current_path = 0.;

            if (path_.size() < 2) {
            /*std::cerr << "Path is too short (" << path_.size() << ")" << std::endl;
                for (auto& s : path_) {
                    std::cerr << s.transpose() << std::endl;
                }
                std::cerr << "Current start: " << start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] << " "
                          << start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] << " ";
                
                if (!benchmark->is_2d)
                    std::cerr << start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] << std::endl;
                */
                BasePlannerBenchmark::stopBenchmark();
                result.description = "OMPL " + m_planner + " - " + benchmark->name;
                result.duration_micros = getTotalDuration();
                result.success.back() = false;
                return result;
            }

            // search the current segment to use
            int current_segment = 0;
            //full_path.push_back(path_.at(current_segment));
            //std::cout << "Full path push back A: " << full_path.back().transpose() << std::endl;
            Eigen::Vector3d start_;
            double MOVE_FORWARD_TIME = 0.25;
            double time_on_segment = 0.;
            while (!in_collision) {
                while (current_segment < path_.size() - 1) {
                    double time_for_segment = (path_.at(current_segment + 1) - path_.at(current_segment)).norm() / 0.75;

                    double move_time = std::min(MOVE_FORWARD_TIME, time_for_segment - time_on_segment);
                    time_on_current_path += move_time;
                    time_on_segment += move_time;

                    if (time_on_segment >= time_for_segment) {
                        // we have reached the end of the segment
                        current_segment++;
                        time_on_segment = 0.;
                        if (current_segment == path_.size() - 1) {
                            has_reached_end = true;
                            break;
                        }
                        full_path.push_back(path_.at(current_segment));
                        start_ = path_.at(current_segment);  // update start
                    } else {
                        // we are still on the same segment
                        start_ = path_.at(current_segment) + (path_.at(current_segment + 1) - path_.at(current_segment)) * (time_on_segment / time_for_segment);
                        full_path.push_back(start_);
                    }

                    // COLLISION CHECK
                    auto path = pdef->getSolutionPath();
                    if (path != nullptr) {
                        ompl::geometric::PathGeometric& pathGeometric = dynamic_cast<ompl::geometric::PathGeometric&>(*path);
                        auto states = pathGeometric.getStates();
                        ValidityCheckerDynObst val_checker(si, benchmark, total_time + time_on_current_path);
                        for (auto state : states) {
                            if (!val_checker.isValid(state)) {
                                in_collision = true;
                                // this means, we have to replan
                                break;
                            }
                        }

                        if (!in_collision) {
                            //std::cout << "Can continue with time " << time_on_current_path << " on segment " << current_segment << " with time "
                            //          << time_for_segment << std::endl;
                        } else {
                            //std::cout << "Collision detected at time " << time_on_current_path << std::endl;
                            break;
                        }
                    }
                }
                if (has_reached_end) {
                    // propagate break
                    break;
                }
            }
            if (has_reached_end) {
                // propagate break
                break;
            }

            //std::cout << "Replanning at time " << total_time + time_on_current_path << std::endl;

            static_cast<ValidityCheckerDynObst*>(validityChecker.get())->set_time(total_time + time_on_current_path);

            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = start_.x();
            start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = start_.y();
            if (!benchmark->is_2d)
                start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = start_.z();

            // goal is the same

            si = std::make_shared<ompl::base::SpaceInformation>(space);
            si->setStateValidityChecker(validityChecker);
            si->setup();
            pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
            pdef->setOptimizationObjective(getThresholdPathLengthObj(si, benchmark->ompl_path_length_threshold));
            if (m_planner == "RRTstar") {
                planner = std::make_shared<ompl::geometric::RRTstar>(si);
                dynamic_cast<ompl::geometric::RRTstar*>(planner.get())->setRange(benchmark->rrt_edge_length);
            } else if (m_planner == "PRM") {
#ifdef OMPL_USE_EDGE_LENGTH
                planner = std::make_shared<ompl::geometric::MyPRM>(si, benchmark->rrt_edge_length);
#else

                planner = std::make_shared<ompl::geometric::PRM>(si);
#endif
            } else if (m_planner != "") {
                std::cerr << "Unknown planner: " << m_planner << std::endl;
                exit(1);
            }
            pdef->setStartAndGoalStates(start, goal);
            planner->setProblemDefinition(pdef);
            planner->setup();

            status = planner->solve(benchmark->ompl_time_limit);

            // and-ing success
            result.success.back() = result.success.back() & (status == ompl::base::PlannerStatus::EXACT_SOLUTION);
            path = pdef->getSolutionPath();
            path_.clear();
            if (path != nullptr) {
                ompl::geometric::PathGeometric& pathGeometric = dynamic_cast<ompl::geometric::PathGeometric&>(*path);
                auto states = pathGeometric.getStates();

                for (auto state : states) {
                    if (!benchmark->is_2d) {
                        path_.push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                                        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]));
                    } else {  // pad with 0.
                        path_.push_back(Eigen::Vector3d(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                                        state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], 0.));
                    }
                }
            }

            total_time += time_on_current_path;
            write_intermediate_path(benchmark, path_, total_time, OMPLPlannerBenchmark::getName());

            // reached end if start is close to goal 1e-3
            if (!benchmark->is_2d) {
                Eigen::Vector3d e_start(start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]);
                Eigen::Vector3d e_goal(goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                       goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1],
                                       goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]);
                has_reached_end = (e_start - e_goal).norm() < 1e-3;
            } else {
                Eigen::Vector2d e_start(start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                        start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]);
                Eigen::Vector2d e_goal(goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0],
                                       goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]);
                has_reached_end = (e_start - e_goal).norm() < 1e-3;
            }
        }

        // push back goal
        full_path.push_back(benchmark->goal[0]);

        result.path.push_back(full_path);

        BasePlannerBenchmark::stopBenchmark();

        result.description = "OMPL " + m_planner + " - " + benchmark->name;
        result.duration_micros = getTotalDuration();

        return result;
    }
}

}  // namespace benchmarking
