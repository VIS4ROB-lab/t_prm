# T-PRM: Temporal Probabilistic Roadmap for Path Planning in Dynamic Environments 
This repository contains the source code for the T-PRM algorithm for path planning in dynamic environments.
If you use this code in your academic work, please cite ([PDF](https://www.research-collection.ethz.ch/handle/20.500.11850/560560), [Video](https://youtu.be/Eh6brn_dVlU)):

    @inproceedings{hueppi2022tprm,
      title={T-PRM: Temporal Probabilistic Roadmap for Path Planning in Dynamic Environments},
      author={Hüppi, Matthias and Bartolomei, Luca and Mascaro, Ruben and Chli, Margarita},
      booktitle={2022 {IEEE/RSJ} International Conference on Intelligent Robots and Systems ({IROS})},
      year={2022}
    }

> **Abstract**: Sampling-based motion planners are widely used in robotics due to their simplicity, flexibility and computational efficiency. However, in their most basic form, these algorithms operate under the assumption of static scenes and lack the ability to avoid collisions with dynamic (i.e. moving) obstacles. This raises safety concerns, limiting the range of possible applications of mobile robots in the real world. Motivated by these challenges, in this work we present Temporal-PRM, a novel sampling-based path-planning algorithm that performs obstacle avoidance in dynamic environments. The proposed approach extends the original Probabilistic Roadmap (PRM) with the notion of time, generating an augmented graph-like structure that can be efficiently queried using a time-aware variant of the A* search algorithm, also introduced in this paper. Our design maintains all the properties of PRM, such as the ability to perform multiple queries and to find smooth paths, while circumventing its downside by enabling collision avoidance in highly dynamic scenes with a minor increase in the computational cost. Through a series of challenging experiments in highly cluttered and dynamic environments, we demonstrate that the proposed path planner outperforms other state-of-the-art sampling-based solvers. Moreover, we show that our algorithm can run onboard a flying robot, performing obstacle avoidance in real time.

This project is released under a GPLv3 license.

## Dependencies
The code has been tested under Ubuntu 20.04 and does not have any special dependency. 

However, in order to run the benchmarks against OMPL, this library needs to be installed:
- If you have ROS installed, run `sudo apt install ros-noetic-ompl`;
- Otherwise run `sudo apt install libompl-dev`.

## Compiling
To compile the T-PRM library, execute the following steps:
1) Clone this repository: `git clone git@github.com:VIS4ROB-lab/t_prm.git`
2) Create a build folder: `mkdir build`
3) Navigate to the build folder: `cd build`
4) Execute cmake: `cmake ..`
5) Compile with make: `make`

## Example
The [examples](examples) folder contains examples on how to use the library, but we strongly suggest to refer to the [documentation](https://vis4rob-lab.github.io/t_prm).
Here, we report the main instructions to run T-PRM with one dynamic obstacle moving in the space:
1. Necessary includes: 
    ```
    #include <tprm/obstacle_impl.h>
    #include <tprm/temporal_prm.h>
    ```
2. Set the (holonomic) robot speed to a desired value:
    ```
    tprm::HolonomicRobot::movement_speed = 0.1;  // m/s
    ```
3. Create the main T-PRM object:
    ```
    tprm::TemporalPRM tprm;
    ```
4. Add a dynamic obstacle. In this case, the obstacle with dimension `0.25 m` is spawned at `(1, 1, 1)` and moves towards `(0, 0, 0)` with a velocity vector `(-0.1, -0.1, -0.1)`:
    ```
    tprm.addDynamicObstacle(std::make_shared<tprm::DynamicSphereObstacle>(tprm::Vector3d::Constant(1.), tprm::Vector3d::Constant(-0.1), 0.25));
    ```
5. Sample the space (`150` samples) and create the roadmap with a maximum edge length of `0.25 m`:
    ```
    tprm.placeSamples(150);
    tprm.buildPRM(0.25);
    ```
6. Query the roadmap for a path from `(0, 0, 0)` to `(1, 1, 1)` starting at time `0.5 s`:
    ```
    auto path = tprm.getShortestPath(tprm::Vector3d(0, 0, 0), tprm::Vector3d(1., 1., 1.), 0.5);
    ```
7. Print the result:
    ```
    if (path.empty()) {
        std::cout << "No path found" << std::endl;
    } else {
        for (auto& node : path)
            std::cout << "Node: " << node.position.transpose() << " at time " << node.time << std::endl;
    }
    ```
    
## Running Benchmarks
In order to run the benchmarks, check out the [instructions](benchmarking) in the `benchmarking` folder. To generate movies out of the benchmarks, check the folder [benchmarking/scripts](benchmarking/scripts).
