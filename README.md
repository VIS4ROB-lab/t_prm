# T-PRM: Temporal Probabilistic Roadmap for Path Planning in Dynamic Environments 
This repository contains the source code for the T-PRM algorithm for path planning in dynamic environments.

> **Abstract**: Sampling-based motion planners are widely used in robotics due to their simplicity, flexibility and computational efficiency. However, in its most basic form, these algorithms operate under the assumption of static scenes and lack the ability to avoid collisions with dynamic (i.e. moving) obstacles. This raises safety concerns, limiting the range of possible applications of mobile robots in the real world. Motivated by these challenges, in this work we present Temporal Probabilistic Roadmap, a novel sampling-based path-planning algorithm that performs obstacle avoidance in dynamic environments. The proposed approach extends the original Probabilistic Roadmap (PRM) with the notion of time, generating an augmented graphlike structure that can be efficiently queried using a timeaware variant of the A* search algorithm, also introduced in this paper. Our design maintains all the properties of PRM, such as the ability to perform multiple queries and to find smooth paths, while being able to avoid collisions in highly dynamic scenes at the expense of minor computational overheads. Through a series of challenging experiments in highly cluttered and dynamic environments, we demonstrate that the proposed path planner outperforms other state-ofthe- art sampling-based solvers. Moreover, we show that our algorithm can run onboard a flying robot, performing obstacle avoidance in real time.

## Compiling
To compile, execute the following steps:
1) Clone this repository
2) Create a build folder (`mkdir build`)
3) Change to the build folder (`cd build`)
4) Execute cmake (`cmake ..`)
5) Execute make (`make`)

## Running Benchmarks
In order to run the benchmarks, check out the instructions in the `benchmarking` folder.

## Example
Check out the examples folder for examples of how to use this library.