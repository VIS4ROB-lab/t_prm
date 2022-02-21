#pragma once
#include <Eigen/Core>

namespace benchmarking {

struct Circle {
    Circle(const Eigen::Vector3d& _center, double _radius) : center(_center), radius(_radius) {}
    Eigen::Vector3d center;
    double radius;
};

struct MovingCircle {
    MovingCircle(const Eigen::Vector3d& _center, double _radius, Eigen::Vector3d _velocity) : center(_center), radius(_radius), velocity(_velocity) {}
    Eigen::Vector3d center;
    double radius;
    Eigen::Vector3d velocity;
};

};  // namespace benchmarking