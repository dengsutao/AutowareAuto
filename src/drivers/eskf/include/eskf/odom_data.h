//
// Created by meng on 2021/2/19.
//

#ifndef ESKF_ODOM_DATA_H
#define ESKF_ODOM_DATA_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class ODOMData{
public:
    ODOMData() = default;

    double time = 0.0;
    Eigen::Vector3d pose = Eigen::Vector3d::Zero(); // in world, enu frame
    Eigen::Vector3d vel = Eigen::Vector3d::Zero(); // self frame, enu frame
};

#endif //ESKF_ODOM_DATA_H
