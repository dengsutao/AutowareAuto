//
// Created by meng on 2021/2/19.
//

#ifndef ESKF_IMU_DATA_H
#define ESKF_IMU_DATA_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

class IMUData{
public:
    IMUData() = default;

    double time = 0.0;
    Eigen::Vector3d linear_accel = Eigen::Vector3d::Zero(); // enu frame
    Eigen::Vector3d angle_velocity = Eigen::Vector3d::Zero(); // enu frame
    Eigen::Vector3d angle = Eigen::Vector3d::Zero(); // enu frame
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
};

#endif //ESKF_IMU_DATA_H
