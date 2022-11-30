//
// Created by meng on 2021/2/19.
//

#ifndef ESKF_GPS_DATA_H
#define ESKF_GPS_DATA_H

#include "GeographicLib/LocalCartesian.hpp"
#include <eigen3/Eigen/Core>

class GPSData{
public:
    GPSData() = default;

    double time = 0.0;
    Eigen::Vector3d position_lla = Eigen::Vector3d::Zero(); // WGS-84 frame
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero(); // enu frame
    Eigen::Vector3d position_enu = Eigen::Vector3d::Zero(); // enu frame
};

#endif //ESKF_GPS_DATA_H
