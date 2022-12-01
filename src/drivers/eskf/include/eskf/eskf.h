//
// Created by meng on 2021/2/19.
//

#ifndef ESKF_DATA_H
#define ESKF_DATA_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "../3rd/sophus/se3.hpp"
#include <deque>
#include <iostream>
#include <fstream>
#include <vector>

#include "imu_data.h"
#include "gps_data.h"
#include "odom_data.h"

using namespace std;

const unsigned int DIM_STATE = 15;
const unsigned int DIM_STATE_NOISE = 6;
const unsigned int DIM_MEASUREMENT = 3;
const unsigned int DIM_MEASUREMENT_NOISE = 3;

const unsigned int INDEX_STATE_POSI = 0;
const unsigned int INDEX_STATE_VEL = 3;
const unsigned int INDEX_STATE_ORI = 6;
const unsigned int INDEX_STATE_GYRO_BIAS = 9;
const unsigned int INDEX_STATE_ACC_BIAS = 12;
const unsigned int INDEX_MEASUREMENT_POSI = 0;

typedef typename Eigen::Matrix<double, DIM_STATE, 1> TypeVectorX;
typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, 1> TypeVectorY;
typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixF;
typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> TypeMatrixB;
typedef typename Eigen::Matrix<double, DIM_STATE_NOISE, DIM_STATE_NOISE> TypeMatrixQ;
typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixP;
typedef typename Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT> TypeMatrixK;
typedef typename Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_MEASUREMENT_NOISE> TypeMatrixC;
typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE> TypeMatrixG;
typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT> TypeMatrixR;

const unsigned int FLAG_RECV_ODOM = 0;
const unsigned int FLAG_RECV_IMU = 1;
const unsigned int FLAG_RECV_GPS = 2;
const unsigned int FLAG_VALID_ODOM = 3;
const unsigned int FLAG_VALID_IMU = 4;
const unsigned int FLAG_VALID_GPS = 5;
const unsigned int FLAG_INIT_DATA = 6;
const unsigned int ALL_INIT_FLAG = 0b1111111;
const unsigned int ALL_RECV_FLAG = 0b111;

class eskf: public rclcpp::Node
{
private:
    TypeVectorX X_;
    TypeVectorY Y_;
    TypeMatrixF F_;
    TypeMatrixB B_;
    TypeMatrixQ Q_;
    TypeMatrixP P_;
    TypeMatrixK K_;
    TypeMatrixC C_;
    TypeMatrixG G_;
    TypeMatrixC R_;
    TypeMatrixG Go_;
    TypeMatrixC Ro_;

    TypeMatrixF Ft_;

    Eigen::Vector3d init_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d init_angle_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d last_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();
    Eigen::Vector3d g_;//重力加速度
    Eigen::Vector3d w_;//地球自传角速度

    static GeographicLib::LocalCartesian geo_converter_;

    GPSData curr_gps_data_;
    GPSData init_gps_data_;
    IMUData curr_imu_data_;
    ODOMData curr_odom_data_;

    std::string save_dir;
    std::string save_file_name;
    std::string record_in_file_path;
    fstream fout;
    int mode;
    vector<GPSData> record_in_data_buff_;

    std::thread monitor_thread, keyboard_thread;
    bool thread_active;

    double gravity = 9.79484197226504;
    double earth_rotation_speed = 7.272205216e-05;
    
    double cov_prior_posi = 1.0;
    double cov_prior_vel = 1.0;
    double cov_prior_ori = 1.0;
    double cov_prior_epsilon = 1.0;
    double cov_prior_delta = 1.0;

    double cov_process_gyro = 1.0;
    double cov_process_accel = 1.0;

    double cov_measurement_posi_gps = 1.0e-3;
    double cov_measurement_posi_odom = 1.0e-2;

    double latitude_ = 0.0; //纬度

    size_t max_queue_length = 10;

    rclcpp::Time init_stamp;
    rclcpp::Time cur_stamp;

private:
    void SetCovarianceQ(double gyro_noise_cov, double accel_noise_cov);

    void SetCovarianceR(double posi_noise_cov);

    void SetCovarianceRo(double posi_noise_cov);

    void SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
                        double gyro_noise, double accel_noise);

    /*!
     * 通过IMU计算位姿和速度
     * @return
     */
    bool UpdateOdomEstimation();

    bool UpdateErrorState();

    bool ComputeAngularDelta(Eigen::Vector3d &angular_delta);

    /*!
     * 计算地球转动给导航系带来的变换
     * @param R_nm_nm_1
     * @return
     */
    bool ComputeEarthTranform(Eigen::Matrix3d &R_nm_nm_1);

    /*!
     * 通过IMU计算当前姿态
     * @param angular_delta
     * @param R_nm_nm_1
     * @param curr_R
     * @param last_R
     * @return
     */
    bool ComputeOrientation(const Eigen::Vector3d &angular_delta,
                            const Eigen::Matrix3d R_nm_nm_1,
                            Eigen::Matrix3d &curr_R,
                            Eigen::Matrix3d &last_R);

    bool ComputeVelocity(Eigen::Vector3d &curr_vel,
                         Eigen::Vector3d &last_vel,
                         const Eigen::Matrix3d &curr_R,
                         const Eigen::Matrix3d last_R);

    Eigen::Vector3d GetUnbiasAccel(const Eigen::Vector3d &accel);

    /*!
     * 通过imu计算当前位移
     * @param curr_vel
     * @param last_vel
     * @return
     */
    bool ComputePosition(const Eigen::Vector3d& curr_vel, const Eigen::Vector3d& last_vel);

    /*!
     * 对误差进行滤波之后，需要在实际算出来的轨迹中，消除这部分误差
     */
    void EliminateError();

    /*!
     * 每次矫正之后，需要重置状态变量X
     */
    void ResetState();

    Eigen::Vector3d correct_eular_angle(Eigen::Vector3d eular_angle);

public:
    eskf(std::string name);
    ~eskf();
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    int init_flag_; // binary state, 0b1111, 从左到右分别对应数据初始化/gps接收/imu接收/odom接收
    static Eigen::Vector3d LLA2ENU(const Eigen::Vector3d& lla);

    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr reset_odom_pub;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pose_pub1;
    //rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_pose_pub2;

    bool predict();
    bool odom_correct();
    bool gps_correct();
    bool record();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
    void thread_twist_pub();

    //终端控制程序
    void keyboard_controll();
    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    GPSData convert_data(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);
    IMUData convert_data(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    ODOMData convert_data(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    std::deque<GPSData> gps_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<ODOMData> odom_data_buff_;
};


#endif //ESKF_DATA_H
