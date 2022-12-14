#include "eskf.h"
#include <iomanip>
#include <thread>
#include <unistd.h>
#include <termios.h>
#include <sys/poll.h>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_SPACE 0x20
#define KEYCODE_ESC 0x1b


using namespace std;

constexpr double kDegree2Radian = M_PI / 180.0;

GeographicLib::LocalCartesian eskf::geo_converter_{32.0, 120.0, 0.0};


struct termios cooked, raw;
/** 
 *  === struct termios ===
 *  tcflag_t c_iflag;  输入模式
 *　tcflag_t c_oflag;  输出模式 
*　tcflag_t c_cflag;  控制模式 
*  tcflag_t c_lflag;  本地模式
*  cc_t c_cc[NCCS];   控制字符 
*/
int kfd=0;

Eigen::Matrix3d BuildSkewMatrix(const Eigen::Vector3d& vec){
    Eigen::Matrix3d matrix;
    matrix << 0.0,     -vec[2],   vec[1],
              vec[2],    0.0,     -vec[0],
              -vec[1],   vec[0],    0.0;

    return matrix;
}

Eigen::Vector3d eskf::LLA2ENU(const Eigen::Vector3d &lla) {
    Eigen::Vector3d enu;
    eskf::geo_converter_.Forward(lla[0], lla[1], lla[2], enu[0], enu[1], enu[2]);
    return enu;
}

void eskf::SetCovarianceQ(double gyro_noise, double accel_noise) {
    Q_.setZero();
    Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * gyro_noise * gyro_noise;
    Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * accel_noise * accel_noise;
}

void eskf::SetCovarianceR(double posi_noise) {
    R_.setZero();
    R_ = Eigen::Matrix3d::Identity() * posi_noise * posi_noise;
}

void eskf::SetCovarianceRo(double posi_noise) {
    Ro_.setZero();
    Ro_ = Eigen::Matrix3d::Identity() * posi_noise * posi_noise;
}

void eskf::SetCovarianceP(double posi_noise, double velo_noise, double ori_noise,
                          double gyro_noise, double accel_noise) {
    P_.setZero();
    P_.block<3,3>(INDEX_STATE_POSI, INDEX_STATE_POSI) = Eigen::Matrix3d::Identity() * posi_noise;
    P_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity() * velo_noise;
    P_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = Eigen::Matrix3d::Identity() * ori_noise; 
    P_.block<3,3>(INDEX_STATE_GYRO_BIAS, INDEX_STATE_GYRO_BIAS) = Eigen::Matrix3d::Identity() * gyro_noise;
    P_.block<3,3>(INDEX_STATE_ACC_BIAS, INDEX_STATE_ACC_BIAS) = Eigen::Matrix3d::Identity() * accel_noise;
}

GPSData eskf::convert_data(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
{
    // gps msg 读数坐标系：WGS-84,经纬度，转ENU
    GPSData gps_data;
    gps_data.time = (double)gps_msg->header.stamp.sec + (double)gps_msg->header.stamp.nanosec * 1.0e-9;

    gps_data.position_lla.x() = gps_msg->latitude;
    gps_data.position_lla.y() = gps_msg->longitude;
    gps_data.position_lla.z() = gps_msg->altitude;

    gps_data.position_enu = LLA2ENU(gps_data.position_lla);
    return gps_data;

}

IMUData eskf::convert_data(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    IMUData imu_data;
    imu_data.time = (double)imu_msg->header.stamp.sec + (double)imu_msg->header.stamp.nanosec * 1.0e-9;

    imu_data.linear_accel.x() = imu_msg->linear_acceleration.x*gravity;
    imu_data.linear_accel.y() = imu_msg->linear_acceleration.y*gravity;
    imu_data.linear_accel.z() = imu_msg->linear_acceleration.z*gravity;

    imu_data.angle_velocity.x() = imu_msg->angular_velocity.x;
    imu_data.angle_velocity.y() = imu_msg->angular_velocity.y;
    imu_data.angle_velocity.z() = imu_msg->angular_velocity.z;

    imu_data.quat = Eigen::Quaterniond(imu_msg->orientation.w, 
                                        imu_msg->orientation.x, 
                                        imu_msg->orientation.y, 
                                        imu_msg->orientation.z);
    // Eigen::Quaterniond wsu2enu_quat = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())*
    //                                 Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())*
    //                                 Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    // imu_data.quat = imu_data.quat * wsu2enu_quat;
    imu_data.angle = quat2eular(imu_data.quat);

    return imu_data;

}

ODOMData eskf::convert_data(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{   
    // ODOM坐标系nwu,角速度对应z轴朝上。
    // 需要转到enu
    ODOMData odom_data;
    odom_data.time = (double)odom_msg->header.stamp.sec + (double)odom_msg->header.stamp.nanosec * 1.0e-9;

    odom_data.pose.x() = -odom_msg->pose.pose.position.y; // x方向坐标
    odom_data.pose.y() = odom_msg->pose.pose.position.x; // y方向坐标
    odom_data.pose.z() = odom_msg->pose.pose.position.z / kDegree2Radian; // 角度值

    odom_data.vel.x() = -odom_msg->twist.twist.linear.y; // x方向速度
    odom_data.vel.y() = odom_msg->twist.twist.linear.x; // y方向速度
    odom_data.vel.z() = odom_msg->twist.twist.angular.z / kDegree2Radian; // 角速度，顺时针为负，逆时针为正，转到enu下不变

    // odom的z轴角度只是相对值，需要加上初始的imu yaw角。注意保证发送odom数据在初始化imu数据之后！！
    double eular_angle_z = init_angle_.z();
    odom_data.pose.z() += eular_angle_z / kDegree2Radian; // 后边暂时用不到
    odom_data.pose.z() = fmod(odom_data.pose.z()+180.0,360.0) - 180.0;
    // RCLCPP_INFO(get_logger(), "before\nodom_data.pose_x:"+std::to_string(odom_data.pose.x())+
    //                             "\nodom_data.pose_y:"+std::to_string(odom_data.pose.y())+
    //                             "\nodom_data.pose_z:"+std::to_string(odom_data.pose.z()));
    // 由于odom只能计算enu坐标系下的相对位移
    // 默认初始yaw角为0，这里要加上初始yaw角的变换，才是odom在大地enu坐标系下的位置和速度
    odom_data.pose.x()=odom_data.pose.x() * cos(eular_angle_z) - odom_data.pose.y() * sin(eular_angle_z); 
    odom_data.pose.y()=odom_data.pose.x() * sin(eular_angle_z) + odom_data.pose.y() * cos(eular_angle_z); 
    odom_data.vel.x()=odom_data.vel.x() * cos(eular_angle_z) - odom_data.vel.y() * sin(eular_angle_z); 
    odom_data.vel.y()=odom_data.vel.x() * sin(eular_angle_z) + odom_data.vel.y() * cos(eular_angle_z); 
    // RCLCPP_INFO(get_logger(), "after\nodom_data.pose_x:"+std::to_string(odom_data.pose.x())+
    //                             "\nodom_data.pose_y:"+std::to_string(odom_data.pose.y())+
    //                             "\nodom_data.pose_z:"+std::to_string(odom_data.pose.z()));
    return odom_data;
}

/*** 第一次接收只初始化起点的lla坐标，FLAG_RECV_GPS=1
 * 数据初始化完毕后，FLAG_INIT_DATA=1时，开始记录gps数据,缓存10条
 * 如果存在异常，FLAG_VALID_GPS=1，gps数据不可用
*/
void eskf::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
{
    if (((init_flag_>>FLAG_RECV_GPS) & 1) == 0) // gps第一次开始接收数据
    {
        // 初始化相对坐标的0点，用于将lla转化为enu坐标系，得到gps的position_enu
        eskf::geo_converter_ = GeographicLib::LocalCartesian{gps_msg->latitude, gps_msg->longitude, gps_msg->altitude};
        // init_gps_data_ = convert_data(gps_msg);
        // curr_gps_data_ = init_gps_data_;
        // gps_data_buff_.push_back(curr_gps_data_);
        init_flag_+=1<<FLAG_RECV_GPS;
        RCLCPP_INFO(get_logger(), "start receive gps data");
        return;
    }
    if (((init_flag_>>FLAG_INIT_DATA)&1)==0) return;
    curr_gps_data_ = convert_data(gps_msg);
    gps_data_buff_.push_back(curr_gps_data_);
    // 需要判断gps数据是否存在异常？
    // gps信号异常的情况：
    // 1.全为0，没有收到信号
    // 2.长时间固定不动，信号不佳场景发生；如果确实静止，信号良好情况下也会有波动
    // 3.线性增长？路径几何为一条直线？
    while (gps_data_buff_.size()>max_queue_length) gps_data_buff_.pop_front();
    Eigen::Vector3d tmp;
    bool invalid = false;
    bool flag_all_same = false;
    // 只在数据积累到一定长度之后开始检查，用于检查长期异常而非短期
    if (gps_data_buff_.size()==max_queue_length)
    {
        // 检查异常
        // 是否存在长期数据完全相同的情况。case1,2
        for (size_t i=0;i<gps_data_buff_.size();i++)
        {
            if (i==0)
            {
                tmp = gps_data_buff_.at(i).position_lla;
            }
            else
            {
                Eigen::Vector3d cur = gps_data_buff_.at(i).position_lla;
                if (cur!=tmp) break;
                if (i==gps_data_buff_.size()-1)
                {
                    flag_all_same = true;
                }
            }
        }
        // case 3, to do
    }
    if (flag_all_same) invalid = true;
    if (gps_data_buff_.at(gps_data_buff_.size()-1).position_lla==Eigen::Vector3d::Zero()) invalid = true;

    if (invalid && ((init_flag_>>FLAG_VALID_GPS)&1)) init_flag_ -= 1<<FLAG_VALID_GPS;
    else if ((!invalid) && (((init_flag_>>FLAG_VALID_GPS)&1)==0)) init_flag_ += 1<<FLAG_VALID_GPS;

}

/*** 第一次接收,FLAG_RECV_IMU=1,FLAG_VALID_IMU=1，暂时不做valid判定
 * 所有数据都接收后，初始化位置为0，姿态为imu对应读数，FLAG_INIT_DATA=1
 * 保持缓存数据长度为2(只有前后两帧)，发布话题重置轮速记位置
 * 此后做预测，有gps，odom数据的话通过eskf更新
 * 非复现模式下(mode!=2)，录制数据
*/
void eskf::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    if (((init_flag_>>FLAG_RECV_IMU) & 1) == 0) // imu第一次开始接收数据
    {
        init_flag_+=1<<FLAG_RECV_IMU;
        init_flag_+=1<<FLAG_VALID_IMU;
        RCLCPP_INFO(get_logger(), "start receive imu data");
        return;
    }
    curr_imu_data_ = convert_data(imu_msg);
    imu_data_buff_.push_back(curr_imu_data_);

    // RCLCPP_INFO(get_logger(), "\ncur_angle_x:"+std::to_string(curr_imu_data_.angle.x()/M_PI*180)+
    //                             "\ncur_angle_y:"+std::to_string(curr_imu_data_.angle.y()/M_PI*180)+
    //                             "\ncur_angle_z:"+std::to_string(curr_imu_data_.angle.z()/M_PI*180));

    // 不能单纯依靠imu进行路径计算，目前暂定3种传感器都接受到数据后，可以开始录制数据。
    // 但不要求gps数据和odom数据都valid。
    if ((init_flag_ & ALL_RECV_FLAG)!=ALL_RECV_FLAG) return;

    // 三种传感器数据接收ready，初始化位置/姿态数据
    if (((init_flag_>>FLAG_INIT_DATA)&1)==0)
    {
        init_angle_ = curr_imu_data_.angle;

        // 设定初始姿态，通过angle文件第一条数据读取
        std::cout<<"initial velocity:"<<init_velocity_.transpose()<<std::endl;
        std::cout<<"initial angle:"<<init_angle_.transpose()/kDegree2Radian<<std::endl;
        RCLCPP_INFO(get_logger(), "init_angle_z:"+std::to_string(init_angle_.z()/M_PI*180));
        Eigen::Quaterniond Q = Eigen::AngleAxisd(init_angle_[2], Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(init_angle_[1], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(init_angle_[0], Eigen::Vector3d::UnitX());
        init_pose_.block<3,3>(0,0) = Q.toRotationMatrix();
        pose_ = init_pose_;
        last_pose_ = init_pose_;
        std::cout<<"init pose:\n"<<pose_<<std::endl;

        // 清空之前累积的数据，保证下一条数据进来后，buff长度为2
        while (imu_data_buff_.size()>1) imu_data_buff_.pop_front();
        // 表征数据位姿初始化完成
        init_flag_+=1<<FLAG_INIT_DATA;
        std_msgs::msg::UInt32 reset_odom_value;
        reset_odom_value.data = 1;
        reset_odom_pub->publish(reset_odom_value);

        // 记录起始时间
        init_stamp = imu_msg->header.stamp;
        return;

    }
    
    cur_stamp = imu_msg->header.stamp;
    // 处理imu，首先只依赖imu数据进行路径预测，然后查看gps_data_buff_,odom_data_buff_,
    // 如果有缓存，取最后一条更新。
    predict();
    if (get_parameter("debug").as_bool()) std::cout<<"imu:"<<pose_.block<3,1>(0,3).transpose()<<std::endl;
    if (((init_flag_>>FLAG_VALID_ODOM) & 1) == 1) odom_correct();
    else if (get_parameter("debug").as_bool()) std::cout<<"odom invalid"<<std::endl;
    if (((init_flag_>>FLAG_VALID_GPS) & 1) == 1) gps_correct();
    else if (get_parameter("debug").as_bool()) std::cout<<"gps invalid"<<std::endl;
    // save time and pose
    if (mode!=2) record();
    // 更新上一帧pose
    last_pose_ = pose_;
    // pop
    imu_data_buff_.pop_front();
}

/*** 第一次接收,FLAG_RECV_ODOM=1,FLAG_VALID_ODOM=1，暂时不做valid判定
 * 只缓存10条数据
 * 轮速计静止时，减小其协方差矩阵，避免imu累积误差
*/
void eskf::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    if (((init_flag_>>FLAG_RECV_ODOM) & 1) == 0) // odom第一次开始接收数据
    {
        init_flag_+=1<<FLAG_RECV_ODOM;
        init_flag_+=1<<FLAG_VALID_ODOM;
        RCLCPP_INFO(get_logger(), "start receive odom data");
        return;
    }
    if (((init_flag_>>FLAG_INIT_DATA)&1)==0) return;
    curr_odom_data_ = convert_data(odom_msg);
    odom_data_buff_.push_back(curr_odom_data_);
    while (odom_data_buff_.size()>max_queue_length) odom_data_buff_.pop_front();
    // 需要判断odom数据是否存在异常？以此给定odom的协方差矩阵
    // 轮速计速度为0时，认为静止，减小轮速计协方差，增加轮速计在路径计算的权重，避免imu静止状态下的累积误差
    if (curr_odom_data_.vel.x()==0 && curr_odom_data_.vel.y()==0 && curr_odom_data_.vel.z()==0) SetCovarianceRo(1.0e-9);
    else
    {
        get_parameter("cov_measurement_posi_odom", cov_measurement_posi_odom);
        SetCovarianceRo(cov_measurement_posi_odom);
    }
    
}

// 理论上imu_data_buff_长度为2，记录前一时刻数据和当前时刻数据
// 除初始化外，每次订阅到数据都预测一次。
bool eskf::predict()
{
    if (imu_data_buff_.size()!=2) 
    {
        RCLCPP_INFO(get_logger(), "imu_data_buff_.size()!=2");
        return false;
    }
    UpdateOdomEstimation();
    UpdateErrorState();
    return false;
}

bool eskf::UpdateOdomEstimation() {
    Eigen::Vector3d angular_delta;
    ComputeAngularDelta(angular_delta);

    Eigen::Matrix3d R_nm_nm_1;
    ComputeEarthTranform(R_nm_nm_1);

    Eigen::Matrix3d curr_R, last_R;
    ComputeOrientation(angular_delta, R_nm_nm_1, curr_R, last_R);

    Eigen::Vector3d curr_vel, last_vel;
    ComputeVelocity(curr_vel, last_vel, curr_R, last_R);

    ComputePosition(curr_vel, last_vel);

    return true;
}

bool eskf::ComputeAngularDelta(Eigen::Vector3d &angular_delta) {
    IMUData curr_imu_data = imu_data_buff_.at(1);
    IMUData last_imu_data = imu_data_buff_.at(0);

    double delta_t = curr_imu_data.time - last_imu_data.time;

    if (delta_t <= 0){
        angular_delta = Eigen::Vector3d(0.0,0.0,0.0);
        return false;
    }

    Eigen::Vector3d curr_angular_vel = curr_imu_data.angle_velocity;

    Eigen::Vector3d last_angular_vel = last_imu_data.angle_velocity;

    Eigen::Vector3d curr_unbias_angular_vel = curr_angular_vel;
    Eigen::Vector3d last_unbias_angular_vel = last_angular_vel;

    angular_delta = 0.5 * (curr_unbias_angular_vel + last_unbias_angular_vel) * delta_t;
    // std::cout<<"curr_unbias_angular_vel, angular_delta"<<curr_unbias_angular_vel.transpose()<<","<<angular_delta.transpose()<<std::endl;
    return true;
}

bool eskf::ComputeEarthTranform(Eigen::Matrix3d &R_nm_nm_1) {
    IMUData curr_imu_data = imu_data_buff_.at(1);
    IMUData last_imu_data = imu_data_buff_.at(0);

    double delta_t = curr_imu_data.time - last_imu_data.time;

    constexpr double rm = 6353346.18315;
    constexpr double rn = 6384140.52699;
    Eigen::Vector3d w_en_n(-velocity_[1] / (rm + curr_gps_data_.position_lla[2]),
                           velocity_[0] / (rn + curr_gps_data_.position_lla[2]),
                           velocity_[0] / (rn + curr_gps_data_.position_lla[2])
                           * std::tan(curr_gps_data_.position_lla[0] * kDegree2Radian));

    Eigen::Vector3d w_in_n = w_en_n + w_;

    auto angular = delta_t * w_in_n;

    Eigen::AngleAxisd angle_axisd(angular.norm(), angular.normalized());

    R_nm_nm_1 = angle_axisd.toRotationMatrix().transpose();
    return true;
}

bool eskf::ComputeOrientation(const Eigen::Vector3d &angular_delta,
                              const Eigen::Matrix3d R_nm_nm_1,
                              Eigen::Matrix3d &curr_R,
                              Eigen::Matrix3d &last_R) {
    Eigen::AngleAxisd angle_axisd(angular_delta.norm(), angular_delta.normalized());
    last_R = pose_.block<3, 3>(0, 0);

    curr_R = R_nm_nm_1 * pose_.block<3, 3>(0, 0) * angle_axisd.toRotationMatrix();

    pose_.block<3, 3>(0, 0) = curr_R;
    return true;
}

bool eskf::ComputeVelocity(Eigen::Vector3d &curr_vel, Eigen::Vector3d& last_vel,
                                             const Eigen::Matrix3d &curr_R,
                                             const Eigen::Matrix3d last_R) {
    IMUData curr_imu_data = imu_data_buff_.at(1);
    IMUData last_imu_data = imu_data_buff_.at(0);
    double delta_t = curr_imu_data.time - last_imu_data.time;
    if (delta_t <=0 ){
        return false;
    }

    Eigen::Vector3d curr_accel = curr_imu_data.linear_accel;
    Eigen::Vector3d curr_unbias_accel = GetUnbiasAccel(curr_R * curr_accel);

    Eigen::Vector3d last_accel = last_imu_data.linear_accel;
    Eigen::Vector3d last_unbias_accel = GetUnbiasAccel(last_R * last_accel);

    last_vel = velocity_;

    velocity_ += delta_t * 0.5 * (curr_unbias_accel + last_unbias_accel);
    curr_vel = velocity_;

    // std::cout<<"cur acc,velo:"<<curr_unbias_accel.transpose()<<","<<curr_vel.transpose()<<std::endl;

    return true;
}

Eigen::Vector3d eskf::GetUnbiasAccel(const Eigen::Vector3d &accel) {
//    return accel - accel_bias_ + g_;
    return accel - g_;
}

bool eskf::ComputePosition(const Eigen::Vector3d& curr_vel, const Eigen::Vector3d& last_vel){
    double delta_t = imu_data_buff_.at(1).time - imu_data_buff_.at(0).time;

    pose_.block<3,1>(0,3) += 0.5 * delta_t * (curr_vel + last_vel);

    return true;
}

bool eskf::UpdateErrorState() {
    IMUData curr_imu_data = imu_data_buff_.at(1);
    IMUData last_imu_data = imu_data_buff_.at(0);
    double delta_t = curr_imu_data.time - last_imu_data.time;

    Eigen::Vector3d accel = pose_.block<3, 3>(0, 0)
                            * curr_imu_data.linear_accel;

    Eigen::Matrix3d F_23 = BuildSkewMatrix(accel);

    F_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_ORI) = F_23;
    F_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = pose_.block<3,3>(0,0);
    F_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_GYRO_BIAS) = -pose_.block<3,3>(0,0);
    B_.block<3,3>(INDEX_STATE_VEL, 3) = pose_.block<3,3>(0,0);
    B_.block<3,3>(INDEX_STATE_ORI, 0) = -pose_.block<3,3>(0,0);

    TypeMatrixF Fk = TypeMatrixF::Identity() + F_ * delta_t;
    TypeMatrixB Bk = B_ * delta_t;

    Ft_ = F_ * delta_t;

    X_ = Fk * X_;
    P_ = Fk * P_ * Fk.transpose() + Bk * Q_ * Bk.transpose();

    return true;
}

// 取odom_data_buff_里最后一条数据做更新
bool eskf::odom_correct()
{
    if (odom_data_buff_.size()==0) return true;
    curr_odom_data_ = odom_data_buff_.back();
    if (get_parameter("debug").as_bool()) 
    std::cout<<"odom:"<<curr_odom_data_.pose.transpose()<<std::endl;

    // 由于轮速计无法得到z方向的高度信息，这里暂时用0替代，imu在z方向累积误差也比较大。
    Eigen::Vector3d odom_pose = Eigen::Vector3d(curr_odom_data_.pose[0], curr_odom_data_.pose[1], 0.0);

    Y_ = pose_.block<3,1>(0,3) - odom_pose;

    K_ = P_ * Go_.transpose() * (Go_ * P_ * Go_.transpose() + C_ * Ro_ * C_.transpose()).inverse();

    P_ = (TypeMatrixP::Identity() - K_ * Go_) * P_;
    X_ = X_ + K_ * (Y_ - Go_ * X_);

    EliminateError();

    ResetState();

    return true;
}

bool eskf::gps_correct()
{
    if (gps_data_buff_.size()==0) return true;
    curr_gps_data_ = gps_data_buff_.back();
    if (get_parameter("debug").as_bool())
    std::cout<<"gps:"<<curr_gps_data_.position_enu.transpose()<<std::endl;

    Y_ = pose_.block<3,1>(0,3) - curr_gps_data_.position_enu;

    K_ = P_ * Go_.transpose() * (Go_ * P_ * Go_.transpose() + C_ * Ro_ * C_.transpose()).inverse();

    P_ = (TypeMatrixP::Identity() - K_ * Go_) * P_;
    X_ = X_ + K_ * (Y_ - Go_ * X_);

    EliminateError();

    ResetState();

    return true;
}

void eskf::ResetState() {
    X_.setZero();
}

void eskf::EliminateError() {
    pose_.block<3,1>(0,3) = pose_.block<3,1>(0,3) - X_.block<3,1>(INDEX_STATE_POSI, 0);

    velocity_ = velocity_ - X_.block<3,1>(INDEX_STATE_VEL, 0);
    Eigen::Matrix3d C_nn = Sophus::SO3d::exp(X_.block<3,1>(INDEX_STATE_ORI, 0)).matrix();
    pose_.block<3,3>(0,0) = C_nn * pose_.block<3,3>(0,0);

    // gyro_bias_ = gyro_bias_ - X_.block<3,1>(INDEX_STATE_GYRO_BIAS, 0);
    // accel_bias_ = accel_bias_ - X_.block<3,1>(INDEX_STATE_ACC_BIAS, 0);
}

bool eskf::record()
{
    auto curr_imu_data = imu_data_buff_.back();
    auto last_imu_data = imu_data_buff_.front();
    auto curr_gps_data = gps_data_buff_.back();
    auto curr_odom_data = odom_data_buff_.back();
    double curr_time = curr_imu_data.time;
    double last_time = last_imu_data.time;
    double delta_t = curr_time - last_time;
    double curr_e = pose_(0,3);
    double curr_n = pose_(1,3);
    double curr_u = pose_(2,3);

    double last_e = last_pose_(0,3);
    double last_n = last_pose_(1,3);
    double last_u = last_pose_(2,3);

    double velo_e = (curr_e - last_e) / delta_t;
    double velo_n = (curr_n - last_n) / delta_t;
    double velo_u = (curr_u - last_u) / delta_t;

    double curr_lat=0, curr_lon=0, curr_alt=0;
    eskf::geo_converter_.Reverse(curr_e, curr_n, curr_u, curr_lat, curr_lon, curr_alt);

    double curr_imu_acc_e = curr_imu_data.linear_accel.x();
    double curr_imu_acc_n = curr_imu_data.linear_accel.y();
    double curr_imu_acc_u = curr_imu_data.linear_accel.z();

    double curr_imu_av_e = curr_imu_data.angle_velocity.x();
    double curr_imu_av_n = curr_imu_data.angle_velocity.y();
    double curr_imu_av_u = curr_imu_data.angle_velocity.z();

    double curr_gps_e = curr_gps_data.position_enu.x();
    double curr_gps_n = curr_gps_data.position_enu.y();
    double curr_gps_u = curr_gps_data.position_enu.z();

    double curr_odom_e = curr_odom_data.pose.x();
    double curr_odom_n = curr_odom_data.pose.y();
    double curr_odom_u = curr_odom_data.pose.z();

    Eigen::Matrix3d mat = pose_.block<3,3>(0,0);
    Eigen::Quaterniond quat(mat);
    Eigen::Vector3d rpy = quat2eular(quat);

    fout<<std::setprecision(11)<<"time:"<<curr_time\
            <<","<<" enu_pose:"<<curr_e<<","<<curr_n<<","<<curr_u\
            <<","<<" lla_pose:"<<curr_lat<<","<<curr_lon<<","<<curr_alt\
            <<","<<" rpy:"<<rpy.x()<<","<<rpy.y()<<","<<rpy.z()\
            <<","<<" enu_velo:"<<velo_e<<","<<velo_n<<","<<velo_u\
            <<","<<" imu_acc:"<<curr_imu_acc_e<<","<<curr_imu_acc_n<<","<<curr_imu_acc_u\
            <<","<<" imu_av:"<<curr_imu_av_e<<","<<curr_imu_av_n<<","<<curr_imu_av_u\
            <<","<<" gps_enu_pose:"<<curr_gps_e<<","<<curr_gps_n<<","<<curr_gps_u\
            <<","<<" odom_enu_pose:"<<curr_odom_e<<","<<curr_odom_n<<","<<curr_odom_u\
            <<std::endl;
    if (get_parameter("debug").as_bool())
    std::cout<<std::setprecision(11)<<"time:"<<curr_time\
            <<","<<" enu_pose:"<<curr_e<<","<<curr_n<<","<<curr_u\
            <<","<<" lla_pose:"<<curr_lat<<","<<curr_lon<<","<<curr_alt\
            <<","<<" rpy:"<<rpy.x()<<","<<rpy.y()<<","<<rpy.z()\
            <<","<<" enu_velo:"<<velo_e<<","<<velo_n<<","<<velo_u\
            <<","<<" imu_acc:"<<curr_imu_acc_e<<","<<curr_imu_acc_n<<","<<curr_imu_acc_u\
            <<","<<" imu_av:"<<curr_imu_av_e<<","<<curr_imu_av_n<<","<<curr_imu_av_u\
            <<","<<" gps_enu_pose:"<<curr_gps_e<<","<<curr_gps_n<<","<<curr_gps_u\
            <<","<<" odom_enu_pose:"<<curr_odom_e<<","<<curr_odom_n<<","<<curr_odom_u\
            <<std::endl;
    nav_msgs::msg::Odometry fused_pose;
    fused_pose.header.frame_id = "base_link";
    fused_pose.child_frame_id = "odom";
    auto duration = cur_stamp - init_stamp;
    fused_pose.header.stamp = rclcpp::Time(0,0) + duration;
    // fused_pose.header.stamp = cur_stamp; 

    fused_pose.pose.pose.position.x = curr_e;
    fused_pose.pose.pose.position.y = curr_n;
    fused_pose.pose.pose.position.z = curr_u;
    // Eigen::Quaterniond quat(mat);
    fused_pose.pose.pose.orientation.w = 1.0;
    fused_pose.pose.pose.orientation.x = 0.0;
    fused_pose.pose.pose.orientation.y = 0.0;
    fused_pose.pose.pose.orientation.z = 0.0;
    fused_pose_pub->publish(fused_pose);

    nav_msgs::msg::Odometry fused_pose1 = fused_pose;
    fused_pose1.header.stamp = get_clock()->now();
    fused_pose_pub1->publish(fused_pose1);

    //fused_pose_pub2->publish(fused_pose1);
    return true;
}

Eigen::Vector3d eskf::quat2eular(Eigen::Quaterniond quat)
{
    auto angle_zyx = quat.matrix().eulerAngles(2,1,0); // z,y,x
    Eigen::Vector3d angle_xyz(angle_zyx[2],angle_zyx[1],angle_zyx[0]);
    Eigen::Vector3d out;
    if (abs(angle_xyz[1])>M_PI/2)
    {
        out.x() = angle_xyz.x()>0? (angle_xyz.x()-M_PI):(angle_xyz.x()+M_PI);
        out.y() = angle_xyz.y()>0? (M_PI-angle_xyz.y()):(-M_PI-angle_xyz.y());
        out.z() = angle_xyz.z()-M_PI;
    }
    else{
        out.x() = angle_xyz.x();
        out.y() = angle_xyz.y();
        out.z() = angle_xyz.z();
    }
    return out;
}

/***
 * 这个线程用于发布轮素计的目标速度，角速度信息，以复现目标路径
 * 复现算法：
 * 1.首先原地旋转，以调整小车当前初始方向为目标路径的初始速度方向
 * 2.小车只发出自身坐标系下x方向速度和yaw角的角速度，一般x速度固定，角速度视角度偏离情况调整
 * 3.小车以固定x方向速度行驶，实时获取位姿
 * 4.计算当前位置到目标路径的匹配点，匹配原则依据最短距离，计算当前点到匹配点的方向向量
 * 5.获取匹配点速度，结合4中的速度方向分量，加权得到目标速度
 * 6.计算目标速度与当前速度的夹角，叉乘判定旋转方向（顺、逆时针），根据夹角大小得到角速度值
 * 7.发布x方向速度(固定值)和角速度值
 * 注：这个版本的算法角速度会一直有值。
***/
void eskf::thread_twist_pub()
{
    while (((init_flag_>>FLAG_INIT_DATA) & 1)==0) sleep(1);
    sleep(2);
    // 超参数
    double min_match_dist = 1.0, VD0=0.1, VD1=0.5;
    double base_velo = 5; // rad/s

    // 初始化, 原地旋转以匹配目标路径初始速度
    //获取录制路线
    vector<GPSData> record_data_buff = record_in_data_buff_;
    // 目标初始点
    GPSData record_data_init = record_data_buff.at(0);
    // vi是初始点的速度方向向量
    Eigen::Vector2d vi = record_data_init.velocity.block<2,1>(0,0);
    std::cout<<"init vi:"<<vi.transpose()<<std::endl;
    // enu坐标系下，角度的0度是y轴正半轴方向，所以真正的角度值应为atan(-x/y),同时需要映射到[-pi,pi]
    double init_yaw;
    double init_x = -vi[0], init_y = vi[1];
    if (init_y==0) init_yaw = init_x>=0? M_PI/2:-M_PI/2;
    else 
    {
        double a = atan(init_x/init_y);
        init_yaw = init_y>=0? a:(a>0?a-M_PI:a+M_PI);
    }
    init_yaw = init_yaw / kDegree2Radian;
    bool flag_init = false;
    std::cout<<"initial rotation..."<<std::endl;
    while (!flag_init)
    {
        if (!thread_active) return; 
        //获取当前位姿
        // Eigen::Matrix4d curr_pose = pose_;
        // Eigen::Matrix3d Mr = curr_pose.block<3,3>(0,0);
        // Eigen::Vector3d cur_angle = Mr.eulerAngles(0,1,2); // [-pi,pi]
        // Eigen::Vector3d enu_angle = correct_eular_angle(cur_angle);

        // std::cout<<curr_imu_data_.angle.transpose()<<std::endl;
        Eigen::Vector3d enu_angle = curr_imu_data_.angle;
        double cur_yaw = enu_angle.z() / kDegree2Radian;
        std::cout<<enu_angle.transpose()<<std::endl;
        std::cout<<"cur/init yaw:"<<cur_yaw<<"/"<<init_yaw<<std::endl;
        // 计算夹角,角度值
        double angle = init_yaw - cur_yaw;
        double velo_theta = 0;

        // 这里可能存在初始化时需要旋转接近一圈的情况。根据正负值优化下即可。
        if (angle > 180) angle = -(360-angle);
        else if (angle < -180) angle = 360+angle;
        // 根据阈值赋予角速度
        if (angle > 10) velo_theta = base_velo;
        else if (angle < -10) velo_theta = -base_velo;
        else 
        {
            flag_init = true;
            geometry_msgs::msg::Twist tst;
            // tst.linear.x = get_parameter("target_twist_x").get_value<double>();;
            twist_pub->publish(tst);
            break;
        }
        geometry_msgs::msg::Twist tst;
        tst.angular.z = velo_theta;
        twist_pub->publish(tst);
        usleep(100); // miliseconds
    }
    std::cout<<"start monitor target path..."<<std::endl;
    size_t last_idx = 0;
    // double te,tn,tu;
    // Eigen::Vector3d record_init_pos = record_in_data_buff_.at(0).position_lla;
    // geo_converter_.Reverse(te,tn,tu,record_init_pos.x(), record_init_pos.y(), record_init_pos.z());
    // Eigen::Vector3d relative_trans(te,tn,tu); // 记录里初始坐标相对于当前初始坐标的位移
    while (true)
    {
        if (!thread_active) return; 

        //获取当前位姿
        Eigen::Matrix4d curr_pose = pose_;
        Eigen::Vector3d curr_velo = velocity_;
        Eigen::Vector3d curr_pos(curr_pose.block<3,1>(0,3)); 
        std::cout<<"cur pose:"<<curr_pos.transpose()<<std::endl;
        std::cout<<"cur velo:"<<velocity_.transpose()<<std::endl;

        //匹配,如何提效？设置上一次匹配的点标记，从该点附近开始查找，查找前后100个记录,10hz的话约为前后10秒的记录
        // int iter_start = max(last_idx-100,0);
        size_t iter_start = last_idx;
        size_t iterr_end = min(last_idx+10, record_data_buff.size());
        double match_dist=1e9;
        size_t match_idx=0;
        Eigen::Vector3d match_pos;
        for (size_t i=iter_start;i<iterr_end;i++)
        {
            // Eigen::Vector3d record_pos(record_data_buff.at(i).position_enu+relative_trans);
            Eigen::Vector3d record_pos(record_data_buff.at(i).position_enu);
            double dist = (curr_pos-record_pos).norm();
            if (match_dist>dist)
            {
                match_dist = dist;
                match_idx = i;
                match_pos = record_pos;
            }
        }
        std::cout<<"matched idx,dist:"<<match_idx<<","<<match_dist<<std::endl;
        std::cout<<"matched pose:"<<match_pos.transpose()<<std::endl;
        Eigen::Vector2d vd, vr, vf, vc;
        if (match_idx==record_data_buff.size()-1 && match_dist < min_match_dist)
        {
            // 发出停止指令
            geometry_msgs::msg::Twist tst;
            tst.angular.z = 0.0;
            tst.linear.x = 0.0;
            twist_pub->publish(tst);
            std::cout<<"finsh monitor!"<<std::endl;
            return;
        }
        else if (match_idx==record_data_buff.size()-1)
        {
            vd = match_pos.block<2,1>(0,0) - curr_pos.block<2,1>(0,0);
            vd.normalize();
            vd = vd*VD1;
            vr = Eigen::Vector2d::Zero();
        }
        else{
            // vd是当前位置到匹配点的速度方向向量，大小固定为0.1m/s
            vd = match_pos.block<2,1>(0,0) - curr_pos.block<2,1>(0,0);
            vd.normalize();
            vd = vd*VD0;
            // vr是匹配点的速度方向向量，大小固定为1m/s
            vr = record_data_buff.at(match_idx).velocity.block<2,1>(0,0);
            vr.normalize();
        }
        
        // vf是vd和vr的融合，小车既要保持匹配点的速度，也要有向原始路径靠近的速度分量
        // 另外应该加入平滑因子，小车应该尽量稳定行驶，避免过分抖动,这里通过固定小车x方向速度，并控制z旋转速度实现
        vf = vd+vr;
        // vc是小车在当前点的速度
        vc = curr_velo.block<2,1>(0,0);
        // 计算夹角,角度值
        double angle = acos(vf.dot(vc)/(vf.norm()*vc.norm())) / kDegree2Radian;
        // 计算旋转方向，顺时针还是逆时针,需要转化为三维向量
        int direct = 1; // true为逆时针，z轴朝上
        if (Eigen::Vector3d(vc(0),vc(1),0).cross(Eigen::Vector3d(vf(0),vf(1),0)).z()<0) direct = -1;
        // 小车旋转角速度和angle应该有一个映射关系，angle越大，角速度应该越大
        double velo_theta = 0;
        if (abs(angle)<90) velo_theta = pow(angle / 90, 2) * base_velo;
        else velo_theta = base_velo;
        velo_theta = direct * velo_theta;
        std::cout<<"target velo theta:"<<velo_theta<<std::endl;

        // publish target velocity
        geometry_msgs::msg::Twist tst;
        tst.angular.z = velo_theta;
        tst.linear.x = get_parameter("target_twist_x").get_value<double>();
        twist_pub->publish(tst);
        last_idx = match_idx;
        sleep(1);
    }
}

void eskf::keyboard_controll()
{
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    /**
	 * c_lflag : 本地模式标志，控制终端编辑功能
	 * ICANON: 使用标准输入模式
	 * ECHO: 显示输入字符
	 */
    raw.c_lflag &=~ (ICANON | ECHO);

	/** 
	 * c_cc[NCCS]：控制字符，用于保存终端驱动程序中的特殊字符，如输入结束符等
	 * VEOL: 附加的End-of-file字符
	 * VEOF: End-of-file字符
	 * */
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    /* *
	 * struct pollfd {
　　       int fd;        文件描述符 
　       　short events;  等待的事件 
　　       short revents; 实际发生了的事件 
　       　};
    */
    struct pollfd ufd;
    ufd.fd = kfd;
    ufd.events = POLLIN;

    char c;
    while (true)
    {
        /* get the next event from the keyboard */
        int num;
        
		/**
		 * poll:把当前的文件指针挂到设备内部定义的等待队列中。
		 * unsigned int (*poll)(struct file * fp, struct poll_table_struct * table)
		 */
        if ((num = poll(&ufd, 1, 250)) < 0)
        {
			/**
			 * perror( ) 用来将上一个函数发生错误的原因输出到标准设备(stderr)。
			 * 参数s所指的字符串会先打印出,后面再加上错误原因字符串。
			 * 此错误原因依照全局变量errno 的值来决定要输出的字符串。
			 * */
            perror("poll():");
            return;
        }
        else if(num > 0)
        {
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                return;
            }
        }
        geometry_msgs::msg::Twist tst;
        switch(c)
        {
            case KEYCODE_W:
            {
                thread_active = true;
                tst.linear.x = 0.5;
                twist_pub->publish(tst);
                break;
            }
            case KEYCODE_S:
            {
                thread_active = true;
                tst.linear.x = -0.5;
                twist_pub->publish(tst);
                break;
            }
            case KEYCODE_A:
            {
                thread_active = true;
                tst.angular.z = 20.0;
                twist_pub->publish(tst);
                break;
            }
            case KEYCODE_D:
            {
                thread_active = true;
                tst.angular.z = -20.0;
                twist_pub->publish(tst);
                break;
            }
                
            case KEYCODE_SPACE:
            {
                thread_active = false;
                tst.linear.x = 0.0;
                twist_pub->publish(tst);
                break;
            }
            case KEYCODE_ESC:
            {
                thread_active = false;
                tst.linear.x = 0.0;
                twist_pub->publish(tst);
                exit(0);
            }
            default:
            {
            }
        }
    }
}

eskf::eskf(std::string name): Node(name)
{
    // init subscriber
    gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps", 10, std::bind(&eskf::gps_callback, this, std::placeholders::_1));
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu1", 100, std::bind(&eskf::imu_callback, this, std::placeholders::_1));
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("odom", 50, std::bind(&eskf::odom_callback, this, std::placeholders::_1));

    // 重置odom数据
    reset_odom_pub = create_publisher<std_msgs::msg::UInt32>("reset_odom", 10);
    // 最终pub的数据
    fused_pose_pub = create_publisher<nav_msgs::msg::Odometry>("fused_pose", 10);
    fused_pose_pub1 = create_publisher<nav_msgs::msg::Odometry>("fused_pose1", 10);
    //fused_pose_pub2 = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    // 小车目标速度的publisher
    twist_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // 默认小车自身坐标系下y方向速度为0
    declare_parameter<double>("target_twist_x", 1); // m/s
    declare_parameter<double>("target_twist_y", 0.0); // m/s
    declare_parameter<double>("target_twist_z", 50.0); // degree/s

    // init parameters
    declare_parameter<double>("gravity", gravity);
    declare_parameter<double>("earth_rotation_speed", earth_rotation_speed);

    declare_parameter<double>("cov_prior_posi", cov_prior_posi);
    declare_parameter<double>("cov_prior_vel", cov_prior_vel);
    declare_parameter<double>("cov_process_gyro", cov_process_gyro);
    declare_parameter<double>("cov_process_accel", cov_process_accel);

    declare_parameter<double>("cov_measurement_posi_gps", cov_measurement_posi_gps);
    declare_parameter<double>("cov_measurement_posi_odom", cov_measurement_posi_odom);

    declare_parameter<double>("latitude_", latitude_);

    //工作模式：0.nothing 1.录制，2.复现+录制
    declare_parameter<int>("mode", 1);

    // debug模式，输出详细信息
    declare_parameter<bool>("debug", false);

    // init matrix
    g_ = Eigen::Vector3d(0.0, 0.0, -gravity);
    w_ = Eigen::Vector3d(0.0, earth_rotation_speed * cos(latitude_ * kDegree2Radian),
                         earth_rotation_speed * sin(latitude_ * kDegree2Radian));

    SetCovarianceP(cov_prior_posi, cov_prior_vel, cov_prior_ori,
                   cov_prior_epsilon, cov_prior_delta);
    SetCovarianceR(cov_measurement_posi_gps);
    SetCovarianceRo(cov_measurement_posi_odom);
    SetCovarianceQ(cov_process_gyro, cov_process_accel);

    X_.setZero();
    F_.setZero();
    B_.setZero();
    C_.setIdentity();
    G_.block<3,3>(INDEX_MEASUREMENT_POSI,INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();
    Go_.block<3,3>(INDEX_MEASUREMENT_POSI,INDEX_MEASUREMENT_POSI) = Eigen::Matrix3d::Identity();

    F_.block<3,3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity();
    F_.block<3,3>(INDEX_STATE_ORI, INDEX_STATE_ORI) = BuildSkewMatrix(-w_);

    // clear buff
    gps_data_buff_.clear();
    imu_data_buff_.clear();
    odom_data_buff_.clear();

    // init state for pose, velocity initialization
    init_flag_=0;
    
    // init save dir, file_name
    save_dir = "/home/dengsutao/AutowareAuto/src/drivers/eskf/records";
    save_file_name = std::to_string((int)(std::chrono::system_clock::now().time_since_epoch().count() / 1e9))+".txt";
    // 复现时读取的录制文件
    declare_parameter<std::string>("record_in","");
    std::string record_in_file_name = get_parameter("record_in").get_value<std::string>();
    record_in_file_path = save_dir+"/"+record_in_file_name;

    std::string file_name = save_dir+"/"+save_file_name;
    fout.open(file_name, fstream::out | ios_base::trunc);
    // 取消科学计数
    std::cout.setf(ios::fixed,ios::floatfield);init_flag_+=FLAG_RECV_ODOM;

    // 获取mode，如果为2，则读取录制路线数据
    get_parameter("mode", mode);
    std::cout<<"mode="<<mode<<std::endl;
    if (mode==2)
    {
        ifstream fin(record_in_file_path);
        if (!fin.is_open())
        {
            std::cerr<<"failed to open file "<<record_in_file_path<<std::endl;
        }
        std::string line;
        std::string temp;
        vector<GPSData> record_in_data_buff;
        while(std::getline(fin, line))
        {
            std::stringstream ss;
            GPSData record_in_data;
            ss<<line;
            
            //  time:
            std::getline(ss, temp, ',');
            size_t idx = temp.find_first_of(':');
            record_in_data.time = std::stod(temp.substr(idx+1, temp.size()-idx-1));

            // enu_pose:x,y,z
            std::getline(ss, temp, ',');
            idx = temp.find_first_of(':');
            record_in_data.position_enu.x() = std::stod(temp.substr(idx+1, temp.size()-idx-1));
            std::getline(ss, temp, ',');
            record_in_data.position_enu.y() = std::stod(temp);
            std::getline(ss, temp, ',');
            record_in_data.position_enu.z() = std::stod(temp);

            // std::getline(ss, temp, ',');
            // record_in_data.position_lla.x() = std::stod(temp);
            // std::getline(ss, temp, ',');
            // record_in_data.position_lla.y() = std::stod(temp);
            // std::getline(ss, temp, ',');
            // record_in_data.position_lla.z() = std::stod(temp);

            // std::getline(ss, temp, ',');
            // record_in_data.velocity.x() = std::stod(temp);
            // std::getline(ss, temp, ',');
            // record_in_data.velocity.y() = std::stod(temp);
            // std::getline(ss, temp, ',');
            // record_in_data.velocity.z() = std::stod(temp);

            record_in_data_buff.push_back(record_in_data);
        }
        // std::cout<<"record in data length="<<record_in_data_buff.size()<<std::endl;

        record_in_data_buff_.clear();
        // std::cout<<"start path optimization..."<<std::endl;
        // 优化路线，计算最简路线，两点之间间隔最小为1米
        for (size_t i=0;i<record_in_data_buff.size();i++)
        {
            GPSData cur_data = record_in_data_buff.at(i);
            if (i==0) 
            {
                record_in_data_buff_.push_back(cur_data);
                // std::cout<<"coord:"<<cur_data.position_enu.transpose()<<std::endl;
                // std::cout<<"time:"<<cur_data.time<<std::endl;
                continue;
            }
            GPSData& last_data = record_in_data_buff_.at(record_in_data_buff_.size()-1);
            // 目前只考虑2d坐标！
            Eigen::Vector2d cur_xy = cur_data.position_enu.block<2,1>(0,0);
            Eigen::Vector2d last_xy = last_data.position_enu.block<2,1>(0,0);
            if ((cur_xy-last_xy).norm()<1) continue;
            double delta_t = cur_data.time - last_data.time;
            // std::cout<<"offset:"<<(cur_xy-last_xy).transpose()<<std::endl;
            last_data.velocity.x() = (cur_data.position_enu.x() - last_data.position_enu.x()) / delta_t;
            last_data.velocity.y() = (cur_data.position_enu.y() - last_data.position_enu.y()) / delta_t;
            // std::cout<<"velocity:"<<last_data.velocity.transpose()<<std::endl;
            record_in_data_buff_.push_back(cur_data);
            // std::cout<<"coord:"<<cur_data.position_enu.transpose()<<std::endl;
            // std::cout<<"time:"<<cur_data.time<<std::endl;
            //最后的速度没有被更改，默认为0。
        }
        size_t size = record_in_data_buff_.size();
        // std::cout<<"velocity:"<<record_in_data_buff_.at(size-1).velocity.transpose()<<std::endl;
        // std::cout<<"record path node number after optimization:"<<size<<std::endl;
        // 开启速度发布的线程
        thread_active = true;
        monitor_thread = std::thread(&eskf::thread_twist_pub, this);
        monitor_thread.detach();

    }
    // // 开启一个终端键盘控制线程，用于处理突发事故
    // // 设置终端参数,TCSANOW:不等数据传输完毕就立刻改变属性
    // thread_active = true;
    // tcsetattr(kfd, TCSANOW, &cooked);
    // kfd = 0;
    // keyboard_thread = std::thread(&eskf::keyboard_controll, this);
    // keyboard_thread.detach();
}

eskf::~eskf()
{
    geometry_msgs::msg::Twist tst;
    tst.angular.z = 0.0;
    tst.linear.x = 0.0;
    twist_pub->publish(tst);

    thread_active =false;
    fout.close();
    RCLCPP_INFO(get_logger(), "record file:"+save_file_name);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<eskf>("eskf");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
