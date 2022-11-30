#include "wheeltec_robot.h"
#include "Quaternion_Solution.h"
sensor_msgs::msg::Imu Mpu6050;//Instantiate an IMU object //实例化IMU对象
/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char** argv)
{
    // ros::init(argc, argv, "wheeltec_robot"); //ROS initializes and sets the node name //ROS初始化 并设置节点名称 
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turn_on_robot>("robot");
    rclcpp::spin(node);
    return 0;
}

/**************************************
构造函数。声明publish变量/参数，清空缓存数据，读取读取STM32串口数据。
这里的usart_port_name固定为/dev/wheeltec_controller，需要执行scripts下的wheeltec_udev.sh进行映射
***************************************/
turn_on_robot::turn_on_robot(std::string name):Node(name)
{
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    imu_publisher = create_publisher<sensor_msgs::msg::Imu>("imu2", 20);
    voltage_publisher = create_publisher<std_msgs::msg::Float32>("PowerVoltage", 10);

    Sampling_Time = 0;
    Power_voltage = 0;
    //Clear the data
    //清空数据
    memset(&Robot_Pos, 0, sizeof(Robot_Pos));
    memset(&Robot_Vel, 0, sizeof(Robot_Vel));
    memset(&Receive_Data, 0, sizeof(Receive_Data)); 
    memset(&Send_Data, 0, sizeof(Send_Data));
    memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));
    //入口参数分别对应：参数服务器上的名称  参数变量名  初始值
    usart_port_name = "/dev/wheeltec_controller";
    serial_baud_rate = 115200;
    odom_frame_id = "odom_combined";
    robot_frame_id = "base_footprint";
    gyro_frame_id = "gyro_link";
    declare_parameter<std::string>("usart_port_name",  usart_port_name); //Fixed serial port number //固定串口号
    declare_parameter<int>        ("serial_baud_rate", serial_baud_rate); //Communicate baud rate 115200 to the lower machine //和下位机通信波特率115200
    declare_parameter<std::string>("odom_frame_id",    odom_frame_id);      //The odometer topic corresponds to the parent TF coordinate //里程计话题对应父TF坐标
    declare_parameter<std::string>("robot_frame_id",   robot_frame_id); //The odometer topic corresponds to sub-TF coordinates //里程计话题对应子TF坐标
    declare_parameter<std::string>("gyro_frame_id",    gyro_frame_id); //IMU topics correspond to TF coordinates //IMU话题对应TF坐标
    declare_parameter<bool>("debug", false);
    reset_odom = false;

    try
    { 
        proto_config.protocol_type_ = fish_protocol::PROTOCOL_TYPE::SERIAL;
        proto_config.serial_baut_ = serial_baud_rate;
        proto_config.serial_address_ = usart_port_name;
        protocol = fish_protocol::GetProtocolByConfig(proto_config);
        protocol->SetDataRecvCallback(std::bind(&turn_on_robot::Receive_STM32_Data, this, std::placeholders::_1));
    }
    catch (...) // catch all exceptions
    {
        RCLCPP_INFO(get_logger(), "failed to open serial port!");
        exit(0);
    }
    RCLCPP_INFO(get_logger(), "successfully open serial port!");
    
    Init_Last_Time = false;
    
    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, std::placeholders::_1));
    reset_odom_sub = this->create_subscription<std_msgs::msg::UInt32>("reset_odom", 10, std::bind(&turn_on_robot::Reset_Odom_Callback, this, std::placeholders::_1));
    // Recv_Imu_Callback(const std_msgs::msg::UInt32::SharedPtr value)
}

// 设置_Last_Time
void turn_on_robot::Set_Last_Time(rclcpp::Time t)
{
    _Last_Time = t;
}

// 读数据的回调函数，每次读到一批数据（24字节），校验格式是否合法（0x7B开头，0x7D结尾），合法进行数据转换，并发布数据
void turn_on_robot::Receive_STM32_Data(const std::string& stm32_data)
{
    if (reset_odom)
    {
        memset(&Robot_Pos, 0, sizeof(Robot_Pos));
        memset(&Robot_Vel, 0, sizeof(Robot_Vel));
        memset(&Receive_Data, 0, sizeof(Receive_Data)); 
        memset(&Send_Data, 0, sizeof(Send_Data));
        memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));
        reset_odom = false;
    }

    uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE];
    if (stm32_data.size()!=RECEIVE_DATA_SIZE) return;
    for (size_t i=0;i<stm32_data.size();i++)
    {
        Receive_Data_Pr[i] = static_cast<uint8_t>(stm32_data[i]);
    }
    if (Receive_Data_Pr[0]!=FRAME_HEADER || Receive_Data_Pr[RECEIVE_DATA_SIZE-1]!=FRAME_TAIL) return;
    memcpy(Receive_Data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    if (Trans_Data())
    {
        Publish_Data();
    }
    else
    {
        RCLCPP_INFO(get_logger(), "wrong in turn_on_robot::Trans_Data(), check sum failed!");
    }

}

/**************************************
Date: January 28, 2021
Function: Data conversion function
功能: 数据转换函数
***************************************/
short turn_on_robot::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    short transition_16;
    transition_16 = 0;
    transition_16 |=  Data_High<<8;   
    transition_16 |=  Data_Low;
    return transition_16;     
}
float turn_on_robot::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
    float data_return;
    short transition_16;
    transition_16 = 0;
    transition_16 |=  Data_High<<8;  //Get the high 8 bits of data   //获取数据的高8位
    transition_16 |=  Data_Low;      //Get the lowest 8 bits of data //获取数据的低8位
    data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
    return data_return;
}

/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
    unsigned char check_sum=0,k;

    if(mode==0) //Receive data mode //接收数据模式
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
        }
    }
    if(mode==1) //Send data mode //发送数据模式
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
        }
    }
    return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}

/**************************************
功能: 数据转换为国际单位
***************************************/
bool turn_on_robot::Trans_Data()
{
    Receive_Data.Frame_Header= Receive_Data.rx[0]; //The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
    Receive_Data.Frame_Tail= Receive_Data.rx[23]; //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D
    if (Receive_Data.rx[22] == Check_Sum(22,READ_DATA_CHECK)) //BCC check passes or two packets are interlaced //BCC校验通过或者两组数据包交错
    {
        Receive_Data.Flag_Stop=Receive_Data.rx[1]; //set aside //预留位
        Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]); //Get the speed of the moving chassis in the X direction //获取运动底盘X方向速度
        Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]); //Get the speed of the moving chassis in the Y direction, The Y speed is only valid in the omnidirectional mobile robot chassis
                                                                            //获取运动底盘Y方向速度，Y速度仅在全向移动机器人底盘有效
        Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]); //Get the speed of the moving chassis in the Z direction //获取运动底盘Z方向速度   
        
        //MPU6050 stands for IMU only and does not refer to a specific model. It can be either MPU6050 or MPU9250
        //Mpu6050仅代表IMU，不指代特定型号，既可以是MPU6050也可以是MPU9250
        Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8],Receive_Data.rx[9]);   //Get the X-axis acceleration of the IMU     //获取IMU的X轴加速度  
        Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10],Receive_Data.rx[11]); //Get the Y-axis acceleration of the IMU     //获取IMU的Y轴加速度
        Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12],Receive_Data.rx[13]); //Get the Z-axis acceleration of the IMU     //获取IMU的Z轴加速度
        Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14],Receive_Data.rx[15]);  //Get the X-axis angular velocity of the IMU //获取IMU的X轴角速度  
        Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16],Receive_Data.rx[17]);  //Get the Y-axis angular velocity of the IMU //获取IMU的Y轴角速度  
        Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18],Receive_Data.rx[19]);  //Get the Z-axis angular velocity of the IMU //获取IMU的Z轴角速度  
        //Linear acceleration unit conversion is related to the range of IMU initialization of STM32, where the range is ±2g=19.6m/s^2
        //线性加速度单位转化，和STM32的IMU初始化的时候的量程有关,这里量程±2g=19.6m/s^2
        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data / ACCEl_RATIO;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data / ACCEl_RATIO;
        //The gyroscope unit conversion is related to the range of STM32's IMU when initialized. Here, the range of IMU's gyroscope is ±500°/s
        //Because the robot generally has a slow Z-axis speed, reducing the range can improve the accuracy
        //陀螺仪单位转化，和STM32的IMU初始化的时候的量程有关，这里IMU的陀螺仪的量程是±500°/s
        //因为机器人一般Z轴速度不快，降低量程可以提高精度
        Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data * GYROSCOPE_RATIO;
        Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data * GYROSCOPE_RATIO;

        //Get the battery voltage
        //获取电池电压
        short transition_16 = 0;
        transition_16 |=  Receive_Data.rx[20]<<8;
        transition_16 |=  Receive_Data.rx[21];  
        Power_voltage = transition_16/1000+(transition_16 % 1000)*0.001; //Unit conversion millivolt(mv)->volt(v) //单位转换毫伏(mv)->伏(v)

        return true;
    }
    std::cout<<"check failed"<<std::endl;
    return false;
}

/**************************************
功能: 发布数据，包括里程计，IMU，电池电压
***************************************/
void turn_on_robot::Publish_Data()
{
    _Now = get_clock()->now();
    if (!Init_Last_Time)
    {
        _Last_Time = _Now;
        Init_Last_Time = true;
    }
    Sampling_Time = _Now.seconds() - _Last_Time.seconds();
    std::cout<<"now:"<<_Now.seconds()<<std::endl;
    std::cout<<"Sampling_Time:"<<Sampling_Time<<std::endl;
    Robot_Pos.X+=(Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time; //Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
    Robot_Pos.Y+=(Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time; //Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m
    Robot_Pos.Z+=Robot_Vel.Z * Sampling_Time; //The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad 
    Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z, Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);
    Publish_Odom(); 
    Publish_ImuSensor();
    Publish_Voltage();
    _Last_Time = _Now;
    //to do，这里其实应该执行spin_some(),等待回调函数
}

/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void turn_on_robot::Publish_Odom()
{
    //Convert the Z-axis rotation Angle into a quaternion for expression 
    //把Z轴转角转换为四元数进行表达
    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion orientation;
    tf2::fromMsg(orientation, odom_quat);

    nav_msgs::msg::Odometry odom; //Instance the odometer topic data //实例化里程计话题数据
    odom.header.stamp = get_clock()->now();
    odom.header.frame_id = odom_frame_id; // Odometer TF parent coordinates //里程计TF父坐标
    odom.pose.pose.position.x = Robot_Pos.X; //Position //位置
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = orientation; //Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

    odom.child_frame_id = robot_frame_id; // Odometer TF subcoordinates //里程计TF子坐标
    odom.twist.twist.linear.x =  Robot_Vel.X; //Speed in the X direction //X方向速度
    odom.twist.twist.linear.y =  Robot_Vel.Y; //Speed in the Y direction //Y方向速度
    odom.twist.twist.angular.z = Robot_Vel.Z; //Angular velocity around the Z axis //绕Z轴角速度 

    //There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    if(Robot_Vel.X== 0&&Robot_Vel.Y== 0&&Robot_Vel.Z== 0)
      //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
      //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
      memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    else
      //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
      //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
      memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));       
    odom_publisher->publish(odom); //Pub odometer topic //发布里程计话题
    if (get_parameter("debug").as_bool())
    RCLCPP_INFO(get_logger(), "odom:"+std::to_string(odom.pose.pose.position.x)+\
                                ","+std::to_string(odom.pose.pose.position.y)+\
                                ","+std::to_string(odom.pose.pose.position.z)+\
                                ","+std::to_string(odom.twist.twist.linear.x)+\
                                ","+std::to_string(odom.twist.twist.linear.y)+\
                                ","+std::to_string(odom.twist.twist.angular.z));
                    
}

void turn_on_robot::Publish_ImuSensor()
{
    sensor_msgs::msg::Imu Imu_Data_Pub; //Instantiate IMU topic data //实例化IMU话题数据
    Imu_Data_Pub.header.stamp = get_clock()->now();
    Imu_Data_Pub.header.frame_id = gyro_frame_id; //IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack 
                                                //IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
    Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; //A quaternion represents a three-axis attitude //四元数表达三轴姿态
    Imu_Data_Pub.orientation.y = Mpu6050.orientation.y; 
    Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
    Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
    Imu_Data_Pub.orientation_covariance[0] = 1e6; //Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e-6;
    Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; //Triaxial angular velocity //三轴角速度
    Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
    Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; //Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
    Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; //Triaxial acceleration //三轴线性加速度
    Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y; 
    Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;  
    imu_publisher->publish(Imu_Data_Pub); //Pub IMU topic //发布IMU话题
    /*
    std::cout<<"imu:"<<"\norientation-->"<<Imu_Data_Pub.orientation.x\
                                        <<","<<Imu_Data_Pub.orientation.y\
                                        <<","<<Imu_Data_Pub.orientation.z\
                                        <<","<<Imu_Data_Pub.orientation.w\
                    <<"\nangular_velocity-->"<<Imu_Data_Pub.angular_velocity.x\
                                        <<","<<Imu_Data_Pub.angular_velocity.y\
                                        <<","<<Imu_Data_Pub.angular_velocity.z\
                    <<"\nlinear_acceleration-->"<<Imu_Data_Pub.linear_acceleration.x\
                                        <<","<<Imu_Data_Pub.linear_acceleration.y\
                                        <<","<<Imu_Data_Pub.linear_acceleration.z\
                                        <<std::endl;*/
}

void turn_on_robot::Publish_Voltage()
{
    std_msgs::msg::Float32 voltage_msgs; //Define the data type of the power supply voltage publishing topic //定义电源电压发布话题的数据类型
    static float Count_Voltage_Pub=0;
    if(Count_Voltage_Pub++>10)
        {
        Count_Voltage_Pub=0;  
        voltage_msgs.data = Power_voltage; //The power supply voltage is obtained //电源供电的电压获取
        voltage_publisher->publish(voltage_msgs); //Post the power supply voltage topic unit: V, volt //发布电源电压话题单位：V、伏特
        }
    // std::cout<<"voltage:"<<voltage_msgs.data<<std::endl;
}

void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
    short transition;  //intermediate variable //中间变量
    Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
    Send_Data.tx[1] = 0; //set aside //预留位
    Send_Data.tx[2] = 0; //set aside //预留位

    //The target velocity of the X-axis of the robot
    //机器人x轴的目标线速度
    transition=0;
    transition = twist_aux->linear.x*1000; //将浮点数放大一千倍，简化传输
    Send_Data.tx[4] = transition;     //取数据的低8位
    Send_Data.tx[3] = transition>>8;  //取数据的高8位

    //The target velocity of the Y-axis of the robot
    //机器人y轴的目标线速度
    transition=0;
    transition = twist_aux->linear.y*1000;
    Send_Data.tx[6] = transition;
    Send_Data.tx[5] = transition>>8;

    //The target angular velocity of the robot's Z axis
    //机器人z轴的目标角速度
    transition=0;
    transition = twist_aux->angular.z*1000;
    Send_Data.tx[8] = transition;
    Send_Data.tx[7] = transition>>8;

    Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
    Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D //帧尾0X7D
    try
    {
        std::string data(SEND_DATA_SIZE,' ');
        for (int i=0;i<SEND_DATA_SIZE;i++)
        {
            data[i] = static_cast<char>(Send_Data.tx[i]);
        }
        protocol->ProtocolSendRawData(data);
    }
    catch (...)   
    {
        RCLCPP_ERROR(get_logger(), "Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
    }
}

void turn_on_robot::Reset_Odom_Callback(const std_msgs::msg::UInt32::SharedPtr value)
{
    if (value->data) reset_odom=true;
}