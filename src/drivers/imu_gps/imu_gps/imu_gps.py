# -*- coding:utf-8 -*-
import serial
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from quaternions import Quaternion 

key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]

ACCData = [0.0]*8
GYROData = [0.0]*8
AngleData = [0.0]*8
FrameState = 0  # What is the state of the judgment
Bytenum = 0  # Read the number of digits in this paragraph
CheckSum = 0  # Sum check bit

a = [0.0]*3
w = [0.0]*3
Angle = [0.0]*3

datatypes = []

def get_acc(datahex):
    axl = datahex[0]
    axh = datahex[1]
    ayl = datahex[2]
    ayh = datahex[3]
    azl = datahex[4]
    azh = datahex[5]
    k_acc = 16.0
    acc_x = (axh << 8 | axl) / 32768.0 * k_acc
    acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
    acc_z = (azh << 8 | azl) / 32768.0 * k_acc
    if acc_x >= k_acc:
        acc_x -= 2 * k_acc
    if acc_y >= k_acc:
        acc_y -= 2 * k_acc
    if acc_z >= k_acc:
        acc_z -= 2 * k_acc
    return [acc_x, acc_y, acc_z]

def get_gyro(datahex):
    wxl = datahex[0]
    wxh = datahex[1]
    wyl = datahex[2]
    wyh = datahex[3]
    wzl = datahex[4]
    wzh = datahex[5]
    k_gyro = 2000.0
    gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
    gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
    gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
    if gyro_x >= k_gyro:
        gyro_x -= 2 * k_gyro
    if gyro_y >= k_gyro:
        gyro_y -= 2 * k_gyro
    if gyro_z >= k_gyro:
        gyro_z -= 2 * k_gyro
    return [gyro_x, gyro_y, gyro_z]

def get_angle(datahex):
    rxl = datahex[0]
    rxh = datahex[1]
    ryl = datahex[2]
    ryh = datahex[3]
    rzl = datahex[4]
    rzh = datahex[5]
    k_angle = 180.0
    angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
    angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
    angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
    if angle_x >= k_angle:
        angle_x -= 2 * k_angle
    if angle_y >= k_angle:
        angle_y -= 2 * k_angle
    if angle_z >= k_angle:
        angle_z -= 2 * k_angle
    return [angle_x, angle_y, angle_z]

def get_time(datahex):
    YY = 2000 + datahex[0]
    MM = datahex[1]
    DD = datahex[2]
    HH = datahex[3]
    MN = datahex[4]
    SS = datahex[5]
    MSL = datahex[6]
    MSH = datahex[7]
    MS = ((MSH<<8)|MSL)
    return str(YY) + str(MM) + str(DD) + str(HH) + str(MN) + str(SS) + str(MS)

def get_loc(datahex):
    Lon0 = datahex[0]
    Lon1 = datahex[1]
    Lon2 = datahex[2]
    Lon3 = datahex[3]
    Lat0 = datahex[4]
    Lat1 = datahex[5]
    Lat2 = datahex[6]
    Lat3 = datahex[7]

    Lon = (Lon3<<24)|(Lon2<<16)|(Lon1<<8)|Lon0
    Lat = (Lat3<<24)|(Lat2<<16)|(Lat1<<8)|Lat0

    return [Lat/10000000.0, Lon/10000000.0, 0.0]

def split_data(inputdata):
    # default include :加速度,角速度,角度,磁场,端口状态,气压,经纬度,四元数
    if inputdata[0] == 0x55 and inputdata[1] == 0x50:
        print('时间')
        return None
    elif inputdata[0] == 0x55 and inputdata[1] == 0x51:
        print('加速度')
        linear_acceleration = get_acc(inputdata[2:10])
        linear_acceleration[0] = -linear_acceleration[0]
        linear_acceleration[1] = -linear_acceleration[1]
        linear_acceleration[2] = -linear_acceleration[2]
        return {'linear_acceleration':linear_acceleration}
    elif inputdata[0] == 0x55 and inputdata[1] == 0x52:
        print('角速度')
        angular_velocity = get_gyro(inputdata[2:10])
        return {"angular_velocity":angular_velocity}
    elif inputdata[0] == 0x55 and inputdata[1] == 0x53:
        print('角度')
        angle_degree = get_angle(inputdata[2:10])
        angle_degree[2] = (angle_degree[2] + 90.0 + 180.0) % 360.0 - 180.0
        return {"angle_degree":angle_degree}
    elif inputdata[0] == 0x55 and inputdata[1] == 0x54:
        print('磁场')
        return None
    elif inputdata[0] == 0x55 and inputdata[1] == 0x55:
        print('端口状态')
        return None
    elif inputdata[0] == 0x55 and inputdata[1] == 0x56:
        print('气压')
        return None
    elif inputdata[0] == 0x55 and inputdata[1] == 0x57:
        print('经纬度')
        lla = get_loc(inputdata[2:10])
        return {"lla":lla}
    elif inputdata[0] == 0x55 and inputdata[1] == 0x58:
        print('GPS')
        return None
    elif inputdata[0] == 0x55 and inputdata[1] == 0x59:
        print('四元数')
        return None
    elif inputdata[0] == 0x55 and inputdata[1] == 0x5A:
        print('GPS定位精度')
        return None
    elif inputdata[0] == 0x55 and inputdata[1] == 0x5F:
        print('四个寄存器')
        return None
    else:
        print('error datahex')
        exit()

def read_one_batch(ser):
    data_dict = {}
    for i in range(8):
        datahex = ser.read(11)
        data = split_data(datahex)
        if not data is None:
            data_dict.update(data)
    return data_dict

class IMU_GPS_Node(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("port", "/dev/imu_gps_usb")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("debug", False)
        self.timeout = 0.5
        self.pub_imu = self.create_publisher(Imu, "imu1", 100)
        self.pub_gps = self.create_publisher(NavSatFix, "gps", 10)
        self.ser = self.init_serial()

    def init_serial(self):
        port = self.get_parameter("port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
        self.get_logger().info(f"port={port},baudrate={baudrate}")
        try:
            ser = serial.Serial(port=port, baudrate=baudrate, timeout=self.timeout)
            if ser.is_open:
                self.get_logger().info("Serial port opened successfully")
        except Exception as e:
            self.get_logger().info("Serial port opening failure")
            exit(0)
        else:
            return ser

    def read_and_publish(self):
        data_dict = read_one_batch(self.ser)
        print(data_dict.keys())
        # pub imu
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"
        
        angle_radian = [data_dict['angle_degree'][i] * math.pi / 180 for i in range(3)]
        qua = Quaternion.from_euler(angle_radian[::-1], axes = ['z', 'y', 'x']) 

        imu_msg.orientation.x = qua.x
        imu_msg.orientation.y = qua.y
        imu_msg.orientation.z = qua.z
        imu_msg.orientation.w = qua.w

        imu_msg.angular_velocity.x = data_dict["angular_velocity"][0]
        imu_msg.angular_velocity.y = data_dict["angular_velocity"][1]
        imu_msg.angular_velocity.z = data_dict["angular_velocity"][2]

        imu_msg.linear_acceleration.x = data_dict["linear_acceleration"][0]
        imu_msg.linear_acceleration.y = data_dict["linear_acceleration"][1]
        imu_msg.linear_acceleration.z = data_dict["linear_acceleration"][2]

        self.pub_imu.publish(imu_msg)
        # pub gps
        gps_msg = NavSatFix()
        gps_msg.latitude = data_dict["lla"][0]
        gps_msg.longitude = data_dict["lla"][1]
        gps_msg.altitude = data_dict["lla"][2]
        self.pub_gps.publish(gps_msg)
        if self.get_parameter("debug").value:
            self.get_logger().info("imu:\n"+"angle-->"+str(data_dict['angle_degree'])+"\nori-->"+str(imu_msg.orientation)+"\nangular_velocity-->"+str(imu_msg.angular_velocity)+"\nlinear_acceleration-->"+str(imu_msg.linear_acceleration))
            self.get_logger().info("gps:\n"+"lat,lon,alt-->"+str(gps_msg.latitude)+","+str(gps_msg.longitude)+","+str(gps_msg.altitude))

def main(args=None):
    rclpy.init(args=args)
    imu_gps_node = IMU_GPS_Node("imu_gps_node")
    while rclpy.ok:
        imu_gps_node.read_and_publish()
    rclpy.spin(imu_gps_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main(None)
