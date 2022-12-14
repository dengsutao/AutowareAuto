import os

import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    imu_gps_runner = launch_ros.actions.Node(
        package='imu_gps',
        executable='imu_gps_node',
        parameters=[{
            'port': "/dev/imu_gps_usb",
            'baudrate': 115200,
            "debug": False
         }])
    gps_runner = launch_ros.actions.Node(
        package='gps',
        executable='gps_node',
        parameters=[{
            'port': "/dev/gps",
            'baudrate': 9600,
            "debug": False
         }])
    wheel_imu_runner = launch_ros.actions.Node(
        package='wheel_imu',
        executable='wheel_imu_node',
        parameters=[{
            'debug': False,
         }])
    eskf_runner = launch_ros.actions.Node(
        package='eskf',
        executable='eskf_node_exe',
        parameters=[{
            'mode': 0,
            'debug': True,
         }])
    return launch.LaunchDescription([
        # imu_gps_runner,
        # gps_runner,
        # wheel_imu_runner,
        eskf_runner
        ])