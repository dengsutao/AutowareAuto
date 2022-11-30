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
    wheel_imu_runner = launch_ros.actions.Node(
        package='wheel_imu',
        executable='wheel_imu_node',
        parameters=[{
            'mode': 1,
            'debug': False,
         }])
    eskf_runner = launch_ros.actions.Node(
        package='eskf',
        executable='eskf_node',
        parameters=[{
            'mode': 1,
            'debug': False,
         }])
    return launch.LaunchDescription([
        imu_gps_runner,
        wheel_imu_runner,
        eskf_runner
        ])