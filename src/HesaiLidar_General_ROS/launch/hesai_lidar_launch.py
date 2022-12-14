from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package ='hesai_lidar',
            node_namespace ='hesai',
            node_executable ='hesai_lidar_node',
            name ='hesai_node',
            output ="screen",
            parameters=[
                {"pcap_file": ""},
                {"server_ip"  : "10.81.81.99"},
                {"lidar_recv_port"  : 2368},
                {"gps_port"  : 10110},
                {"start_angle"  : 0.0},
                {"lidar_type"  : "PandarQT"},
                {"frame_id"  : "PandarQT"},
                {"pcldata_type"  : 0},
                {"publish_type"  : "both"},
                {"timestamp_type"  : "''"},
                {"data_type"  : "''"},
                {"lidar_correction_file"  : "./src/HesaiLidar_General_ROS/config/PandarQT.csv"},
                {"multicast_ip"  : "''"},
                {"coordinate_correction_flag"  : False},
                {"fixed_frame"  : "''"},
                {"target_frame_frame"  : "''"}
            ]
        )
    ])



