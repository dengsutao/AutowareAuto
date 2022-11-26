lgsvl_laser_adapter_nodes {#lgsvl_laser_adapter_nodes-package-design}
===========

This is the design document for the `lgsvl_laser_adapter_nodes` package.


# Purpose / Use cases

The LGSVL 2D LiDAR sensor has a bug in its output `LaserScan` message. The number of scan points should be `angle_max - angle_min / angle_increment + 1`, where the `+ 1` comes from including both the max and min angles. But LGSVL did not include the last scan point (`angle_max`) in the data sequence.

Packages such as `slam_toolbox` need these numbers to match. Since LGSVL is at EOL, This package resolves such issue.

# Design

Upon receiving a `LaserScan` from LGSVL, the node subtracts `angle_max` by `angle_increment`.

## Inputs / Outputs / API

### Subscribers

- `sensor_msgs/msg/LaserScan`: `scan_in`

### Publishers

- `sensor_msgs/msg/LaserScan`: `scan_out`

## Inner-workings / Algorithms
<!-- If applicable -->


## Error detection and handling
<!-- Required -->


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->


# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->


# Related issues
<!-- Required -->
