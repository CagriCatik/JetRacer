# jetracer_hardware

ROS 2 port of the ROS 1 JetRacer hardware package.

## Migrated nodes

- `jetracer_serial_node`: serial bridge for the JetRacer microcontroller, IMU, odometry, and motor telemetry
- `laser_filter.py`: ROS 2 port of the original scan masking node
- `calibrate_linear.py`: ROS 2 port of the original odometry scale calibration helper

## Launch files

- `hardware.launch.py`: start the serial bridge
- `lidar.launch.py`: start the RPLIDAR driver with ROS 2 parameters
- `laser_filter.launch.py`: start the LiDAR driver and the migrated scan filter
- `calibrate_linear.launch.py`: start the migrated linear calibration node

## Topics

- subscribes: `cmd_vel`
- publishes: `imu`, `odom_raw`, `motor/lvel`, `motor/rvel`, `motor/lset`, `motor/rset`

## Notes

- The serial protocol is preserved from the ROS 1 package.
- Dynamic reconfigure was replaced with ROS 2 parameters.
- The LiDAR frame is expected to be provided by the robot description TF tree.
