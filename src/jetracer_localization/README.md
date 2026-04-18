# jetracer_localization

This package contains the ROS 2 migration of the ROS 1 state-estimation layer.

## Included now

- `localization.launch.py`: starts `robot_localization` EKF and remaps filtered odometry to `odom`
- `odom_pose_to_odometry.py`: compatibility node for pose-with-covariance sources that need republishing as `nav_msgs/Odometry`
- `config/ekf.yaml`: baseline EKF configuration using `odom_raw` and `imu`
