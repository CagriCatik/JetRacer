# RViz

Reusable RViz profiles for the JetRacer ROS 2 stack:

- `jetracer.rviz`: description and TF-centric profile
- `navigation.rviz`: map-based navigation profile
- `slam.rviz`: live mapping profile
- `autonomy.rviz`: autonomy/lane-following telemetry profile

Use with:

```bash
ros2 launch jetracer_description rviz.launch.py rviz_profile:=navigation
```
