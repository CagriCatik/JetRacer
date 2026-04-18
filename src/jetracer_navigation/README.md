# jetracer_navigation

ROS 2 migration of the ROS 1 JetRacer navigation package.

## Migrated nodes

- `multipoint_nav.py`: ROS 2 port of the clicked-point patrol tool, implemented against the Nav2 `NavigateToPose` action
- `cmd_vel_to_steering.py`: adapts Nav2 angular velocity commands into the steering-angle convention expected by the JetRacer motor controller

## Launch files

- `nav.launch.py`: map server, AMCL, Nav2 nodes, steering adapter, and optional multipoint patrol
- `slam.launch.py`: mapping-only bringup using `slam_toolbox` or Cartographer
- `slam_nav.launch.py`: mapping plus Nav2 navigation against the live `/map`
- `multipoint_nav.launch.py`: start only the patrol tool

## Notes

- The ROS 1 `move_base` + TEB stack was replaced with Nav2.
- The steering adapter is required because the original JetRacer MCU expects a steering command in `Twist.angular.z`, not a raw yaw-rate command.
- The original Cartographer Lua file was preserved and moved into `config/cartographer/`.
