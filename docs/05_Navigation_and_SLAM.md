# 05. Navigation and SLAM 

The **`jetracer_navigation`** and **`jetracer_localization`** packages merge incoming geometric and dynamic signals to physically locate the robot within a room and chart the mathematically optimal track line over multiple rooms.

## Sensor Fusion (The EKF)

A solitary sensor is untrustworthy. Encoders slip on dust (predicting infinite speed). IMUs succumb to electromagnetic drift when passing heavy wiring (distorting your sense of North). 

Using the `robot_localization` ROS 2 package, we construct an **Extended Kalman Filter (EKF)** that mathematically averages the sensors simultaneously:

- **Gyroscopic Yaw:** Handled exclusively by the `BNO085` AHRS Sensor. 
- **Linear Drive:** Handled exclusively by the Hall-Effect `Wheel Encoders`.
- **Constraint Handling**: We specifically drop the IMU's raw Linear Acceleration vectors from the EKF array (`ekf.yaml`) because double-integrating them on 4GB compute introduces wild algorithmic runaway.

## Autonomous 2D Pathfinding

```mermaid
graph TD
    subgraph Mapping Layer
      Lidar[RPLidar A1] -->|/scan| SLAM[slam_toolbox]
      SLAM -->|Creates| MAP[(2D Occupancy Grid)]
    end

    subgraph Nav2 Pathing
      MAP --> PLAN[SmacPlannerHybrid]
      EKF[EKF Localization] --> PLAN
      PLAN -->|Outputs Yaw Rate| NAVCmd[cmd_vel_nav]
    end

    subgraph Hardware Constraint
      NAVCmd --> INV[cmd_vel_to_steering.py]
      INV -->|Calculates Ackermann Angle| TWIST[twist_mux]
    end
```

### Navigating Like a Car
Robots like Roombas use "Differential Drive"—meaning they mathematically output trajectories that demand turning in place. 
Your JetRacer uses a physical rack-and-pinion front axis. Therefore:
1. Nav2 relies on the **SmacPlannerHybrid**, configuring exclusively to an `ACKERMANN` motion model with a locked $0.40m$ turning radius. 
2. The `cmd_vel_to_steering.py` inverse kinematics script physically maps the requested Yaw rotation to your vehicle's physical $0.255m$ wheelbase, ensuring the tires actually pivot to the exact geometric angle requested by the AI.

### Frontier Exploration (Auto-Mapping)
By loading the `explore_lite` package, the car evaluates its own 2D Occupancy Grid. It recognizes the ragged edges between "Known White Floor" and "Unknown Gray Void" as algorithmic *Frontiers*. It autonomously injects coordinate destinations to these frontiers, driving until the Map closes!

---
> [!TIP]
> **Next Step:** Understand how we stop simultaneous sensors from overriding each other in [06. Behavior and Arbitration](06_Behavior_and_Arbitration.md).
