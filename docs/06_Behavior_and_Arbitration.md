# 6. Behavior and Arbitration

The **`jetracer_behavior`** package serves as the "Pre-Frontal Cortex" of the JetRacer. It utilizes incoming mathematics to execute concrete action-states, effectively bridging the gap between passive calculation and physical driving logic.

## Decoupled Arbitration (`twist_mux`)

The core architecture prevents the vehicle from receiving simultaneous, conflicting commands by implementing a mathematical arbiter, referred to as a multiplexer. This design ensures that the vehicle never receives or executes opposing instructions at the same time. In complex systems, control inputs often originate from multiple sources, such as an autonomous driving module, an emergency system, or manual user input. Without a coordinating mechanism, these sources could issue conflicting commands simultaneously—for example, accelerating and braking at the same time. The arbiter serves as a central decision-making unit that processes all incoming commands, evaluates them based on defined criteria such as priorities or system states, and selects exactly one valid command to forward to the vehicle. All other competing inputs are suppressed at that moment. This approach guarantees that the vehicle operates in a consistent and controlled manner, even when multiple control processes are active concurrently.

Every node transmits velocity commands linearly to `twist_mux`, but assigned uniquely enforced Priority Tags.

| Priority | Publisher | Subscribed Node | Outcome Context |
| :---: | :--- | :--- | :--- |
| **10** (Highest) | `teleop_joy` | Human Gamepad | Reclaims all control for manual escape maneuvering. |
| **8** | `collision_assurance` | RPLidar Safety Node | Slams Priority Zeros if LiDAR identifies object < `0.45m` away. |
| **8** | `semantic_behavior` | YOLO Logic Node | Slams Priority Zeros if specific item (Stop Sign) fills > `15%` of camera field. |
| **5** | `lane_following` | B-Spline Pathing | Outputs rapid micro-adjustments to remain between white lines at `0.35m/s`. |
| **3** (Lowest) | `nav_server` | Room-to-Room SLAM | Slowest loop outputting map-based goal arrivals. |

## The State Machines

The JetRacer utilizes two independent behavioral nodes that weaponize Priority 8.

### 1. Collision Assurance Logic

The node sets a localized 30° geometric cone mathematically mapping directly onto laser returns immediately forward of the bumper.

- *Interrupt Phase:* If distance drops rapidly below threshold, it intercepts the Multiplexer.
- *Release Phase:* When the obstacle shifts, the node instantly releases Priority 8, letting Priority 5 snap back into operational control to continue racing dynamically!

### 2. Semantic Logic

If YOLO flags `"Stop Sign"`, mathematical area comparisons calculate if the sign implies physical nearness ($[X \times Y] / [Camera\_Scale] > 15\%$).

- *Interrupt Phase:* The car floods Priority 8 with Zeros to brake smoothly.
- *Cooldown Phase:* The Node triggers an internal 5-second `Sleep` state to ignore the Stop Sign while passing it, seamlessly yielding the multiplexing channels back down to Priority 5.

---
> [!TIP]
> **Next Step:** Continue to [07. Voice Commander Stack](07_Voice_Commander_Stack.md) to review how offline Audio affects mapping decisions.
