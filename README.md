# ROS2_Brake_Decider

A ROS 2 C++ node that decides whether to issue a brake command based on ego speed and obstacle distance, with tunable parameters (reaction_time, decel, and safety_margin). 

Implements the classic stopping-distance model:

`d_stop = v * t_react + v² / (2a)`

…and compares it to available distance with a safety margin:

`d_avail = d * (1 - safety_margin)`

Publishes `std_msgs/Bool` on `/brake_cmd` at 20 Hz.

---

## Why this exists

Small, focused node to demonstrate an ADAS-style decision in ROS 2:
- Clear separation of concerns: **sensors → decision → actuation**.
- Parameters are tunable at runtime (no rebuild).
- Tiny enough to read in one sitting, useful as a building block in larger stacks.

---


## Project Context

This node is Stage 2 of my [ADAS Learning Sprint](https://github.com/IvanMcCauley/Adas_Learning_Sprint):

1. **[Braking decision library (C++17)](https://github.com/IvanMcCauley/braking_decision_lib)** - standalone math & unit tests  
2. **ROS 2 integration (this repo)** - parameterized node with pub/sub  
3. **1D longitudinal simulation** - test decisions against vehicle dynamics


---

## Features

- Subscribes:  
  `/ego_speed` (`std_msgs/Float64`), `/obstacle_distance` (`std_msgs/Float64`)
- Publishes:  
  `/brake_cmd` (`std_msgs/Bool`) at 20 Hz
- Parameters (live-tunable):
  - `reaction_time` (s) - default `1.0`
  - `decel` (m/s², magnitude) - default `9.0` (must be `> 0`)
  - `safety_margin` (0–1) - default `0.10`
- Input validation in the parameter callback:
  - Rejects `decel <= 0`
  - Rejects `reaction_time < 0`
  - Rejects `safety_margin` outside `[0, 1]`

---

## Quick start

### Requirements
- Ubuntu 22.04 + ROS 2 **Humble** (ros-base is fine)
- `colcon`, `rclcpp`, `std_msgs`

### Build
```bash
# inside your colcon workspace
cd ~/ros2_ws/src
git clone https://github.com/<you>/ros2_brake_decider.git
cd ..
colcon build --packages-select ros2_brake_decider
source install/setup.bash

### Run with launch (loads params from YAML)
```bash
ros2 launch ros2_brake_decider brake_decider.launch.py
```
### Drive it with fake data (in two more terminals)
```bash
# Terminal B (speed)
source ~/ros2_ws/install/setup.bash
ros2 topic pub /ego_speed std_msgs/msg/Float64 '{data: 10.0}' -r 2

# Terminal C (distance)
source ~/ros2_ws/install/setup.bash
ros2 topic pub /obstacle_distance std_msgs/msg/Float64 '{data: 100.0}' -r 2
```
#### See decisions:
```bash
ros2 topic echo /brake_cmd
```

---

## Parameters

YAML file: `config/brake_params.yaml`

```yaml
brake_decider:
  ros__parameters:
    reaction_time: 1.0     # seconds
    decel: 9.0             # m/s^2 (must be > 0)
    safety_margin: 0.1     # fraction (0..1)
```
Live tune without restarting:
```bash
ros2 param set /brake_decider reaction_time 2.0
ros2 param set /brake_decider decel 7.0
ros2 param set /brake_decider safety_margin 0.25
```

## Topics
<table> <tr> <th>Name</th> <th>Type</th> <th>Direction</th> <th>Notes</th> </tr> <tr> <td><code>/ego_speed</code></td> <td><code>std_msgs/Float64</code></td> <td>subscribe</td> <td>ego velocity in m/s</td> </tr> <tr> <td><code>/obstacle_distance</code></td> <td><code>std_msgs/Float64</code></td> <td>subscribe</td> <td>distance to obstacle in m</td> </tr> <tr> <td><code>/brake_cmd</code></td> <td><code>std_msgs/Bool</code></td> <td>publish</td> <td><code>true</code> = brake, <code>false</code> = no brake</td> </tr> </table>

**Rate:** decisions published at ~20 Hz via a wall timer (not on every message), which keeps output steady even if inputs arrive at different rates.

## Decision Logic
1. Read latest ego speed <code>v</code> and obstacle distance <code>d</code>.
2. Compute reaction distance: d_react = <code>v * reaction_time</code>.
3. Compute brake distance: d_brake = <code>v^2 / (2 * decel)</code>.
4. Compute stopping distance: <code>d_stop = d_react + d_brake</code>.
5. Compute available distance: <code>d_avail = d * (1 - safety_margin)</code>.
6. If <code>d_stop >= d_avail</code>, publish <code>true</code>, else <code>false</code>.

**Edge cases:**
If <code>decel</code> is invalid <code>(<= 0)</code>, the parameter change is rejected; node continues with last known good value.

### Core C++ snippet
```cpp
// 20 Hz: read params + latest sensors, compute decision, publish /brake_cmd
tick_timer_ = this->create_wall_timer(
  std::chrono::milliseconds(50),
  [this]() {
    // read current params (so live updates apply)
    double rt{}, decel{}, margin{};
    this->get_parameter("reaction_time", rt);
    this->get_parameter("decel", decel);
    this->get_parameter("safety_margin", margin);

    // latest sensor values (updated by subscribers)
    const double v = last_speed_;     // m/s
    const double d = last_distance_;  // m

    // stopping distance = reaction + braking (decel is a magnitude)
    const double d_react = v * rt;
    const double d_brake = (decel > 0.0) ? (v * v) / (2.0 * decel)
                                         : std::numeric_limits<double>::infinity();
    const double d_stop  = d_react + d_brake;
    const double d_avail = d * (1.0 - margin);

    std_msgs::msg::Bool cmd;
    cmd.data = (d_stop >= d_avail);
    brake_pub_->publish(cmd);
  }
);

```
---
## Minimal code layout
```
ros2_brake_decider/
├─ include/               # (reserved, minimal public API today)
├─ src/
│  └─ brake_decider_node.cpp   # the node (params, subs, timer, publisher)
├─ launch/
│  └─ brake_decider.launch.py  # loads YAML + runs node
├─ config/
│  └─ brake_params.yaml        # default parameters
├─ CMakeLists.txt
└─ package.xml
```

### Browse the code:

| Path | Description |
|------|-------------|
| [src/brake_decider_node.cpp](src/brake_decider_node.cpp) | The node (params, subs, timer, publisher) |
| [launch/brake_decider.launch.py](launch/brake_decider.launch.py) | Loads YAML + runs node |
| [config/brake_params.yaml](config/brake_params.yaml) | Default parameters |
| [CMakeLists.txt](CMakeLists.txt) | Build config |
| [package.xml](package.xml) | Package manifest |

---

## Runbook (handy commands)
Inspect the node:
```bash
ros2 node info /brake_decider
```

Show topics:
```bash
ros2 topic list | egrep 'ego_speed|obstacle_distance|brake_cmd'
```

Check publish rate:
```bash
ros2 topic hz /brake_cmd
```

One-shot test values:
```bash
ros2 topic pub /ego_speed std_msgs/msg/Float64 '{data: 10.0}' --once
ros2 topic pub /obstacle_distance std_msgs/msg/Float64 '{data: 15.0}' --once   # likely brake=true
ros2 topic pub /obstacle_distance std_msgs/msg/Float64 '{data: 100.0}' --once  #
```

---

## Troubleshooting

- <code>WARNING: topic [/brake_cmd] does not appear to be published yet</code>
Make sure the node is running: <code>ros2 node list</code> should show <code>/brake_decider</code>.
Also confirm publisher exists: <code>ros2 node info /brake_decider</code> → look under **Publishers**.

- **No log output from subscribers:**
Verify exact topic names (/ego_speed, /obstacle_distance) and that you sourced the workspace in each terminal:
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/ros2_ws/install/setup.bash
  ```

- **Parameter changes “failed”**
  The node rejects invalid values:
  - <code>decel <= 0</code>
  - <code>reaction_time < 0</code>
  - <code>safety_margin</code> not in <code>[0,1]</code>
   See the node terminal for the <code>[WARN]</code> message.

- **Launch can’t find YAML/launch file**
  - Confirm CMake installs assets:
  ```cmake
  install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
  ```

---

## Performance notes

The math is trivial (a few doubles per tick). The node runs comfortably at much higher rates than 20 Hz; I choose 20 Hz for readability and to avoid spamming logs.

## Roadmap
- Optional debounce/hysteresis so /brake_cmd doesn’t chatter near the boundary
- Optional diagnostic topic with current d_stop, d_avail, and inputs
- Stage 3: integrate with a 1D longitudinal simulation to compare decisions vs. actual stopping distance under dynamics

## License
MIT: See License

## Credits
This node re-implements the stopping-distance logic from the companion C++ library braking_decision_lib I made (insert link), adapted for ROS 2 with live parameters and pub/sub wiring.
