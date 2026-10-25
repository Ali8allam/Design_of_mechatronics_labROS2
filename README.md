# Lab 1 – ROS 2, Turtlesim, Actions & Services

Contributers :

 Ali Nabil Ali  9245 
 Sherif Mohamed 9359
 Ahmed Mohamed  9288

ROS 2 distribution tested:(Jazzy) & the code uses only `rclpy` APIs that exist in .

---

## 1. Packages

| Package | Contents |
|---------|----------|
| `lab1_interfaces`  | `msg/TargetPose` (x, y, yaw), `action/GoToPose`, `action/DrawSquare`, `srv/SetSpeedLimits` |

| `lab1_turtle_control` | `go_to_pose_server` (action + speed params + speed service), `reset_service`, `square_action_server`, pure-Python motion logic + unit tests |

| `lab1_bringup` |`launch/system.launch.py`, `launch/square_demo.launch.py`, `config/turtle_params.yaml` |

### Interfaces nodes use to talk to each other 

TargetPose.msg        float32 x, y, yaw

GoToPose.action       goal: TargetPose target
                      result: bool success, float32 distance_error, float32 yaw_error, string message
                      feedback: float32 distance_remaining, float32 angle_remaining, string phase
DrawSquare.action     goal: float32 side_length (0 = default 3.0)
                      result: bool success, uint8 goals_completed, string message
                      feedback: uint8 current_side, string status, float32 distance_remaining
SetSpeedLimits.srv    request: float64 max_linear_speed, float64 max_angular_speed
                      response: bool success, string message
```

### Nodes, topics, services, actions
| Node | Provides | Uses |
|------|----------|------|
| `go_to_pose_server` | action `/go_to_pose`, service `/set_speed_limits`, params `max_linear_speed`, `max_angular_speed` | sub `/turtle1/pose`, pub `/turtle1/cmd_vel` |
| `reset_service` | service `/reset_turtle` (`std_srvs/Trigger`) | turtlesim `/turtle1/set_pen`, `/turtle1/teleport_absolute`, `/clear` |
| `square_action_server` | action `/draw_square` | service `/reset_turtle`, action `/go_to_pose` (4 goals) |

---

## 2. Build

```bash
# ROS 2 + turtlesim must be installed, e.g.:  sudo apt install ros-$ROS_DISTRO-turtlesim
mkdir -p ~/lab1_ws/src && cd ~/lab1_ws/src
git clone <THIS_REPO_URL> .
cd ~/lab1_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash          # in EVERY new terminal
```

Run the unit tests (no ROS graph needed; they simulate the turtle):
```bash
colcon test --packages-select lab1_turtle_control && colcon test-result --verbose
```

---

## 3. Run everything

```bash
ros2 launch lab1_bringup system.launch.py
# custom speed limits from the launch file:
ros2 launch lab1_bringup system.launch.py max_linear_speed:=1.0 max_angular_speed:=1.5
# start the system AND draw the square automatically:
ros2 launch lab1_bringup square_demo.launch.py side_length:=3.0
```

## 4. Commands to try every requirement

**R1 – turtlesim controllable from our nodes**
```bash
ros2 node list          # /turtlesim /go_to_pose_server /reset_service /square_action_server
ros2 topic list         # /turtle1/pose  /turtle1/cmd_vel
ros2 service list       # /clear /reset /spawn /turtle1/teleport_absolute ... /reset_turtle /set_speed_limits
```

**R2 – custom message / interfaces**
```bash
ros2 interface list | grep lab1_interfaces
ros2 interface show lab1_interfaces/msg/TargetPose
ros2 interface show lab1_interfaces/action/GoToPose
ros2 interface show lab1_interfaces/action/DrawSquare
ros2 interface show lab1_interfaces/srv/SetSpeedLimits
```

**R3 – goToPose action** (coordinates must be inside 0..11)
```bash
ros2 action list
ros2 action send_goal /go_to_pose lab1_interfaces/action/GoToPose "{target: {x: 8.0, y: 8.0, yaw: 1.57}}" --feedback
ros2 action send_goal /go_to_pose lab1_interfaces/action/GoToPose "{target: {x: 2.0, y: 3.0, yaw: -3.0}}" --feedback
ros2 topic echo /turtle1/pose --once        # compare with the goal (tolerance 0.05 / 0.03 rad)
# goals outside the window are rejected:
ros2 action send_goal /go_to_pose lab1_interfaces/action/GoToPose "{target: {x: 15.0, y: 5.0, yaw: 0.0}}"
```

**R4 – reset service**
```bash
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 topic echo /turtle1/pose --once        # x ~ 5.544, y ~ 5.544, theta 0
```

**R5 – square action** (resets first, then exactly 4 goToPose goals)
```bash
ros2 action send_goal /draw_square lab1_interfaces/action/DrawSquare "{side_length: 3.0}" --feedback
ros2 action send_goal /draw_square lab1_interfaces/action/DrawSquare "{side_length: 0.0}"   # default side
# the go_to_pose_server log shows "Goal #1 ... Goal #4 accepted"
```

**R6 – launch files:** in section 3 .
```bash
ros2 launch lab1_bringup system.launch.py
```

**R7 – speed parameters and speed service (live)**
```bash
ros2 param list /go_to_pose_server
ros2 param get  /go_to_pose_server max_linear_speed
ros2 param set  /go_to_pose_server max_linear_speed 0.5      # note: write 0.5, not 1/2
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear_speed: 1.0, max_angular_speed: 1.5}"
ros2 param get  /go_to_pose_server max_angular_speed         # -> 1.5
# invalid values are refused (service returns success=false, `ros2 param set` is rejected):
ros2 service call /set_speed_limits lab1_interfaces/srv/SetSpeedLimits "{max_linear_speed: -1.0, max_angular_speed: 1.0}"
ros2 param set /go_to_pose_server max_linear_speed 0.0
```
Try it live: send a long goal in one terminal and change the limit from another while the turtle moves.

**R8 – packages:** three packages as described in section 1.

---

## 5. How it works (short)

* **goToPose** is a three-phase controller: rotate to face the target → drive while steering → rotate to the final yaw. Angle errors are wrapped to [-pi, pi]. The speed limits are read from the ROS parameters on every control tick (50 Hz), so changes apply immediately. Goals outside the window are rejected, a second goal is rejected while one is running, goals time out after 60 s, and the turtle is always stopped when a goal ends or is cancelled.
* **reset** lifts the pen, teleports to the start pose, clears the screen, and puts the pen down again (even if a step fails).
* **square** resets, waits 0.3 s for the pose to update, then sends 4 goals (right, up, left, down corners). The final yaw of each corner already points along the next side.
* The controller logic lives in `motion_states.py` with no ROS imports, so it is unit-tested with a simulated turtle (`test/`).

## 6. Design patterns 

* **State** – `motion_states.py`: `RotateToHeadingState`, `DriveState`, `RotateToYawState`, `DoneState` each hold one control law and decide the next state; `GoToPoseController` is the context. This replaces a long if/elif phase machine and makes adding a phase a one-class change. Behaviour is covered by `test_motion_states.py`.
* **Observer** – ROS pub/sub is itself the Observer pattern: `go_to_pose_server` subscribes to `/turtle1/pose` (subject: turtlesim), and the action feedback le]ts `square_action_server` observe the progress of `go_to_pose`, which it forwards as its own feedback.

## 7. Bag file 

```bash
# terminal 1: ros2 launch lab1_bringup system.launch.py
# terminal 2: record while the square runs
ros2 bag record -o square_bag /turtle1/cmd_vel /turtle1/pose
# terminal 3:
ros2 action send_goal /draw_square lab1_interfaces/action/DrawSquare "{side_length: 3.0}"
# Ctrl+C the recording, then:
ros2 bag info square_bag

# playback: reset first (a cmd_vel bag only repeats the path from the same start pose)
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 bag play square_bag --topics /turtle1/cmd_vel
```
_(Add a screenshot / note of your own run here.)_

## 8. Troubleshooting
| Problem | Fix |
|---|---|
| package / launch file not found | `source install/setup.bash` in that terminal; rebuild |
| goal rejected | target outside 0..11, or another goal is still running |
| `ros2 param set` fails with a type error | use a float: `0.5`, not `1` |
