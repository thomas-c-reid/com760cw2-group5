# COM760 CW2 - Group 5: Bug 2 Navigation

Bug 2 autonomous navigation for a differential-drive robot in Gazebo using ROS Noetic.

## How to Run

```bash
# Build the package
cd ~/catkin_ws && catkin_make

# Launch (starts Gazebo, spawns robot, runs Bug 2)
roslaunch com760cw2_group5 bug2_navigation.launch

# Optional: reset the robot to start position
rosservice call /reset_bug2 "{}"

# Optional: set a new goal
rosservice call /set_goal "{x: 3.0, y: 2.0}"

# Optional: change speed or wall-follow direction
rosservice call /set_bug_status "{flag: true, speed: 0.5, direction: 'left', message: ''}"
```

## Package Structure

- `scripts/bug2.py` - Bug 2 state machine (go-to-point, follow-wall, goal-reached)
- `scripts/GoToPoint.py` - Point-to-point navigation with proportional control
- `scripts/FollowWall.py` - Right-hand-rule wall following using laser regions
- `scripts/reset_run.py` - Convenience script to reset the simulation
- `msg/RobotStatus.msg` - Custom message publishing robot state at 10Hz
- `srv/SetBugStatus.srv` - Runtime control of speed, direction, and enable/disable
- `srv/SetGoal.srv` - Set a new navigation goal
- `srv/SwitchBehavior.srv` - Activate/deactivate GoToPoint and FollowWall nodes
- `urdf/group5Bot.urdf` - Differential-drive robot with 360-degree laser scanner
- `world/group5.world` - Obstacle course with jersey barriers
- `launch/bug2_navigation.launch` - Single launch file for the full system

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/group5Bot/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/group5Bot/laser/scan` | `sensor_msgs/LaserScan` | 360-sample laser data |
| `/group5Bot/status` | `RobotStatus` | Robot state, pose, metrics |

## ROS Services

| Service | Type | Description |
|---------|------|-------------|
| `/set_goal` | `SetGoal` | Set navigation target (x, y) |
| `/set_bug_status` | `SetBugStatus` | Control speed, direction, enable/disable |
| `/reset_bug2` | `std_srvs/Trigger` | Reset robot to start position |
| `/go_to_point_switch` | `SwitchBehavior` | Toggle GoToPoint behavior |
| `/wall_follower_switch` | `SwitchBehavior` | Toggle FollowWall behavior |
