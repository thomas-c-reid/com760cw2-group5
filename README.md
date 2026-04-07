# COM760 Robotics & AI — Coursework 2 (Group 5)

A ROS Noetic package implementing autonomous robot navigation with the
**Bug 2 algorithm** in a simulated Gazebo environment. The robot navigates
from a start pose to a goal using only local laser data and a global target
position — no map required.

Built for the COM760 group project (Ulster University, 2025/26).

---

## What it does

`group5Bot` — a differential-drive robot with a 360° laser scanner — is
dropped into a Gazebo world containing static obstacles (jersey barriers,
perimeter walls). Given a goal coordinate, it:

1. **Drives straight toward the goal** (`GoToPoint`)
2. **Detects obstacles** via laser and switches to wall-following when the
   path is blocked (`FollowWall`, right-hand rule)
3. **Resumes heading to the goal** once it re-crosses the straight-line
   *M-line* between the start and the target, closer than where it left it
4. **Stops** when it arrives within 0.3 m of the goal

The three behaviours are coordinated by a central state machine (`bug2.py`)
and can be enabled/disabled at runtime through custom ROS services.

---

## Stack

- **ROS Noetic** on Ubuntu 20.04 (runs in Docker on any host OS)
- **Gazebo 11** for physics simulation
- **Python 3** for all nodes
- **noVNC** browser desktop so the Gazebo/RViz GUIs work without needing an
  X server on the host

---

## Package layout

```
src/com760cw2_group5/
├── scripts/
│   ├── bug2.py              # state machine / orchestrator
│   ├── GoToPoint.py         # drives toward a goal point
│   └── FollowWall.py        # right-hand-rule wall follower
├── msg/
│   └── RobotStatus.msg      # custom message (state, position, wall hits)
├── srv/
│   ├── SetGoal.srv          # custom service: set a new goal at runtime
│   └── SwitchBehavior.srv   # custom service: enable/disable a behaviour
├── urdf/
│   └── group5Bot.urdf       # robot model (chassis, wheels, laser)
├── world/
│   └── group5.world         # Gazebo world with static obstacles
├── launch/
│   └── bug2_navigation.launch
├── CMakeLists.txt
└── package.xml
```

Per the spec, the robot and its topics use the group ID:
- Robot name: `group5Bot`
- Laser topic: `/group5Bot/laser/scan`
- Cmd_vel topic: `/group5Bot/cmd_vel`
- Status topic: `/group5Bot/status`

---

## Getting started

The full setup walkthrough — from installing Docker to running the
simulation — lives in **[SETUP.md](SETUP.md)**. That guide assumes no prior
Docker/ROS experience.

Short version for anyone who already has Docker installed:
```bash
docker compose up --build                              # first run
docker exec -it com760cw2 bash                         # shell into container
roslaunch com760cw2_group5 bug2_navigation.launch      # run the sim
```
Then open http://localhost:8080/vnc.html (password `ros_docker`) to see
Gazebo.

---
