# COM760 CW2 — Setup & Run Guide (Group 5)

This guide takes you from a fresh laptop to a running Bug 2 simulation in
Gazebo. No prior Docker, ROS, or Linux experience is assumed.

The whole project runs inside a Docker container so we don't have to install
ROS Noetic, Gazebo, or any of the Python dependencies on the host machine.
You'll edit files normally on your laptop with whatever editor you like, and
they appear instantly inside the container.

---

## 1. What you're getting set up

- **Docker Desktop** — runs the Linux/ROS environment in a container
- **The repo** — contains the ROS package (`com760cw2_group5`), the
  Dockerfile, and a `docker-compose.yml` that ties it all together
- **A browser-based desktop (noVNC)** — so you can see Gazebo and RViz GUIs
  even though they're running inside a Linux container

You will end up doing all your work in two places:
1. Your **normal code editor on the host** (VS Code, etc.) for editing files
2. A **browser tab** at `http://localhost:8080/vnc.html` to see the simulation

---

## 2. Install Docker Desktop


1. Go to https://www.docker.com/products/docker-desktop/
2. Download the build
3. launch it
4. Accept the default settings. Wait until the Docker whale icon in the menu
   bar stops animating — that means the engine is running.


### Verify Docker works
Open a terminal and run:
```bash
docker --version
docker compose version
```
Both should print a version. If they error, Docker isn't running yet — open
Docker Desktop and wait for it to start.

---

## 3. Get the repo

```bash
git clone:
```

The folder structure you'll see:
```
.
├── docker/                 # Dockerfile + VNC startup scripts
├── docker-compose.yml      # one-command container management
├── src/
│   └── com760cw2_group5/   # our ROS package (this is where the code lives)
└── SETUP.md                # this file
```

The `src/` folder is **mounted into the container**, so anything you edit on
the host (e.g. `src/com760cw2_group5/scripts/bug2.py`) is immediately visible
inside the running container at `/ros_ws/src/com760cw2_group5/scripts/bug2.py`.
You don't need to rebuild Docker when you change code.

---

## 4. Build and start the container

From the project root (where `docker-compose.yml` lives):

```bash
docker compose up --build
```

**First run only:** this will download the ROS Noetic base image and install
Gazebo, RViz, VNC, and a bunch of ROS packages. Expect 5–15 minutes depending
on your internet. You'll see lots of `apt-get` output. This is normal.

When it's done you'll see a banner like:
```
╔══════════════════════════════════════════════════════╗
║   COM760 CW2 — ROS Noetic + Gazebo Ready            ║
║                                                      ║
║   Open in browser:                                   ║
║   http://localhost:8080/vnc.html                     ║
║                                                      ║
║   VNC password: ros_docker                           ║
╚══════════════════════════════════════════════════════╝
```

**Leave that terminal alone** — closing it stops the container. Open a new
terminal for everything else from here on.

### Subsequent runs
You only need `--build` if the `Dockerfile` itself has changed. Normal startup:
```bash
docker compose up
```

To run it in the background instead of tying up a terminal:
```bash
docker compose up -d
```

To stop:
```bash
docker compose down
```

---

## 5. Open the browser desktop

In your web browser go to:

> **http://localhost:8080/vnc.html**

Click **Connect**, enter the password **`ros_docker`**, and you should see an
empty XFCE desktop. This is the Linux desktop running inside the container —
you'll launch Gazebo and RViz from terminals on this desktop.

If the page doesn't load, check that:
- The container is still running (`docker ps` should show `com760cw2`)
- Nothing else on your machine is using port 8080

---

## 6. Open a shell inside the container

You need to be inside the container to run ROS commands. From your host
terminal:

```bash
docker exec -it com760cw2 bash
```

You'll get a prompt like `root@<id>:/ros_ws#`. From here you have full ROS
Noetic available. Useful checks:
```bash
echo $ROS_DISTRO            # → noetic
rospack list | head -5      # ROS sees its packages
ls /ros_ws/src              # → com760cw2_group5  (your code, mounted from host)
```

You can open multiple shells into the same container by running
`docker exec -it com760cw2 bash` again in another host terminal — you'll need
at least two (one for `roslaunch`, one for inspecting topics, etc.).

> **Tip:** the workspace is auto-built when the container starts. If you
> change Python files, you don't need to rebuild — they're picked up live.
> If you change `CMakeLists.txt`, `package.xml`, custom messages
> (`.msg`/`.srv`), or the URDF, you do need to rebuild:
> ```bash
> cd /ros_ws && catkin_make && source devel/setup.bash
> ```

---

## 7. Run the Bug 2 simulation

Inside a container shell:

```bash
cd /ros_ws && catkin_make

roslaunch com760cw2_group5 bug2_navigation.launch
```

This single command:
1. Starts Gazebo with our custom world (`group5.world`)
2. Spawns `group5Bot` (our robot, with a laser scanner) at the spawn pose
3. Starts the three navigation nodes: `bug2.py`, `GoToPoint.py`, `FollowWall.py`
4. Sets the goal coordinates as ROS params

To stop the simulation: `Ctrl+C` in the launch terminal.

This will setup the simulation however to unpause the scene (it is naturally paused) you need to send
the following message in a terminal window
```bash
rosservice call /gazebo/unpause_physics
```

### Common variations
Override the goal at launch time:
```bash
roslaunch com760cw2_group5 bug2_navigation.launch goal_x:=2.0 goal_y:=-1.5
```

---

## 8. Useful inspection commands

Run these in a second container shell while the simulation is running.

```bash
# What topics exist?
rostopic list

# Watch the robot's status (custom message we publish)
rostopic echo /group5Bot/status

# Watch what the navigation is commanding
rostopic echo /group5Bot/cmd_vel

# See the laser scan data
rostopic echo /group5Bot/laser/scan

# See all the nodes that are running and how they're connected
rqt_graph        # opens a GUI in the noVNC desktop
```

---

## 9. Editing code

Just open the project folder on your **host** in VS Code (or whatever).
Anything under `src/com760cw2_group5/` is the code. The files you'll touch
most:

| File | What it does |
|---|---|
| `scripts/bug2.py` | The state machine — when to go-to-point vs wall-follow |
| `scripts/GoToPoint.py` | Drives the robot in a straight line toward a goal |
| `scripts/FollowWall.py` | Right-hand-rule wall-following behaviour |
| `urdf/group5Bot.urdf` | The robot model (incl. laser scanner) |
| `world/group5.world` | The Gazebo simulation environment |
| `launch/bug2_navigation.launch` | Wires everything together at launch |
| `msg/RobotStatus.msg` | Custom status message (state, position, wall hits) |
| `srv/SetGoal.srv` | Custom service to change goal at runtime |
| `srv/SwitchBehavior.srv` | Custom service to enable/disable a behaviour |

After editing a `.py` file → just `Ctrl+C` and re-`roslaunch`.
After editing `.msg`/`.srv`/`CMakeLists.txt`/URDF → rebuild:
```bash
cd /ros_ws && catkin_make && source devel/setup.bash
```

---

## 10. Troubleshooting

**"Cannot connect to the Docker daemon"**
Docker Desktop isn't running. Open it, wait for the engine to start.

**Port 8080 already in use**
Something else on your machine is using it. Either kill that process, or edit
`docker-compose.yml` and change `"8080:8080"` to e.g. `"8090:8080"`, then visit
`http://localhost:8090/vnc.html`.

**noVNC connects but the desktop is black**
The X server inside the container hasn't started cleanly. Restart the
container:
```bash
docker compose down
docker compose up
```

**`roslaunch` says it can't find the package**
The workspace probably wasn't sourced. Run:
```bash
source /ros_ws/devel/setup.bash
```
Or if the package was added since the container started, rebuild first:
```bash
cd /ros_ws && catkin_make && source devel/setup.bash
```

**The robot spawns but doesn't move**
Gazebo is paused. Click the play button (▶) at the bottom-left of the Gazebo
window in the noVNC desktop.

**"No such file or directory: bug2.py"**
The Python scripts need to be executable. From inside the container:
```bash
chmod +x /ros_ws/src/com760cw2_group5/scripts/*.py
```

---

## 11. Quick reference cheat-sheet

```bash
# Host machine
docker compose up -d                     # start container in background
docker compose down                      # stop container
docker exec -it com760cw2 bash           # open a shell inside
docker logs -f com760cw2                 # tail the container output

# Inside the container
cd /ros_ws && catkin_make                # rebuild after C++/msg/urdf changes
source devel/setup.bash                  # source the workspace
roslaunch com760cw2_group5 bug2_navigation.launch
rostopic list                            # see all topics
rostopic echo /group5Bot/status          # watch status messages
rosservice call /set_goal "x: 1.0
y: 2.0"
```

**Browser desktop:** http://localhost:8080/vnc.html — password `ros_docker`
