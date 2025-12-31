# PyBullet + ROS 2 Simulation (Jazzy)

This project integrates PyBullet with ROS 2 Jazzy to simulate a mobile robot with:

- Real-time physics
- Camera publishing `/camera/image_raw`
- Odometry publishing `/odom`
- TF tree (`odom → base_link → camera_link`)
- Keyboard teleoperation via `/cmd_vel`
- RViz visualization

It behaves like a real ROS 2 robot, but fully simulated.

---

## How the System Works

### 1. PyBullet Simulation Node (`pybullet_node`)
This node:

- Starts PyBullet in GUI mode
- Loads the robot and world
- Steps the physics engine
- Publishes:
  - `/odom`
  - `/camera/image_raw`
  - TF transforms
- Subscribes to `/cmd_vel` to move the robot

---

### 2. Keyboard Teleoperation
This node publishes velocity commands on `/cmd_vel` when you press keys.

---

### 3. RViz Visualization
RViz shows:

- TF frames
- Odometry
- Camera image
- Robot movement

---

##  Running the Simulation

Open **three terminals** and source ROS 2 + your workspace in each:

```bash
source /opt/ros/jazzy/setup.bash
source ~/projects/ros2_pybullet_ml_project/ros2_ws/install/setup.bash
