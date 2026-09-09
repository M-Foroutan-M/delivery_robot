# AUGDD ROS2 Simulation

Target environment: **Ubuntu 22.04 + ROS2 Humble + Gazebo Classic 11**.
(If you're on Jazzy/Gazebo Sim/Ignition instead, the diff_drive/lidar plugin
names differ slightly — flag it and I'll adapt.)

## Prereqs (run once)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  python3-colcon-common-extensions
pip install nav2_simple_commander --break-system-packages 2>/dev/null || true
```

## Build the workspace

```bash
cd ~/augdd_ws
colcon build --symlink-install
source install/setup.bash
```

---

## STEP 1 — Sanity-check the URDF (do this first)

```bash
ros2 launch augdd_description view_robot.launch.py
```

In RViz2:
1. Set **Fixed Frame** to `base_link`
2. Add a **RobotModel** display, set Description Topic to `/robot_description`
3. Use the joint_state_publisher_gui sliders to spin the wheels — confirm
   nothing looks stretched/detached and the chassis proportions look right.

If that renders a boxy 4-wheeled robot with a small cylinder (LiDAR) on top,
Step 1 is done. Tell me when it works (or paste any error) and I'll give you
Step 2: the Gazebo world + spawning the robot + teleop.

---

## Notes for the paper (methodology section)

- Chassis modeled as a single rigid box, 514×494×455mm envelope, 27kg
  (midpoint of report's 25–30kg target range) — acrylic doors/roof and
  internal tray structure omitted from the collision/inertial model since
  they don't affect navigation-relevant dynamics.
- Front wheels are the only driven joints (differential drive); rear wheels
  are free-spinning, low-friction casters. This is a standard simplification
  of 4WD skid-steer platforms for planar navigation simulation — document
  this explicitly as an assumption, since it's a legitimate limitation to
  name rather than hide.
- 2D LiDAR: 360° horizontal FOV, 10Hz, 0.12–10m range, Gaussian noise
  (σ=0.01m) added to approximate a real low-cost 2D LiDAR (this justifies
  using LiDAR-based SLAM/Nav2 even though your physical prototype only had
  ultrasonic sensors — the sim explores the "future work" LiDAR integration
  your report calls out).
