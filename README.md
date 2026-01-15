# Fra2mo and iiwa Cooperative System

ROS 2 project for the Robotic Lab course.

Fra2mo explores the environment, detects green objects, transports them to a drop-off position, and triggers the iiwa robot to perform a pick-and-place task.

---

## Setup

Install dependencies:

```bash
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## Run

Launch in two separate terminals, in this order:

**Terminal 1 – Robots Bringup**

```bash
ros2 launch ros2_fra2mo robots_bringup.launch.py
```

**Terminal 2 – Cooperative System**

```bash
ros2 launch ros2_fra2mo cooperative_system.launch.py
```
