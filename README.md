# Smart Vision-Based Conveyor Sorting System

A ROS2-based industrial conveyor belt simulation featuring real-time color detection, PID-controlled belt speed, automated sorting into color-matched bins, jam detection, and 3D visualization in RViz. Built with C++, OpenCV, and Gazebo on Ubuntu 22.04.

---

## Demo

> RViz visualization showing the full sorting pipeline in action:
> - Colored box moving along the belt
> - Pusher arrow firing at the correct bin
> - Live bin counters updating
> - System status bar

![RViz Demo](docs/rviz_demo.png)

> Gazebo simulation showing the physical world — conveyor belt, camera, and color-coded bins:

![Gazebo World](docs/gazebo_world.png)

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Tech Stack](#tech-stack)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Project](#running-the-project)
- [Simulating a Jam](#simulating-a-jam)
- [ROS2 Topics](#ros2-topics)
- [Node Descriptions](#node-descriptions)
- [Concepts Demonstrated](#concepts-demonstrated)
- [Future Work](#future-work)

---

## Overview

This project simulates a real-world industrial conveyor sorting system using ROS2 as the middleware backbone. Objects (boxes) of different colors are spawned onto a conveyor belt and automatically sorted into color-matched bins using a computer vision pipeline.

A fourth **miscellaneous bin** at the end of the belt catches any unrecognized colors. A jam detection system monitors belt speed and triggers an alarm if the belt stalls. A simulated jam can be triggered at any time via a ROS2 topic.

The system runs entirely in software — no physical hardware required. Gazebo provides the 3D physics world, RViz provides real-time system monitoring, and all logic runs as ROS2 nodes in C++ and Python.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS2 Node Graph                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  [Gazebo World]                                                  │
│       │                                                          │
│       ▼                                                          │
│  [camera_node] ──/camera/image_raw──► [vision_node]              │
│                                            │                     │
│                                    /detected_color               │
│                                            │                     │
│                                            ▼                     │
│                                  [object_tracker_node]           │
│                                            │                     │
│                                    /object_position              │
│                                            │                     │
│                    ┌───────────────────────┘                     │
│                    ▼                                             │
│           [controller_node] ◄── /conveyor_speed                  │
│                    │                                             │
│          ┌─────────┼──────────┐                                  │
│          ▼         ▼          ▼                                  │
│   /conveyor_cmd /sorter_cmd /jam_alert                           │
│          │         │          │                                  │
│          ▼         ▼          ▼                                  │
│  [conveyor_node] [sorting_node] [jam_monitor_node]               │
│          │                      │                                │
│   /conveyor_speed          /jam_alert                            │
│                                                                  │
│  [spawn_boxes.py] ──/active_box──► [rviz_display_node]           │
│         │               └──────► [camera_node]                   │
│   /bin_counts ──────────────────► [rviz_display_node]            │
│                                                                  │
│  [rviz_display_node] ──/conveyor_markers──► [RViz2]              │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

### Belt Layout

```
SPAWN                                                        END
  │                                                           │
  ▼    [RED BIN]      [BLUE BIN]     [GREEN BIN]   [MISC BIN]
══╪═══════╪══════════════╪══════════════╪═══════════════╪════►
        X=-0.8         X=0.2         X=1.2           X=1.7
          ↑               ↑             ↑               ↑
        pusher          pusher        pusher        pusher
```

---

## Features

- **Color Detection** — HSV thresholding via OpenCV detects red, blue, and green objects in real time
- **PID Speed Control** — Custom PID controller maintains target belt speed with smooth ramp-up
- **Automatic Sorting** — Boxes are pushed sideways into matching color bins by a simulated actuator
- **Miscellaneous Bin** — Unknown colors (purple, yellow, orange, white) ride to the end of the belt and fall into a 4th bin
- **Jam Detection** — Monitors belt speed; triggers alarm if speed stays below threshold for 5+ seconds
- **Simulated Jam** — Trigger a belt freeze at any time via ROS2 topic; auto-clears after 8 seconds
- **Live RViz Dashboard** — 3D conveyor visualization with moving box marker, pusher arrows, bin counters, and jam status
- **Synchronized Camera Feed** — Camera feed shows the same box currently active on the belt
- **Gazebo World** — SDF-based 3D world with conveyor belt, camera sensor, and colored bins
- **Random Box Order** — Boxes spawn in random color order with ~25% chance of misc colors

---

## Tech Stack

| Layer | Technology |
|---|---|
| Middleware | ROS2 Humble |
| Simulation | Gazebo 11 |
| Visualization | RViz2 |
| Computer Vision | OpenCV 4 (via cv_bridge) |
| Language (nodes) | C++17 |
| Language (scripts) | Python 3.10 |
| Control | Custom PID controller |
| Build System | CMake + colcon |
| OS | Ubuntu 22.04 (WSL2) |

---

## Project Structure

```
ros2_conveyor_sorter/
└── src/
    └── conveyor_sort/
        ├── CMakeLists.txt
        ├── package.xml
        │
        ├── include/conveyor_sort/
        │   ├── pid_controller.hpp       # PID controller interface
        │   └── jam_detector.hpp         # Jam detection logic interface
        │
        ├── src/
        │   ├── pid_controller.cpp       # PID implementation
        │   ├── jam_detector.cpp         # Jam detection implementation
        │   ├── camera_node.cpp          # Simulated/Gazebo camera publisher
        │   ├── vision_node.cpp          # HSV color detection via OpenCV
        │   ├── object_tracker_node.cpp  # Position tracking + gate prediction
        │   ├── controller_node.cpp      # PID control + sorting trigger
        │   ├── conveyor_node.cpp        # Motor simulation with PID feedback
        │   ├── sorting_node.cpp         # Actuator/bin assignment
        │   ├── jam_monitor_node.cpp     # Speed-based jam detection
        │   └── rviz_display_node.cpp    # RViz MarkerArray publisher
        │
        ├── scripts/
        │   ├── spawn_boxes.py           # Gazebo box spawner + belt logic
        │   └── plot_speed.py            # Optional: live speed plotter
        │
        ├── msg/
        │   ├── DetectedObject.msg       # Color + bounding box
        │   ├── ObjectPosition.msg       # Belt position + gate flag
        │   └── JamAlert.msg             # Jam status + reason
        │
        ├── worlds/
        │   └── conveyor_world.sdf       # Gazebo world (belt, bins, camera)
        │
        ├── models/
        │   ├── conveyor_belt/           # Belt SDF model
        │   ├── colored_box/             # Box SDF model
        │   └── sorting_bin/             # Bin SDF model
        │
        ├── config/
        │   ├── pid_params.yaml          # PID gains
        │   └── vision_params.yaml       # HSV threshold values
        │
        ├── launch/
        │   ├── simulation.launch.py     # Full system with Gazebo
        │   └── full_system.launch.py    # Nodes only (no Gazebo)
        │
        └── rviz/
            └── conveyor_viz.rviz        # RViz configuration
```

---

## Prerequisites

- Ubuntu 22.04 (native or WSL2 on Windows)
- ROS2 Humble
- Gazebo 11
- Python 3.10+

---

## Installation

### 1. Install ROS2 Humble

Follow the official guide: https://docs.ros.org/en/humble/Installation.html

### 2. Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-rviz2 \
  python3-colcon-common-extensions \
  libopencv-dev
```

### 3. Clone the repository

```bash
mkdir -p ~/ros2_conveyor_sorter/src
cd ~/ros2_conveyor_sorter/src
git clone https://github.com/YOUR_USERNAME/ros2-conveyor-sorter.git conveyor_sort
```

### 4. Build

```bash
cd ~/ros2_conveyor_sorter
colcon build --symlink-install
source install/setup.bash
```

### 5. Add to bashrc (optional but recommended)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_conveyor_sorter/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### WSL2 display setup

If running on WSL2, add these to your `.bashrc`:

```bash
echo "export DISPLAY=:0" >> ~/.bashrc
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> ~/.bashrc
source ~/.bashrc
```

---

## Running the Project

### Option A — Full simulation (Gazebo + RViz)

```bash
ros2 launch conveyor_sort simulation.launch.py
```

Gazebo opens first (~10s startup), then RViz appears. Boxes begin spawning on the belt automatically.

### Option B — RViz only (no Gazebo, lighter)

```bash
ros2 launch conveyor_sort full_system.launch.py
```

All nodes run with simulated camera. RViz shows the full sorting pipeline without Gazebo overhead.

### Monitoring topics

```bash
# List all active topics
ros2 topic list

# Watch detected colors
ros2 topic echo /detected_color

# Watch conveyor speed
ros2 topic echo /conveyor_speed

# Watch sorting commands
ros2 topic echo /sorter_cmd

# Watch jam alerts
ros2 topic echo /jam_alert

# Watch bin counts
ros2 topic echo /bin_counts
```

---

## Simulating a Jam

While the simulation is running, open a second terminal and run:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub --once /simulate_jam std_msgs/msg/String "data: 'jam'"
```

What happens:
1. Belt freezes immediately — all boxes stop moving
2. RViz shows **"SIMULATED JAM! Belt Frozen"** in red
3. After 8 seconds, belt automatically resumes
4. Jam clears and next box spawns

---

## ROS2 Topics

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | camera_node | vision_node |
| `/detected_color` | `conveyor_sort/DetectedObject` | vision_node | object_tracker_node |
| `/object_position` | `conveyor_sort/ObjectPosition` | object_tracker_node | controller_node, jam_monitor_node |
| `/conveyor_cmd` | `std_msgs/Float32` | controller_node | conveyor_node |
| `/conveyor_speed` | `std_msgs/Float32` | conveyor_node | controller_node, jam_monitor_node |
| `/sorter_cmd` | `std_msgs/String` | controller_node | sorting_node |
| `/jam_alert` | `conveyor_sort/JamAlert` | controller_node, jam_monitor_node | rviz_display_node |
| `/active_box` | `std_msgs/String` | spawn_boxes.py | rviz_display_node, camera_node |
| `/bin_counts` | `std_msgs/String` | spawn_boxes.py | rviz_display_node |
| `/simulate_jam` | `std_msgs/String` | user (manual) | spawn_boxes.py |
| `/conveyor_markers` | `visualization_msgs/MarkerArray` | rviz_display_node | RViz2 |

---

## Node Descriptions

### `camera_node`
Publishes camera frames to `/camera/image_raw`. In simulation mode, renders a synthetic belt view showing the current active box with correct color, position, bin markers, and push direction arrow. Subscribes to `/active_box` to stay in sync with the spawner.

### `vision_node`
Converts BGR images to HSV color space and applies threshold masks to detect red, blue, and green objects. Finds the largest contour per color, computes bounding box and centroid, and publishes to `/detected_color`.

**HSV ranges:**
| Color | H min | H max | S min | V min |
|---|---|---|---|---|
| Red | 0 | 10 | 100 | 100 |
| Blue | 110 | 130 | 100 | 100 |
| Green | 50 | 70 | 100 | 100 |

### `object_tracker_node`
Receives detected object data and estimates position along the belt (in pixel space). Calculates estimated time to the sorting gate and sets the `approaching_gate` flag when the object is within 150px of the gate.

### `controller_node`
Core control node. Runs a PID loop at 10Hz to maintain target belt speed. Triggers sorting commands when an object approaches the gate. Monitors belt speed for sustained low-speed conditions (jam detection) after a 35-second startup grace period.

### `conveyor_node`
Simulates the belt motor. Receives speed commands and applies inertia simulation with small noise to produce realistic speed feedback. Publishes measured speed to `/conveyor_speed`.

### `sorting_node`
Receives color-based sorting commands and logs actuator actions. Prints which bin each object is directed to.

### `jam_monitor_node`
Independent jam watchdog. Monitors `/conveyor_speed` and fires a `JamAlert` if speed stays below 0.15 m/s for more than 5 continuous seconds. Ignores the first 35 seconds to allow system warmup.

### `rviz_display_node`
Publishes a `MarkerArray` at 10Hz containing all visual elements: conveyor belt, rails, bins, pusher arrows, moving box, bin count labels, total count, and jam status bar. Subscribes to multiple topics to stay synchronized with the full system state.

### `spawn_boxes.py`
Python node that manages the Gazebo lifecycle of boxes. Spawns one box at a time, tracks its position in Python, pushes it sideways into the correct bin using coordinate updates, counts boxes per bin, and publishes active box state for visualization. Supports jam simulation via `/simulate_jam` topic.

---

## Concepts Demonstrated

### Software Engineering
| Concept | Implementation |
|---|---|
| ROS2 pub/sub architecture | 10 nodes communicating via 12 topics |
| Custom message types | `DetectedObject`, `ObjectPosition`, `JamAlert` |
| Launch file system | Parameterized multi-node launch with timed startup |
| C++17 OOP | PID and JamDetector as reusable library classes |
| Python/C++ interop | Python spawner + C++ control nodes on same topics |

### Robotics & Control
| Concept | Implementation |
|---|---|
| PID control | Conveyor speed regulation with anti-windup |
| State machines | Box lifecycle: moving → pushing → done |
| Sensor fusion | Camera + speed sensor for jam detection |
| Actuator control | Simulated pneumatic pusher triggering |

### Computer Vision
| Concept | Implementation |
|---|---|
| Color space conversion | BGR → HSV for robust color detection |
| Image thresholding | HSV range masks per color |
| Contour detection | `findContours` for object localization |
| Centroid calculation | Moments-based centroid for tracking |

### Industrial Automation
| Concept | Implementation |
|---|---|
| Conveyor control | Speed regulation, start/stop logic |
| Sorting by property | Color-based bin assignment |
| Fault detection | Jam monitoring |
| Safety systems | Emergency stop on jam |
| Reject bin | Misc bin catches unclassified items |

---


## Future Work

- [ ] Multi-object tracking with unique IDs
- [ ] Real Gazebo camera integration (requires native Ubuntu or GPU passthrough)
- [ ] Web dashboard using `roslibjs` for browser-based monitoring
- [ ] Machine learning color classifier replacing HSV thresholds
- [ ] Migrate to ROS2 Iron / Jazzy + Gazebo Harmonic
- [ ] Hardware-in-the-loop testing with a Raspberry Pi camera
- [ ] URDF-based conveyor with proper joint controllers
- [ ] Action server interface for operator jam reset
