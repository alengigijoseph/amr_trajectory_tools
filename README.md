# ROS2 Package for Robot Trajectory Recording and Visualization

## Overview
This package provides two ROS2 nodes for recording, saving, and visualizing a robot's trajectory. The nodes are provided as both C++ and python executables.

### **1. TrajectorySaver (`trajectory_saver`)**
- Subscribes to pose data and records the robot's trajectory.
- Allows customization of the pose frame using the `--frame` argument (default: `odom`).
- Saves trajectory data as a JSON file upon request.
- Publishes trajectory markers for visualization.

### **2. TrajectoryReader (`trajectory_reader`)**
- Reads a saved trajectory JSON file.
- Republishes trajectory markers for visualization.

---

## Installation and Usage
Ensure ROS2 is installed and sourced before running the nodes.

### **Clone the package into your project src folder**
```sh
cd ~/your_workspace/src
git clone https://github.com/alengigijoseph/amr_trajectory_tools.git
```
### **Build and src your workspace**
```sh
cd ..
colcon build
source install/setup.bash
```
Make sure your robot is publishing odometry data before running the nodes.

### **Running the TrajectorySaver Node C++**
```sh
ros2 run amr_trajectory_tools trajectory_saver
```
### **Running the TrajectorySaver Node Python**
```sh
ros2 run amr_trajectory_tools trajectory_saver.py
```

### **Saving the Trajectory**
Call the service to save the recorded trajectory:
```sh
ros2 service call /save_trajectory amr_trajectory_tools/srv/SaveTrajectoryMsg "{filename: 'trajectory1', duration: 10.0}"
```
- **`filename`**: Name of the file to save the trajectory.  
- **`duration`**: The duration (in seconds) for which data should be recorded.  

### **Running the TrajectoryReader Node C++**
```sh
ros2 run amr_trajectory_tools trajectory_reader
```

### **Running the TrajectoryReader Node Python**
```sh
ros2 run amr_trajectory_tools trajectory_reader.py
```

### **Loading and Visualizing a Saved Trajectory**
```sh
ros2 service call /read_trajectory amr_trajectory_tools/srv/ReadTrajectoryMsg "{filename: '/path/to/file/trajectory1.json'}"
```
- **`filename`**: Path to the saved trajectory JSON file.  

---

## How It Works

### **1. Trajectory Collection (`trajectory_saver`)**
1. Initializes the ROS2 node and sets parameters.  
2. Subscribes to the `/odom` topic (default) to receive pose data.  
   - The topic can be changed using `--pose_topic`.  
3. Records pose data at a configurable frequency (default: `5.0 Hz`, adjustable via `--record_frequency`).  
4. Publishes visualization markers at each recorded point.  
   - The marker's reference frame defaults to `odom`, but can be changed using `--frame`.  
5. Provides a `/save_trajectory` service to save the recorded trajectory:  
   - Filters trajectory points within the requested duration.  
   - Converts quaternion orientation to yaw.  
   - Saves data in JSON format.  

### **2. Trajectory Storage Format (JSON)**
Each saved trajectory follows this format:
```json
[
  { "x": 1.0, "y": 2.0, "yaw": 0.5 },
  { "x": 1.2, "y": 2.1, "yaw": 0.6 }
]
```

### **3. Trajectory Visualization (`trajectory_reader`)**
1. Initializes the ROS2 node and sets parameters.  
2. Provides a `/read_trajectory` service to load a saved trajectory.  
3. Upon request:  
   - Reads the JSON file and extracts trajectory points.  
   - Publishes markers for visualization (`MarkerArray`).  
   - The reference frame defaults to `odom`, but can be modified using `--frame`.  
4. Each trajectory point is visualized as a small sphere.  

---

## Customization
You can adjust the following parameters:

| Parameter | Default Value | Description |
|-----------|-------------|-------------|
| `--record_frequency` | `5.0` Hz | Frequency of trajectory recording. |
| `--pose_topic` | `/odom` | Topic to subscribe for pose data. |
| `--marker_topic` | `/trajectory_markers` | Topic for publishing visualization markers. |
| `--frame` | `odom` | Reference frame for trajectory markers. |
| **Marker size/color** | - | Modify `publish_markers()` in `trajectory_saver.py`. |

**Note**: Ensure that `trajectory_saver` and `trajectory_reader` do not publish to the same marker topic.

---

## Dependencies
The following ROS2 messages and services are required:

### Required ROS2 Packages
- `rclcpp` – ROS2 C++ client library for developing nodes.
- `nav_msgs` – Provides navigation-related messages, including `Path`.
- `visualization_msgs` – Contains message types for visualization in RViz.
- `jsoncpp` – Library for parsing and manipulating JSON data.


### Custom Service Messages
This package defines and uses the following custom service messages:

- `srv/SaveTrajectoryMsg.srv`
- `srv/ReadTrajectoryMsg.srv`

These package also depend on `nav_msgs` and `visualization_msgs`.

## Issues
### **Json.h is not found**
```sh
sudo apt update
sudo apt install libjsoncpp-dev
```
run the following to locate json.h 

```sh
find /usr -name json.h 2>/dev/null //
```
by default json.h will be inside /usr/include/jsoncpp, if it is elsewhere update the path in CMakeLists.txt -> include_directories(/usr/include/jsoncpp) 



---

