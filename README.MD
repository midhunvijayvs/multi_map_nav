# 🌀 Multi-Map Wormhole Navigation for ROS

https://github.com/midhunvijayvs/multi_map_nav.git

This ROS package, `multi_map_nav`, implements a **teleportation-style navigation system** for robots operating across multiple maps using *wormholes*. It features:

- SQLite-based wormhole database
- A ROS Action Server for navigation
- RViz visualization with directional arrows and labels
- TF broadcasting for map transitions
- A sample whomehole navigation client

---

## 📁 Package Contents

Multi_map_nav/
├── action/ # Custom WormholeJump.action
├── include/multi_map_nav/ # Header files
├── launch/ # Launch files
├── msg/ # (Optional) ROS messages
├── src/ # Core C++ code (action server, client, marker publishers)
├── wormholes.db # SQLite DB storing wormhole map coordinates
├── CMakeLists.txt
└── package.xml


---

## ⚙️ Dependencies

### ROS packages (add to `package.xml` and `CMakeLists.txt`):

- `roscpp`
- `actionlib`
- `actionlib_msgs`
- `geometry_msgs`
- `std_msgs`
- `tf`
- `visualization_msgs`
- `move_base_msgs`
- `roslib`
- `robot_state_publisher`
- `rviz`

### System dependencies:
- `sqlite3` (already used in C++)

Install missing packages (Ubuntu):
```bash
sudo apt-get install ros-${ROS_DISTRO}-<package-name>


### 🚀 How to Run

1. Build the workspace

    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash

2. Initialize the Wormhole DB

    rosrun multi_map_nav wormhole_db_initializer

    This will create wormholes.db with example teleport points.


3. Launch the full system

    roslaunch multi_map_nav multi_map_nav.launch

This includes:

    wormhole_action_server

    tf_broadcaster 

    wormhole_marker_publisher

    RViz for visualization


    A robot simulation also added to see the jump visually in the rviz after 2 seconds the client action run. the client will be run automatically with launch code and can see the sample jump.

4. Send a wormhole jump mannually using command command

    edit wormholes and then
    rosrun multi_map_nav navigate_wormhole_client