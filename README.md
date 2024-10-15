# jessie_bringup

Package for starting components on Jessie

## Prerequisites

* This package uses ROS2, so this should be installed on the robot first.
* The ROS2 interfaces for the xArm should also be set up on the robot. The components, along with instructions for setting them up, can be found at [`xarm_ros2`](https://github.com/xArm-Developer/xarm_ros2).

## Usage

The bringup is expected to run on the robot itself.

To use it, first, clone this repository into a ROS workspace:
```
cd /path/to/ws/src
git clone https://github.com/a2s-institute/jessie_bringup.git
```

Then, build the package and (re)source the workspace:
```
colcon build --packages-select jessie_bringup
source /path/to/ws/install/setup.bash
```

Finally, run the robot components:
```
ros2 launch jessie_bringup jessie.launch.py
```

## Conventions

### Robot left / right arm

In the bringup, we launch controllers for both arms on Jessie. For defining the left / right arm, we follow a robot-centric convention (i.e. left / right from the point of view of the robot):
* The arm with IP `192.168.1.204` is considered to be the left arm
* The arm with IP `192.168.1.209` is considered to be the right arm

### Namespaces corresponding to the arms

The components corresponding to the different arms are under dedicated namespaces as follows:
* For the left arm: `/left_arm_xarm/[topic/service]`
* For the right arm: `/right_arm_xarm/[topic/service]`