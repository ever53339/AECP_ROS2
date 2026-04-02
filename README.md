# ROS 2 Implementation of AECP — Autonomous Elemental Characterization Platform
This repository contains the ROS 2 action servers and Behavior Tree implementations of the autonomous elemental characterization platform. 
This is one component of the software stack for the paper ["Autonomous elemental characterization enabled by a low cost robotic platform built upon a generalized software architecture"](https://pubs.rsc.org/en/content/articlehtml/2026/dd/d5dd00263j).

# Environments
- Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- ROS 2 Humble
- Python 3.10

# Usage
1. Clone this repo to your ROS 2 workspace
2. Install dependencies

```
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```
3. Build
```
colcon build
```
 Start use the packages

