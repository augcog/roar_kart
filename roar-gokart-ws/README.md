<!--
 Copyright 2023 michael. All rights reserved.
 Use of this source code is governed by a BSD-style
 license that can be found in the LICENSE file.
-->

# ROAR Indy WS

This repo is the parent repo for the ROAR Go-Kart project. 

## Quick Start
- Prereqs:
  - Install ROS2 Foxy
    - https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
  - [Optional] if using Carla simulator, install `carla-ros-bridge` for ros2
    - https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/
  - [Optional] If using hardware, make sure to install corresponding SDKs. More documentation provided after cloning, in each respective repo's README
  - [Optional] To contribute please install commit linters
    - upgrade NodeJS: https://blog.hubspot.com/website/update-node-js
    - Install linter: https://github.com/legend80s/git-commit-msg-linter

Assuming you are at `roar-gokart-ws`

- For ROAR launches
`vcs import src/ < repos/launches.repos --recursive`

- For Core: 
`vcs import src/ < repos/core.repos --recursive`

- For simulation  
`vcs import src/ < repos/simulation.repos --recursive`

- For hardware
 `vcs import src/ < repos/gokart.repos --recursive`



- Installation
- `rosdep install --from-paths src --ignore-src -r -y`
- `colcon build`

- Usage
  - Simulation:
    - `./scripts/carla/launch_simulation_server.sh`
    - `./scripts/carla/racing/client_auto.sh`


# FAQ
- `rapidjson` not found
`sudo apt-get install rapidjson-dev`



## Some quick functions
1. To record everything in Carla
```
ros2 launch roar-indy-launches roar.launch.py manual_control:=True carla:=True record:=True
```
2. To record waypoint in Carla

```
ros2 launch roar-indy-launches roar.launch.py manual_control:=True carla:=True should_record_waypoint:=True
```

3. to check what options are available
```
ros2 launch roar-indy-launches roar.launch.py -s
```

4. Manual control in Carla
```
ros2 launch roar-indy-launches roar.launch.py manual_control:=True carla:=True
```
5. Auto waypoint following in Carla
```
ros2 launch roar-indy-launches roar.launch.py carla:=True core:=True visualization:=True param_file:=./src/launches/launches/config/carla/config.yaml
```
Use RQT or use cmd to invoke
```
ros2 service call /controller/manager/safety_toggle roar_msgs/srv/ToggleControlSafetySwitch "{is_safety_on: True}"
```