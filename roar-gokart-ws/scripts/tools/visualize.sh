# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.
source install/setup.bash
ros2 run rviz2 rviz2 -d ./src/launches/launches/config/core/core.rviz --ros-args -p use_sim_time:=True