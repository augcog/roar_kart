# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

source install/setup.bash
ros2 launch roar-indy-launches roar.launch.py manual_control:=True carla:=True params_file:=./src/launches/launches/config/carla/config.yaml