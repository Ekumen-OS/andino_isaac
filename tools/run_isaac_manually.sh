#!/bin/bash
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/$(whoami)/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.isaac.ros2_bridge/humble/lib


/home/$(whoami)/.local/share/ov/pkg/isaac_sim-2023.1.1/isaac-sim.sh

