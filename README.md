# :green_heart: Andino simulation in Nvidia Isaac

This package provides a simulation environment for [Andino](https://github.com/Ekumen-OS/andino) in [Nvidia's Isaac sim](https://www.nvidia.com/en-us/deep-learning-ai/industries/robotics/) integrated with ROS 2.

## :clamp: Platforms

- ROS 2: Humble Hawksbill
- OS:
  - Ubuntu 22.04 Jammy Jellyfish
- Isaac Sim 2023.1.0

## :inbox_tray: Installation

Clone this repository into the src folder of a ROS2 workspace

```
git clone https://github.com/ekumenlabs/andino_isaac.git
```

## :package: Build

The package contains some dependencies that must be installed in order to build it:

```
rosdep install --from-paths src -i -y
```

Then build the package and source the install workspace. To do so run the following commands:

```sh
colcon build
source install/setup.bash
```

## :rocket: Usage

Launch the simulation with the provided launchfile

```
ros2 launch andino_isaac andino_isaac.launch.py
```

2. Enjoy!:
  - Teleop the robot