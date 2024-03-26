# Andino simulation in Nvidia Isaac

<img src=docs/andino_isaac.png width=700/>

This package provides a simulation environment for [Andino](https://github.com/Ekumen-OS/andino) in [Nvidia's Isaac sim](https://www.nvidia.com/en-us/deep-learning-ai/industries/robotics/) integrated with ROS 2 and powered by [Nvidia Omniverse](https://www.nvidia.com/en-us/omniverse/).

## :clamp: Platforms

- ROS 2: Humble Hawksbill
- OS:
  - Ubuntu 22.04 Jammy Jellyfish
- Isaac Sim 2023.1.1

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


### Teleoperate the robot

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


