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

## :inbox_tray: Docker environment installation

- Configure and pull the Nvidia Isaac Sim Docker container, follow steps 1 to 6 of the [Nvidia instructions](https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_container.html#container-deployment).
- Build the Isaac/ROS 2 container:

```
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1
./docker/build.sh
```

- Run the container and continue with the ROS 2 build

```
./docker/run_isaac_docker.sh
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

[andino_isaac.webm](https://github.com/ekumenlabs/andino_isaac/assets/53065142/1802dc98-d8a0-4ee4-bbb5-df5590063b63)

