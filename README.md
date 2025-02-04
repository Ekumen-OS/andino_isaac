# Andino simulation in Nvidia Isaac

<img src=docs/andino_isaac.png width=700/>

This package provides a simulation environment for [Andino](https://github.com/Ekumen-OS/andino) in [Nvidia's Isaac sim](https://www.nvidia.com/en-us/deep-learning-ai/industries/robotics/) integrated with ROS 2 and powered by [Nvidia Omniverse](https://www.nvidia.com/en-us/omniverse/).

## :clamp: Platforms

- ROS 2: Humble Hawksbill
- OS:
  - Ubuntu 22.04 Jammy Jellyfish
- Isaac Sim 4.5.0 (optional if not using Docker)

## :inbox_tray: Installation

Clone this repository into the src folder of a ROS2 workspace

```sh
git clone https://github.com/ekumenlabs/andino_isaac.git
```

## :inbox_tray: (Optional) Docker environment installation

This is an optional step. If not using Docker go to the [Build](#package-build) section.

- Configure and pull the Nvidia Isaac Sim Docker container, follow steps of the [Nvidia instructions](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_container.html) up to pulling the 4.5.0 image (Step 3 of the [Container Deployment](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_container.html#container-deployment) section).
- Build and run the Isaac/ROS 2 container:

```sh
./docker/run.sh
```

Note: If you need to re-build the image pass the `--build` flag to the above command.

## :package: Build

The package contains some dependencies that must be installed in order to build it:

```sh
rosdep install --from-paths src -i -y
```

Then build the package and source the install workspace. To do so run the following commands:

```sh
colcon build
source install/setup.bash
```

## :rocket: Usage

Launch the simulation with the provided launchfile

```sh
ros2 launch andino_isaac andino_isaac.launch.py
```

### Teleoperate the robot

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

[andino_isaac.webm](https://github.com/ekumenlabs/andino_isaac/assets/53065142/1802dc98-d8a0-4ee4-bbb5-df5590063b63)
