# Andino simulation in Nvidia Isaac

<img src=docs/andino_isaac.png width=700/>

This package provides a simulation environment for [Andino](https://github.com/Ekumen-OS/andino) in [Nvidia's Isaac sim](https://www.nvidia.com/en-us/deep-learning-ai/industries/robotics/) integrated with ROS 2 and powered by [Nvidia Omniverse](https://www.nvidia.com/en-us/omniverse/).

## :clamp: Platforms

- OS:
  - Ubuntu 22.04 Jammy Jellyfish or Ubuntu 24.04 Noble Numbat
- ROS 2 Jazzy (optional if the user wants to test ROS 2 integration)

## :inbox_tray: Installation

Clone this repository:

```sh
git clone https://github.com/ekumenlabs/andino_isaac.git
cd andino_isaac
```

Source the environment and build the Docker image:

```sh
source setup.bash
andino_isaac build
```

## :rocket: Usage

Launch the simulation with the following command, this will use the default environment:

```sh
andino_isaac run
```

Check all the available options with `andino_isaac --help`.

### Teleoperate the robot

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

[andino_isaac.webm](https://github.com/ekumenlabs/andino_isaac/assets/53065142/1802dc98-d8a0-4ee4-bbb5-df5590063b63)

## :computer: Development

For development in Isaac sim a devcontainer file is provided. This devcontainer enables Python analysis for Isaac sim code while inside the container, making it easier for the user to work with the API. To use it, open the repository directory with VSCode and click on "reopen in container" when prompted at the bottom right.

This opens a VSCode window inside the Isaac sim container with the tools enabled. The container also has GUI and GPU capabilities and Isaac sim installed in the `/isaac-sim` directory. To open Isaac sim inside this devcontainer run:

```sh
./isaac-sim/isaac-sim.sh
```
