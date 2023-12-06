# andino_isaac

## Requirements
- A docker environment is yet to be implemented, so it can only be run locally
- ROS2 Humble
- Tested with Isaac Sim 2022.2.1 and 2023.1.0

## Usage

1. Launch Isaac Sim

2. One time setup, skip if already done:
  - In the w
  - Enable ROS 2 Bridge extension

3. Open USDA andino description.

4. Launch ROS 2 andino_description

  ```
  ros2 launch andino_description andino_description.launch.py
  ```

5. Play the simulation

6. Enjoy!:
  - Teleop the robot
  - Launch rviz and see TF
