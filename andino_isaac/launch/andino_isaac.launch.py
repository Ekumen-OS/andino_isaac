import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    # Arguments
    world_name = DeclareLaunchArgument(
        'world_name',
        default_value='plain_world.usda',
        description='Name of the world to launch',
    )
    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='andino.usda',
        description='Name of the robot to spawn',
    )
    headless = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Isaac sim headless',
    )
    renderer = DeclareLaunchArgument(
        'renderer',
        default_value='RayTracedLighting',
        description='Renderer to use, either RayTracedLighting or PathTracedLighting',
    )
    verbose = DeclareLaunchArgument(
        'verbose',
        default_value='True',
        description='Show Isaac sim output',
    )

    # Paths to places
    pkg_andino_isaac_path = get_package_share_directory('andino_isaac')
    user_home_path = os.path.expanduser("~")
    isaac_install_path = os.path.join(user_home_path, ".local/share/ov/pkg/isaac_sim-2023.1.0")
    isaac_python_launcher_path = os.path.join(isaac_install_path, "python.sh")
    isaac_custom_launch_script = os.path.join(pkg_andino_isaac_path, "tools", "isaac_launch_script.py")
    full_path_to_world = PathJoinSubstitution([pkg_andino_isaac_path, 'isaac_worlds', LaunchConfiguration('world_name')])
    full_path_to_robot = PathJoinSubstitution([pkg_andino_isaac_path, 'andino_isaac_description', LaunchConfiguration('robot_name')])

    # Environment variables
    prev_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_library_path_env_var = prev_ld_library_path + ":" + user_home_path + "/.local/share/ov/pkg/isaac_sim-2023.1.0/exts/omni.isaac.ros2_bridge/humble/lib"
    rmw_implementation_env_var = 'rmw_fastrtps_cpp'

    return LaunchDescription([
        SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value=rmw_implementation_env_var),
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=ld_library_path_env_var),
        world_name,
        robot_name,
        headless,
        renderer,
        verbose,

        ExecuteProcess(
            cmd = [
                isaac_python_launcher_path,
                isaac_custom_launch_script,
                '--world_file', full_path_to_world,
                '--robot_file', full_path_to_robot,
                '--headless', LaunchConfiguration('headless'),
                '--renderer', LaunchConfiguration('renderer')
            ],
            shell = LaunchConfiguration('verbose'),
            output = "screen"
        )
    ])
