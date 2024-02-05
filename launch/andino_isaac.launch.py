import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from xacro import process_file

def search_isaac_install_path():
    ISAAC_VERSION = "isaac_sim-2023.1.1"
    user_home_path = os.path.expanduser("~")
    isaac_install_path = ""
    for dirpath, dirnames, _ in os.walk(user_home_path):
        for dirname in dirnames:
            if dirname == ISAAC_VERSION:
                isaac_install_path = os.path.join(dirpath, ISAAC_VERSION)
    return isaac_install_path

def get_robot_state_publisher_params():
    pkg_andino_description = get_package_share_directory('andino_description')
    # Obtain urdf from xacro files.
    doc = process_file(os.path.join(pkg_andino_description, 'urdf', 'andino.urdf.xacro'))
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc,
              'use_sim_time': True,
              'publish_frequency': 30.0}
    return params

def generate_launch_description():
    # Paths to places
    pkg_andino_isaac_path = get_package_share_directory('andino_isaac')
    # Look for the omniverse install path
    isaac_install_path = search_isaac_install_path()
    isaac_python_launcher_path = os.path.join(isaac_install_path, "python.sh")
    isaac_custom_launch_script = os.path.join(pkg_andino_isaac_path, "tools", "isaac_launch_script.py")
    full_path_to_world = PathJoinSubstitution([pkg_andino_isaac_path, 'isaac_worlds', LaunchConfiguration('world_name')])
    full_path_to_robot = PathJoinSubstitution([pkg_andino_isaac_path, 'andino_isaac_description', LaunchConfiguration('robot_name')])

    # Environment variables
    prev_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_library_path_env_var = prev_ld_library_path + ":" + isaac_install_path + "/exts/omni.isaac.ros2_bridge/humble/lib"
    rmw_implementation_env_var = 'rmw_fastrtps_cpp'

    # Arguments
    world_name = DeclareLaunchArgument(
        'world_name',
        default_value='plain_world.usda',
        description='Name of the world to launch',
        choices=[world for world in os.listdir(os.path.join(pkg_andino_isaac_path, 'isaac_worlds')) if world.endswith(('.usd', '.usda'))]
    )
    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='andino.usda',
        description='Name of the robot to spawn',
        choices=[robot for robot in os.listdir(os.path.join(pkg_andino_isaac_path, 'andino_isaac_description')) if robot.endswith(('.usd', '.usda'))]
    )
    headless = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Isaac sim headless',
    )
    renderer = DeclareLaunchArgument(
        'renderer',
        default_value='RayTracedLighting',
        description='Renderer to use',
        choices=["RayTracedLighting", "PathTracing"]
    )
    verbose = DeclareLaunchArgument(
        'verbose',
        default_value='True',
        description='Show Isaac sim output',
    )
    rsp_argument = DeclareLaunchArgument(
        'rsp',
        default_value='true',
        description='Run robot state publisher node.'
    )

    # Andino description
    rsp = Node(package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='',
                output='both',
                parameters=[get_robot_state_publisher_params()],
                condition=IfCondition(LaunchConfiguration('rsp'))
    )

    return LaunchDescription([
        world_name,
        robot_name,
        headless,
        renderer,
        verbose,
        rsp_argument,
        rsp,

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
            output = "screen",
            additional_env = {
                "RMW_IMPLEMENTATION": rmw_implementation_env_var,
                "LD_LIBRARY_PATH": ld_library_path_env_var
            }
        )
    ])
