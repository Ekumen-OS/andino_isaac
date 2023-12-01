import argparse
import numpy as np
import os
import carb
from omni.isaac.kit import SimulationApp

# This sample loads a usd stage and starts simulation
CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}

# Set up command line arguments
parser = argparse.ArgumentParser("Simulation loader argument parser")
parser.add_argument("--world_file", default="plain_world.usda", help="World file name including extension")
parser.add_argument("--robot_file", default="just_andino.usda", help="Robot file name including extension")
args, unknown = parser.parse_known_args()
cwd = os.getcwd()
world_file = cwd + "/isaac_worlds/" + args.world_file
robot_file = cwd + "/andino_isaac_description/" + args.robot_file

# Start the omniverse application
simulation_app = SimulationApp(launch_config=CONFIG)

# Now that the simulation app is open, continue to load the extension, world and robot
# These imports need to be here as they depend on SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import open_stage, is_stage_loading, add_reference_to_stage
from omni.isaac.version import get_version

# Enable the ros2_bridge extension. Environment variables must be set
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
prev_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
user_home_path = os.path.expanduser("~")
isaac_pkg_folder_name = "isaac_sim-" + get_version()[0]
os.environ["LD_LIBRARY_PATH"] = prev_ld_library_path + ":" + user_home_path + "/.local/share/ov/pkg/" + isaac_pkg_folder_name + "/exts/omni.isaac.ros2_bridge/humble/lib"
if not enable_extension("omni.isaac.ros2_bridge"):
	print("ROS2_BRIDGE extension could not be loaded, aborting startup")
	simulation_app.close()

# Open world
try:
	print("Loading world please wait")
	open_stage(world_file)
	# Wait two frames so that stage starts loading
	simulation_app.update()
	simulation_app.update()
	while is_stage_loading():
			simulation_app.update()
	world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
	world = World(**world_settings)
	print("World loaded")
except ValueError:
	print("Stage could not be loaded, check file path. Aborting startup")
	simulation_app.close()

# Load robot
print("Loading robot")
try:
	add_reference_to_stage(usd_path=robot_file, prim_path="/andino")
	print("Robot loaded")
except FileNotFoundError:
	print("Robot could not be loaded. Check robot file path or load it manually in the simulation")

world.reset()

while simulation_app.is_running():
    world.step()

    # Deal with pause/stop
    if world.is_playing():
        if world.current_time_step_index == 0:
            world.reset()

simulation_app.close()
