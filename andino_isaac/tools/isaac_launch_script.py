import argparse
import carb
from omni.isaac.kit import SimulationApp

# This sample loads a usd stage and starts simulation
CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}

# Set up command line arguments
parser = argparse.ArgumentParser("Simulation loader argument parser")
parser.add_argument("--world_file", help="Full path to the world file")
parser.add_argument("--robot_file", help="Full path to the robot file")
args, unknown = parser.parse_known_args()

# Start the omniverse application
simulation_app = SimulationApp(launch_config=CONFIG)

# Now that the simulation app is open, continue to load the extension, world and robot
# These imports need to be here as they depend on SimulationApp
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import open_stage, is_stage_loading, add_reference_to_stage

# Enable the ros2_bridge extension. Environment variables must be set
if not enable_extension("omni.isaac.ros2_bridge"):
	print("ROS2_BRIDGE extension could not be loaded, aborting startup")
	simulation_app.close()

# Open world
try:
	print("Loading world please wait")
	open_stage(args.world_file)
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
	add_reference_to_stage(usd_path=args.robot_file, prim_path="/andino")
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
