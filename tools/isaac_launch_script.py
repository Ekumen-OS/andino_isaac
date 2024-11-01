import argparse
import carb
from omni.isaac.kit import SimulationApp

# This sample loads a usd stage and starts simulation
CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}

# Set up command line arguments
parser = argparse.ArgumentParser("Simulation loader argument parser")
parser.add_argument("--world_file", required=True, help="Full path to the world file")
parser.add_argument("--robot_file", help="Full path to the robot file")
parser.add_argument("--headless", default="False", help="Run stage headless")
parser.add_argument("--renderer", default="RayTracedLighting", choices=["RayTracedLighting", "PathTracing"], help="Renderer to use")
args, unknown = parser.parse_known_args()

# Start the omniverse application
if "True" in args.headless:
	CONFIG["headless"] = True
CONFIG["renderer"] = args.renderer
simulation_app = SimulationApp(launch_config=CONFIG)

# Now that the simulation app is open, continue to load the extension, world and robot

# Enable the ros2_bridge extension. Environment variables must be set
from omni.isaac.core.utils.extensions import enable_extension
if not enable_extension("omni.isaac.ros2_bridge"):
	carb.log_error("ROS2_BRIDGE extension could not be loaded, aborting startup")
	simulation_app.close()
simulation_app.update()

# Load stage
import omni
from omni.isaac.core.utils.stage import is_stage_loading, add_reference_to_stage
try:
    omni.usd.get_context().open_stage(args.world_file)
except ValueError:
    carb.log_error(f"The usd path {args.world_file} could not be opened.")
    simulation_app.close()
simulation_app.update()
simulation_app.update()
while is_stage_loading():
    simulation_app.update()

# Load robot
carb.log_info("Loading robot")
try:
	add_reference_to_stage(usd_path=args.robot_file, prim_path="/andino")
	carb.log_info("Robot loaded")
except FileNotFoundError:
	carb.log_warn("Robot could not be loaded. Check robot file path or load it manually in the simulation")

omni.timeline.get_timeline_interface().play()
while simulation_app.is_running():
    simulation_app.update()
omni.timeline.get_timeline_interface().stop()
simulation_app.close()
