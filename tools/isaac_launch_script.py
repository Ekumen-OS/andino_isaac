import argparse
import carb
import time
import isaacsim
from isaacsim.simulation_app import SimulationApp

# Set up command line arguments
parser = argparse.ArgumentParser("Simulation loader argument parser")
parser.add_argument("--world_file", required=True, help="Full path to the world file")
parser.add_argument("--robot_file", help="Full path to the robot file")
parser.add_argument("--headless", default="False", help="Run stage headless")
parser.add_argument("--renderer", default="RayTracedLighting", choices=["RayTracedLighting", "PathTracing"], help="Renderer to use")
args, unknown = parser.parse_known_args()

# This sample loads a usd stage and starts simulation
CONFIG = {"width": 1280, "height": 720, "sync_loads": True, "headless": False, "renderer": "RayTracedLighting"}

# Start the omniverse application
if "True" in args.headless:
	CONFIG["headless"] = True
CONFIG["renderer"] = args.renderer
simulation_app = SimulationApp(launch_config=CONFIG)

# Now that the simulation app is open, continue to load the extension, world and robot
from isaacsim.core.utils.extensions import enable_extension

if not enable_extension("isaacsim.core.nodes"):
    carb.log_error("Unable to load the nodes extension, aborting startup")
    simulation_app.close()

# Enable the ros2_bridge extension. Environment variables must be set
if not enable_extension("isaacsim.ros2.bridge"):
    carb.log_error("Unable to load the ros2_bridge extension, aborting startup")
    simulation_app.close()

# Without this wait cycle, Isaac will sometimes crash when loading the stage right after loading
# the ros2_bridge extension.
simulation_app.update()

# Load stage
import omni
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.core.utils.stage import add_reference_to_stage
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

# Run the simulation loop
from isaacsim.core.api.simulation_context import SimulationContext
simulation_context = SimulationContext(stage_units_in_meters=1.0)

rendering_dt = 1.0 / 60.0
physics_dt = 1.0 / 60.0

simulation_context.set_simulation_dt(
    physics_dt=physics_dt,
    rendering_dt=rendering_dt,
)

omni.timeline.get_timeline_interface().play()

# Step the simulation at a steady 1.0 RTF whenever possible.
# If we can't keep up with 1.0, then simulate as fast as possible.
last_sleep_end = time.time()
while simulation_app.is_running():
    simulation_context.step(render=True)
    time_elapsed = time.time() - last_sleep_end
    sleep_time = physics_dt - time_elapsed
    if sleep_time > 0.0:
        time.sleep(sleep_time)
    last_sleep_end = time.time()

omni.timeline.get_timeline_interface().stop()
simulation_app.close()
