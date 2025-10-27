import carb
import time
import os
import isaacsim
from isaacsim.simulation_app import SimulationApp

# Get paths from env variables
scenario_file_path = os.environ["SCENARIO_PATH"]
andino_file_path = os.environ["ROBOT_PATH"]

# Set up the SimulationApp configuration
CONFIG = {
    "width": 1280,
    "height": 720,
    "sync_loads": True,
    "headless": True if os.environ['HEADLESS'] == 'true' else False,
    "renderer": "RayTracedLighting",
}
if os.environ["VIEWPORT"] == "local":
    # no changes needed.
    pass
elif os.environ["VIEWPORT"] == "remote":
    # Empty the DISPLAY environment variable or this won't work if xpra is working.
    os.environ["DISPLAY"] = ":"
    CONFIG.update(
        {
            "window_width": 1920,
            "window_height": 1080,
            "headless": True,
            "hide_ui": False,
        }
    )
# Start the omniverse application
simulation_app = SimulationApp(launch_config=CONFIG)

# Now that the simulation app is open, continue to load the extensions, world and robot
from isaacsim.core.utils.extensions import enable_extension

# Enable the omnigraphs extension
if not enable_extension("isaacsim.core.nodes"):
    carb.log_error("Unable to load the nodes extension, aborting startup")
    simulation_app.close()

# Enable the ros2_bridge extension
if not enable_extension("isaacsim.ros2.bridge"):
    carb.log_error("Unable to load the ros2_bridge extension, aborting startup")
    simulation_app.close()

# Enable the livestream extension if user chose remote viewport
if os.environ["VIEWPORT"] == "remote":
    enable_extension("omni.kit.livestream.webrtc")

# Without this wait cycle, Isaac will sometimes crash when loading the stage right after loading
# the ros2_bridge extension.
simulation_app.update()

# Load stage
import omni
from isaacsim.core.utils.stage import is_stage_loading
from isaacsim.core.utils.stage import add_reference_to_stage
try:
    omni.usd.get_context().open_stage(scenario_file_path)
except ValueError:
    carb.log_error(f"The usd path {scenario_file_path} could not be opened.")
    simulation_app.close()
simulation_app.update()
simulation_app.update()
while is_stage_loading():
    simulation_app.update()

# Load robot
carb.log_info("Loading robot")
try:
	add_reference_to_stage(usd_path=andino_file_path, prim_path="/andino")
	carb.log_info("Robot loaded")
except FileNotFoundError:
	carb.log_warn("Robot could not be loaded. Check robot file path or load it manually in the simulation")

# Run the simulation loop
from isaacsim.core.api.simulation_context import SimulationContext
simulation_context = SimulationContext(stage_units_in_meters=1.0)

rendering_dt = 1.0 / float(os.environ['RENDERING_RATE'])
physics_dt = 1.0 / float(os.environ['PHYSICS_RATE'])

simulation_context.set_simulation_dt(
    physics_dt=physics_dt,
    rendering_dt=rendering_dt,
)

if not os.environ['START_STOPPED']:
    omni.timeline.get_timeline_interface().play()
    carb.log_info("Simulation started")

# Step the simulation at a steady 1.0 RTF whenever possible.
# If we can't keep up with 1.0, then simulate as fast as possible.
last_sleep_end = time.time()
while simulation_app.is_running():
    simulation_context.step(render=True)
    if not os.environ['RUN_REAL_TIME']:
        continue
    time_elapsed = time.time() - last_sleep_end
    sleep_time = physics_dt - time_elapsed
    if sleep_time > 0.0:
        time.sleep(sleep_time)
    last_sleep_end = time.time()

omni.timeline.get_timeline_interface().stop()
simulation_app.close()
