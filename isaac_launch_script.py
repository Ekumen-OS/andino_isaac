import os
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot

class SimulationLoader(object):
	def __init__(self) -> None:
		self._world = None
		self._current_tasks = None
		self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
		return

	def load(self):
		self.enable_ros2_bridge_extension()
		self.load_world()
		self.spawn_robot()
		#self._world.play()

	def enable_ros2_bridge_extension(self):
		# Environment variables needed to load the ROS2 bridge
		os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
		prev_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
		user_home_path = os.path.expanduser("~")
		os.environ["LD_LIBRARY_PATH"] = prev_ld_library_path + ":" + user_home_path + "/.local/share/ov/pkg/isaac_sim-2023.1.0/exts/omni.isaac.ros2_bridge/humble/lib"
		# Enable ROS2_bridge extension. This must be done before loading Andino given that the action graphs depend on the extension
		enable_extension("omni.isaac.ros2_bridge")

	def load_world(self):
		print("Loading world")
		# Opening the stage USD file with the robot
		try:
			open_stage("/home/aeneas/omniverse/andino_isaac/isaac_worlds/plain_world.usda")
			self._world = World(**self._world_settings)
		except ValueError:
			print("Stage could not be loaded, check path")
		return

	def spawn_robot(self):
		# Spawn the robot as a payload
		print("Spawning robot")
		add_reference_to_stage(usd_path="/home/aeneas/omniverse/andino_isaac/andino_isaac_description/andino_isaac.usda", prim_path="/andino")
		self.robot1 = self._world.scene.add(WheeledRobot(prim_path="/andino", name="robot1", position=[0.,0.,0.2]))
		# open_stage("/home/aeneas/omniverse/andino_isaac/andino_isaac_description/andino_isaac.usda")
		return

if __name__ == '__main__':
	sim_loader = SimulationLoader()
	sim_loader.load()


