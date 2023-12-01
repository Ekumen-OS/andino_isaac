import os
import carb
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core import World
from omni.isaac.version import get_version

class SimulationLoader(object):
	def __init__(self) -> None:
		self._world = None
		self._current_tasks = None
		self._world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
		return

	def load(self):
		# ROS2_BRIDGE extension is critical
		if not self.enable_ros2_bridge_extension():
			print ("Unable to load ros2_bridge extension, aborting startup")
			return
		# World is critical
		if not self.load_world():
			print ("Unable to load world, aborting startup")
			return
		# Robot is not critical, do not abort if loading fails
		self.spawn_robot()
		#self._world.play()

	def enable_ros2_bridge_extension(self) -> bool:
		"""
		Method to make sure the ROS2_BRIDGE Isaac extension is enabled, aborting execution if it's not
		"""
		# Enable ROS2_bridge extension. This must be done before loading Andino given that the action graphs depend on the extension
		if not enable_extension("omni.isaac.ros2_bridge"):
			print("ROS2_BRIDGE extension could not be loaded, aborting startup")
			return False
		return True

	def load_world(self) -> bool:
		"""
		Method to load the specified simulation world
		"""
		try:
			open_stage("/home/aeneas/omniverse/andino_isaac/isaac_worlds/plain_world.usda")
			self._world = World(**self._world_settings)
		except ValueError:
			print("Stage could not be loaded, check file path")
			return False
		return True

	def spawn_robot(self):
		"""
		Method to spawn the robot as a reference
		"""
		try:
			add_reference_to_stage(usd_path="/home/aeneas/omniverse/andino_isaac/andino_isaac_description/andino_isaac.usda", prim_path="/andino")
		except FileNotFoundError:
			print("Robot could not be loaded. Check robot file path or load it manually in the simulation")
		return

if __name__ == '__main__':
	sim_loader = SimulationLoader()
	sim_loader.load()
