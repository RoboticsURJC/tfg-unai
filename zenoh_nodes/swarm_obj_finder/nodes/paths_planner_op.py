from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import yaml

from visualization_msgs.msg import MarkerArray

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir)
from geom_utils import *
from comms_utils import *
from marker_utils import *
from map_utils import *



INPUT_WP_REQUEST = "WPRequest"

OUTPUT_DEBUG_MAP_DIV = "DebugMapDiv"
OUTPUT_DEBUG_MARKER  = "DebugMarkers"
OUTPUT_NEXT_WP       = "NextWP"

MAP_KEY_IMAGE       = "image"
MAP_KEY_RESOLUTION  = "resolution"
MAP_KEY_ORIGIN      = "origin"
MAP_KEY_OCCU_THRESH = "occupied_thresh"
MAP_KEY_FREE_THRESH = "free_thresh"
MAP_KEY_NEGATE      = "negate"

MARKER_FRAME_ID = "map"



class PathsPlanner(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration

        # Single inputs:
        self.input_wp_req = inputs.get(INPUT_WP_REQUEST, None)
        # Single outputs:
        self.output_debug_img = outputs.get(OUTPUT_DEBUG_MAP_DIV, None)
        self.output_markers = outputs.get(OUTPUT_DEBUG_MARKER, None)
        self.output_next_wp = outputs.get(OUTPUT_NEXT_WP, None)

        # TODO: With the new update the common config file is not needed anymore
        # since the it can be put directly in the data-flow yaml file:

        # Add the common configuration to this node's configuration
        common_cfg_file = str(configuration.get("common_cfg_file",
                                                "config/common_cfg.yaml"))
        common_cfg_yaml_file = open(common_cfg_file)
        common_cfg_dict = yaml.load(common_cfg_yaml_file,
                                    Loader=yaml.FullLoader)
        common_cfg_yaml_file.close()
        configuration.update(common_cfg_dict)

        # Get configuration values:
        self.robot_num = int(configuration.get("swarm_size", 2))
        self.robot_namespaces = list(configuration.get("robot_namespaces",
                                                       ["robot1", "robot2"]))
        self.ns_bytes_length = int(configuration.get("ns_bytes_length", 64))
        self.wp_separation = float(configuration.get("waypoint_separation", 1.0))
        map_yaml_path = str(configuration.get(
                                "map_yaml_file",
                                "maps/turtlebot3_world/turtlebot3_world.yaml")
                            )

        # Other attributes needed:
        self.debug_img_sent = False
        self.ids = 0
        self.times = 0
        self.colors = [[1.0, 0.5, 1.0, 1.0],
                       [0.0, 1.0, 0.5, 1.0]]

        # Load map (yaml fields and img from yaml file):
        self.load_map(map_yaml_path)

        # Get the bounding box and divide the map (get the bounding corners):
        divs, self.debug_div_img_msg_ser = divide_map(self.img_interpr,
                                                      self.map_img,
                                                      self.robot_num,
                                                      (self.map_free_thresh,
                                                       self.map_occupied_thresh),
                                                      True)
        # For each division get the path and store it:
        marker_array_msg = MarkerArray()
        marker_array_msg.markers = []
        self.paths = []
        invert = False
        for div, ns in zip(divs, self.robot_namespaces):
            path = get_path_from_area(div, self.img_interpr,
                                      (self.map_free_thresh,
                                       self.map_occupied_thresh),
                                      self.wp_world_separation, self.map_origin,
                                      self.map_resolution, invert)
            # Invert every odd path to separete the
            # robots (different starting points):
            invert = not invert
            self.paths.append(path)
            marker_array_msg.markers += self.get_markers_from_path(path, ns)
        self.marker_array_msg_ser = ser_ros2_msg(marker_array_msg)
        
    def load_map(self, map_yaml_path: str) -> None:
        map_yaml_file = open(map_yaml_path)
        self.map_data_dict = yaml.load(map_yaml_file, Loader=yaml.FullLoader)
        map_yaml_file.close()

        # Check the needed fields:
        keys_needed = [MAP_KEY_IMAGE, MAP_KEY_RESOLUTION, MAP_KEY_ORIGIN,
                       MAP_KEY_OCCU_THRESH, MAP_KEY_FREE_THRESH]
        if (not all(map(lambda x: x in self.map_data_dict.keys(), keys_needed))\
            or len(self.map_data_dict.get(MAP_KEY_ORIGIN)) != 3): # [x, y, yaw]
            print(f"ERROR: required field/s ({keys_needed}) missing in map yaml file")
            raise Exception("ERROR: required field/s missing")
        
        self.map_img = cv2.imread(self.map_data_dict.get(MAP_KEY_IMAGE),
                                  cv2.IMREAD_GRAYSCALE)
        # Map img values interpretation (Following the specifications in:
        # http://wiki.ros.org/map_server:
        map_negate = bool(self.map_data_dict.get(MAP_KEY_NEGATE))
        self.img_interpr = self.map_img / 255.0 if map_negate else (255 - self.map_img) / 255.0

        self.map_free_thresh = self.map_data_dict.get(MAP_KEY_FREE_THRESH)
        self.map_occupied_thresh = self.map_data_dict.get(MAP_KEY_OCCU_THRESH)
        self.map_origin = self.map_data_dict.get(MAP_KEY_ORIGIN)
        self.map_resolution = self.map_data_dict.get(MAP_KEY_RESOLUTION)
        self.wp_world_separation = round(self.wp_separation / self.map_resolution)

    def get_markers_from_path(self, path: list, ns: str) -> list:
        markers = []
        for wp in path:
            self.ids += 1
            color = self.colors[self.times % len(self.colors)]
            marker_dict = {"id": self.ids, "ns": ns,
                            "frame_locked": False, "frame_id": MARKER_FRAME_ID,
                            "lifetime_s": 0, "lifetime_ns":0,
                            "pose": [wp.pose.position.x,
                                     wp.pose.position.y,
                                     wp.pose.orientation], # [x, y, yaw(quat)]
                            "scale": [0.1, 0.05, 0.05], "color_rgba": color}
            
            markers.append(get_marker(marker_dict))
        self.times += 1
        # The first one will always be red:
        markers[0].color.r = 1.0
        markers[0].color.g = 0.0
        markers[0].color.b = 0.0
        return markers

    async def iteration(self) -> None:
        if not self.debug_img_sent:
            print("PATHS_PLANNER_OP -> Sending debug information")
            await self.output_debug_img.send(self.debug_div_img_msg_ser)
            await self.output_markers.send(self.marker_array_msg_ser)
            self.debug_img_sent = True

        # Process waypoint requests:
        data_msg = await self.input_wp_req.recv()

        ns = deser_string(data_msg.data, ' ') # Who (namespace)
        index = self.robot_namespaces.index(ns)
        next_wp = self.paths[index].pop(0)
        next_wp_ser = ser_ros2_msg(next_wp)
        await self.output_next_wp.send(data_msg.data + next_wp_ser)
        print(f"PATHS_PLANNER_OP -> Sending next waypoint to {ns}")

        return None

    def finalize(self) -> None:
        return None



def register():
    return PathsPlanner
