from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import time, asyncio
from numpy import arcsin, cos, sin

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

import sys, os, inspect
currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe()))
    )
sys.path.insert(0, currentdir)
from comms_utils import *
from geom_utils import *
from marker_utils import *
from message_utils import CentroidMessage, WorldPosition



INPUT_OBJ_DETECTED = "ObjDetected"
INPUTS_ROBOT_POSES = ["RobotPose1", "RobotPose2"]

OUTPUT_WORLD_OBJ_POSE = "WorldObjPose"
OUTPUT_DEBUG_MARKER   = "DebugMarkers"

MARKER_FRAME_ID = "map"



class ObjPosInfer(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration

        # Get configuration values:
        self.robot_num = int(configuration.get("swarm_size", 2))
        self.robot_namespaces = list(configuration.get("robot_namespaces",
                                                       ["robot1", "robot2"]))
        self.ns_bytes_length = int(configuration.get("ns_bytes_length", 64))
        self.float_bytes_length = int(configuration.get("float_bytes_length", 8))
        self.safe_distance = float(configuration.get("safe_distance", 0.3))
        
        # Single inputs:
        self.input_obj_detected = inputs.take(
            INPUT_OBJ_DETECTED,
            CentroidMessage,
            get_ctrd_msg_deserializer(
                self.ns_bytes_length,
                self.float_bytes_length
                )
            )
        # Single outputs:
        self.output_world_pos = outputs.take(
            OUTPUT_WORLD_OBJ_POSE,
            WorldPosition,
            get_world_pos_msg_serializer(self.ns_bytes_length)
            )
        self.output_debug_marker = outputs.take(
            OUTPUT_DEBUG_MARKER,
            MarkerArray,
            ser_ros2_msg
            )
        
        self.inputs_robot_poses = list()
        for in_rob_pose in INPUTS_ROBOT_POSES:
            # Listed inputs:
            self.inputs_robot_poses.append(inputs.take(
                in_rob_pose,
                PoseStamped,
                get_ros2_deserializer(PoseStamped))
                )

        # Other attributes needed:
        self.pending = list()
        self.robot_poses = [PoseStamped()] * self.robot_num

    def robot2world(self, centroid: tuple, index: int) -> tuple:
        x, _, z = centroid
        h = math.hypot(x, z)
        obj_angle = arcsin(x / h)
        _, _, robot_yaw = quat2euler(self.robot_poses[index].pose.orientation)

        world_angle = robot_yaw - obj_angle

        world_pose = PoseStamped()
        world_pose.pose.position.x = self.robot_poses[index].pose.position.x\
            + h * cos(world_angle)
        world_pose.pose.position.y = self.robot_poses[index].pose.position.y\
            + h * sin(world_angle)
        world_pose.pose.orientation = self.robot_poses[index].pose.orientation

        ### DEBUG (for markers):
        marker_arr = MarkerArray()

        # Marker (blue) from map frame (absolute coords)
        marker_dict = {"id": 1000, "ns": "obj_pose", "frame_locked": False,
                       "frame_id": MARKER_FRAME_ID, "scale": [0.2, 0.1, 0.1], 
                       "lifetime_s": 0, "lifetime_ns": 0,
                       "pose": [world_pose.pose.position.x,
                                world_pose.pose.position.y,
                                world_pose.pose.orientation], #[x, y, yaw(quat)]
                       "color_rgba": [0.2, 0.2, 1.0, 1.0]}
        marker_arr.markers.append(get_marker(marker_dict))

        return (WorldPosition(world_pos=world_pose), marker_arr)

    def create_task_list(self):
        task_list = [] + self.pending

        # For every listed input append an async task to the task_list:
        for i, in_rob_pose in enumerate(INPUTS_ROBOT_POSES):
            if not any(t.get_name() == in_rob_pose for t in task_list):
                task_list.append(
                    asyncio.create_task(
                        get_input_func(in_rob_pose,
                                       self.inputs_robot_poses[i])(),
                        name=in_rob_pose
                    )
                )
        # Append single inputs async task to the task_list one by one:
        if not any(t.get_name() == INPUT_OBJ_DETECTED for t in task_list):
            task_list.append(
                asyncio.create_task(get_input_func(INPUT_OBJ_DETECTED,
                                                   self.input_obj_detected)(),
                                    name=INPUT_OBJ_DETECTED)
            )
        
        return task_list

    async def iteration(self) -> None:
        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )
        self.pending = list(pending)
        for d in done:
            (who, data_msg) = d.result()

            msg = data_msg.get_data()
            # Get the robot poses:
            if who in INPUTS_ROBOT_POSES:
                index = int(who[-1]) -1 # Who should be RobotPose1, RobotPose2, ...
                self.robot_poses[index] = msg

            # Object detected by the obj_detector_op node:
            if who == INPUT_OBJ_DETECTED:
                centroid_msg = msg
                ns = centroid_msg.get_founder()
                
                index = self.robot_namespaces.index(ns)
                # Get the centroid coords from the msg:
                centroid = centroid_msg.get_centroid()

                # Get the object's world pose and a marker message
                world_pose, debug_marker_msg = self.robot2world(centroid,
                                                                index)
                world_pose.set_sender(ns)
                # Send the 3D pose:
                await self.output_world_pos.send(world_pose)
                position = world_pose.get_world_position()
                print(
                    f"OBJ_POS_INFER_OP| Object position from {ns} is ("
                    f"{round(position.pose.position.x, 2)}, "
                    f"{round(position.pose.position.z, 2)})"
                    )
                await self.output_debug_marker.send(debug_marker_msg)

        return None

    def finalize(self) -> None:
        return None



def register():
    return ObjPosInfer
