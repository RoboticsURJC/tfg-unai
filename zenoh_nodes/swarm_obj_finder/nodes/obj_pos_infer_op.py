from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import time, asyncio, yaml
from numpy import arctan2, rad2deg, cos, sin, deg2rad

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, LaserScan
from visualization_msgs.msg import MarkerArray

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir)
from comms_utils import *
from geom_utils import *
from marker_utils import *



INPUT_OBJ_DETECTED = "ObjDetected"
INPUTS_ROBOT_POSES = ["RobotPose1", "RobotPose2"]
INPUTS_CAM_INFOS   = ["CamInfo1", "CamInfo2"]
INPUTS_LIDARS      = ["Lidar1", "Lidar2"]

OUTPUT_WORLD_OBJ_POSE = "WorldObjPose"
OUTPUT_DEBUG_MARKER   = "DebugMarkers"

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
        self.input_obj_detected = inputs.get(INPUT_OBJ_DETECTED, None)
        # Single outputs:
        self.output_world_pos = outputs.get(OUTPUT_WORLD_OBJ_POSE, None)
        self.output_debug_marker = outputs.get(OUTPUT_DEBUG_MARKER, None)
        
        self.inputs_robot_poses = list()
        self.inputs_cam_infos = list()
        self.inputs_lidars = list()
        for in_rob_pose, in_cam_info, in_lidar in zip(INPUTS_ROBOT_POSES,
                                                      INPUTS_CAM_INFOS,
                                                      INPUTS_LIDARS):
            # Listed inputs:
            self.inputs_robot_poses.append(inputs.get(in_rob_pose, None))
            self.inputs_cam_infos.append(inputs.get(in_cam_info, None))
            self.inputs_lidars.append(inputs.get(in_lidar, None))

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
        self.int_bytes_length = int(configuration.get("int_bytes_length", 4))
        self.lidar_threshold = int(configuration.get("lidar_threshold", 4))
        self.safe_distance = float(configuration.get("safe_distance", 0.3))

        # Other attributes needed:
        self.first_time = True
        self.first_obj_found = True
        self.pending = list()
        self.cam_infos = [CameraInfo()] * self.robot_num
        self.robot_poses = [PoseStamped()] * self.robot_num
        self.lidars = [LaserScan()] * self.robot_num
        self.last_time = time.time()

    def lidar_mean(self, lidar: LaserScan, angle: int, thresh: int) -> float:
        lidar_ht = round(thresh / 2)
        avg = 0
        counter = 0
        # Get the mean of a lidar range without having into account inf values:
        for i in range(angle - lidar_ht, angle + lidar_ht):
            if i >= 360:
                i -= 360
            if lidar.range_min < lidar.ranges[i] < lidar.range_max:
                avg += lidar.ranges[i]
                counter += 1
            
        if avg == 0:
            return None
        avg /= counter
        return avg

    def img2world(self, pix: tuple, index: int) -> tuple:
        # Get the angle of view (AOV) from the cam info:
        film_size_x = self.cam_infos[index].width
        obj_x_img = pix[0]
        f_x = self.cam_infos[index].k[0] # Focal point x (from 3x3 k matrix).
        # f_y = cam_info.k[4] # Focal point y (not needed).
        aov_h = 2 * arctan2(film_size_x, 2*f_x) # Horizontal angle of view.

        # Angle of the object from the robot between [-aov_h/2, aov_h/2]:
        obj_ang = - rad2deg((aov_h * obj_x_img / film_size_x) - (aov_h / 2))
        obj_ang = round(obj_ang)

        # Get the dist of the object to get its polar coords from robot's pose:
        obj_dist = self.lidar_mean(self.lidars[index], obj_ang,
                                   self.lidar_threshold) - self.safe_distance
        _, _, yaw = quat2euler(self.robot_poses[index].pose.orientation) #rpy
        tot_ang = yaw + deg2rad(obj_ang)
        x_world_dist_from_rob = obj_dist * cos(tot_ang)
        y_world_dist_from_rob = obj_dist * sin(tot_ang)

        world_pose = PoseStamped()
        world_pose.pose.position.x = self.robot_poses[index].pose.position.x\
            + x_world_dist_from_rob
        world_pose.pose.position.y = self.robot_poses[index].pose.position.y\
            + y_world_dist_from_rob
        world_pose.pose.orientation = self.robot_poses[index].pose.orientation
        
        ### DEBUG:
        marker_arr = MarkerArray()

        # Marker (blue) from map frame (absolute coords)
        marker_dict = {"id": 1000, "ns": "obj_pose", "frame_locked": False,
                       "frame_id": MARKER_FRAME_ID, "scale": [0.2, 0.1, 0.1], 
                       "lifetime_s": 0, "lifetime_ns":0,
                       "pose": [world_pose.pose.position.x,
                                world_pose.pose.position.y,
                                world_pose.pose.orientation], # [x, y, yaw(quat)]
                       "color_rgba": [0.2, 0.2, 1.0, 1.0]}
        marker_arr.markers.append(get_marker(marker_dict))

        # To see the marker from the robot (but it's displayed in every robot):
        # Marker (pinkie) from base_scan frame (relative to the robot coords)
        
        #marker_dict = {"id": 1001, "ns": "obj_pose", "frame_locked": False,
        #               "frame_id":"base_scan", "lifetime_s": 0, "lifetime_ns":0,
        #               "pose": [obj_dist * cos(deg2rad(obj_ang)),
        #                        obj_dist * sin(deg2rad(obj_ang)),
        #                        Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)], # [x, y, yaw(quat)]
        #               "scale": [0.2, 0.1, 0.1], "color_rgba": [0.7, 0.5, 1.0, 1.0]}
        #marker_arr.markers.append(get_marker(marker_dict))
        ###

        return (world_pose, marker_arr)

    def create_task_list(self):
        task_list = [] + self.pending

        # For every listed input append an async task to the task_list:
        for i, (in_rob_pose, in_lidar) in enumerate(
            zip(INPUTS_ROBOT_POSES, INPUTS_LIDARS)
            ):
            if not any(t.get_name() == in_rob_pose for t in task_list):
                task_list.append(
                    asyncio.create_task(
                        get_input_func(in_rob_pose, self.inputs_robot_poses[i])(),
                        name=in_rob_pose
                    )
                )
            if not any(t.get_name() == in_lidar for t in task_list):
                task_list.append(
                    asyncio.create_task(
                        get_input_func(in_lidar, self.inputs_lidars[i])(),
                        name=in_lidar
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
        # Get the cam info of each robot only once:
        if self.first_time:
            for i in range(self.robot_num):
                cam_info_msg_ser = await self.inputs_cam_infos[i].recv()
                self.cam_infos[i] = deser_ros2_msg(cam_info_msg_ser.data, CameraInfo)
            self.first_time = False

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )
        self.pending = list(pending)
        for d in done:
            (who, data_msg) = d.result()

            # Get the robot poses:
            if who in INPUTS_ROBOT_POSES:
                index = int(who[-1]) -1 # Who should be RobotPose1, RobotPose2, ...
                self.robot_poses[index] = deser_ros2_msg(data_msg.data,
                                                         PoseStamped)

            # Get the lidars:
            if who in INPUTS_LIDARS:
                index = int(who[-1]) -1 # Who should be Lidar1, Lidar2, ...
                self.lidars[index] = deser_ros2_msg(data_msg.data, LaserScan)

            # Object detected by the obj_detector_op node:
            if who == INPUT_OBJ_DETECTED:
                ser_ns = data_msg.data[:self.ns_bytes_length]
                ns = deser_string(ser_ns)

                if self.first_obj_found:
                    self.firs_robot = ns
                    self.first_obj_found = False
                
                if ns == self.firs_robot:
                    index = self.robot_namespaces.index(ns)
                    # Get the centroid from the msg:
                    centroid = deser_int_list(
                        data_msg.data[self.ns_bytes_length:], self.int_bytes_length
                        )
                    # Convert it from 2D to 3D thanks to the lidar:
                    world_pose, debug_marker_msg = self.img2world(tuple(centroid),
                                                                  index)
                    # Send the 3D pose:
                    ser_world_pos = ser_ros2_msg(world_pose)
                    await self.output_world_pos.send(ser_ns + ser_world_pos)
                    
                    ser_debug_marker = ser_ros2_msg(debug_marker_msg)
                    await self.output_debug_marker.send(ser_debug_marker)
                    self.last_time = time.time()

        return None

    def finalize(self) -> None:
        return None



def register():
    return PathsPlanner
