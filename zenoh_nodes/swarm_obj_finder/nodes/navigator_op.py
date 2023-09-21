from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import asyncio, time, yaml
from math import sqrt

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from tf2_msgs.msg import TFMessage
import tf2_ros, rclpy


import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir)
from comms_utils import *
from geom_utils import *



INPUT_NEXT_WP        = "NextWP"
INPUT_WORLD_OBJ_POSE = "WorldObjPose"
INPUTS_TFS           = ["TF1", "TF2"]

OUTPUT_WP_REQ       = "WPRequest"
OUTPUTS_ROBOT_POSES = ["RobotPose1", "RobotPose2"]
OUTPUTS_WPS         = ["Waypoint1", "Waypoint2"]



class Navigator(Operator):
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

        self.goal_checker_min_dist = float(
            configuration.get("goal_checker_min_dist", 0.3)
            )
        self.goal_resend_timeout = float(
            configuration.get("goal_resend_timeout", 1.0)
            )
        
        # Single inputs:
        #self.input_next_wp = inputs.get(INPUT_NEXT_WP, None)
        #self.input_world_pos = inputs.get(INPUT_WORLD_OBJ_POSE, None)
        #self.input_next_wp = inputs.take(
        #    INPUT_NEXT_WP,
        #    WorldPosition,
        #    get_world_pos_msg_deserializer(self.ns_bytes_length)
        #    )
        self.input_world_pos = inputs.take(
            INPUT_WORLD_OBJ_POSE,
            WorldPosition,
            get_world_pos_msg_deserializer(self.ns_bytes_length)
            )

        # Single outputs:
        #self.output_wp_req = outputs.get(OUTPUT_WP_REQ, None)
        self.output_wp_req = outputs.take(
            OUTPUT_WP_REQ,
            str,
            get_str_serializer(self.ns_bytes_length))
        
        self.inputs_tfs = list()
        self.outputs_robot_poses = list()
        self.outputs_wps = list()
        for in_tf, out_rob_pose, out_wp in zip(INPUTS_TFS,
                                               OUTPUTS_ROBOT_POSES,
                                               OUTPUTS_WPS):
            # Listed inputs:
            #self.inputs_tfs.append(inputs.get(in_tf, None))
            self.inputs_tfs.append(
                inputs.take(in_tf, TFMessage, get_ros2_deserializer(TFMessage))
                )
            # Listed outputs:
            #self.outputs_robot_poses.append(outputs.get(out_rob_pose, None))
            #self.outputs_wps.append(outputs.get(out_wp, None))
            self.outputs_robot_poses.append(
                outputs.take(out_rob_pose, PoseStamped, ser_ros2_msg)
                )
            self.outputs_wps.append(
                outputs.take(out_wp, PoseStamped, ser_ros2_msg)
                )

        # Other attributes needed:
        self.pending = list()
        self.first_time = True
        self.object_found = False
        self.buffer_core = tf2_ros.BufferCore(Duration(sec=1, nanosec=0))
        self.current_wps = [[PoseStamped(), time.time(), -1.0]] * self.robot_num

    def create_task_list(self):
        task_list = [] + self.pending

        # For every listed input append an async task to the task_list:
        for i, in_tf in enumerate(INPUTS_TFS):
            if not any(t.get_name() == in_tf for t in task_list):
                task_list.append(
                    asyncio.create_task(
                        get_input_func(in_tf, self.inputs_tfs[i])(), name=in_tf
                    )
                )
        # Append single inputs async task to the task_list one by one:
        #if not any(t.get_name() == INPUT_NEXT_WP for t in task_list):
        #    task_list.append(
        #        asyncio.create_task(get_input_func(INPUT_NEXT_WP,
        #                                           self.input_next_wp)(),
        #                            name=INPUT_NEXT_WP)
        #    )
        if not any(t.get_name() == INPUT_WORLD_OBJ_POSE for t in task_list):
            task_list.append(
                asyncio.create_task(get_input_func(INPUT_WORLD_OBJ_POSE,
                                                   self.input_world_pos)(),
                                    name=INPUT_WORLD_OBJ_POSE)
            )
        
        return task_list

    async def iteration(self) -> None:
        # Make the first request for each robot only once:
        if self.first_time:
            for ns in self.robot_namespaces:
                print(f"NAVIGATOR_OP -> {ns} sending first waypoint request...")
                #ns_ser = ser_string(ns, self.ns_bytes_length, ' ')
                #await self.output_wp_req.send(ns_ser)
                await self.output_wp_req.send(ns)
            self.first_time = False

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )
        self.pending = list(pending)
        for d in done:
            (who, data_msg) = d.result()

            # Get the next waypoint:
            #if who == INPUT_NEXT_WP and not self.object_found:
            #    ns = deser_string(data_msg.data[:self.ns_bytes_length], ' ')
            #    index = self.robot_namespaces.index(ns)
            #    
            #    ser_current_wp = data_msg.data[self.ns_bytes_length:]
            #    self.current_wps[index] = [deser_ros2_msg(ser_current_wp, PoseStamped),
            #                               time.time(), -1.0]
            #    print(f"NAVIGATOR_OP -> {self.robot_namespaces[index]} received next waypoint, sending it: {get_xy_from_pose(self.current_wps[index][0])}")
            #    await self.outputs_wps[index].send(ser_current_wp)

            # Get the robots poses:
            if who in INPUTS_TFS:
                index = int(who[-1]) -1 # Who should be TF1, TF2, ...
                ns = self.robot_namespaces[index]
                #self.tf_msg = deser_ros2_msg(data_msg.data, TFMessage)
                self.tf_msg = data_msg.get_data()
                for tf in self.tf_msg.transforms:

                    tf_names = ["map->odom",
                                "odom->base_footprint"]
                    key = tf.header.frame_id + "->" + tf.child_frame_id
                    if key in tf_names:
                        self.buffer_core.set_transform(tf, "default_authority")
                    
                        try:
                            new_tf = self.buffer_core.lookup_transform_core(
                                'map', 'base_footprint', rclpy.time.Time()
                                )
                            new_tf.child_frame_id = 'world_robot_pos'

                            # Send robot's TFs to obj_pos_infer operator:
                            robot_pose = PoseStamped()
                            robot_pose.pose.position.x = new_tf.transform.translation.x
                            robot_pose.pose.position.y = new_tf.transform.translation.y
                            robot_pose.pose.orientation = new_tf.transform.rotation
                            #ser_pose = ser_ros2_msg(robot_pose)
                            #await self.outputs_robot_poses[index].send(ser_pose)
                            await self.outputs_robot_poses[index].send(robot_pose)

                            x_dist = new_tf.transform.translation.x - self.current_wps[index][0].pose.position.x
                            y_dist = new_tf.transform.translation.y - self.current_wps[index][0].pose.position.y
                            dist = sqrt(x_dist**2 + y_dist**2)
                            self.current_wps[index][2] = dist
                            if (dist < self.goal_checker_min_dist
                                and not self.object_found):
                                print("NAVIGATOR_OP -> Waypoint reached, sending next request...")
                                ns = self.robot_namespaces[index]
                                #ns_ser = ser_string(ns, self.ns_bytes_length, ' ')
                                #await self.output_wp_req.send(ns_ser)
                                await self.output_wp_req.send(ns)
                            
                        except Exception as e:
                            pass
                            #print(e) ### DEBUG

            # Get the object's 3D pose:
            if who == INPUT_WORLD_OBJ_POSE:
                self.object_found = True
                #ser_ns = data_msg.data[:self.ns_bytes_length]
                #ns = deser_string(ser_ns)
                wp_msg = data_msg.get_data() # it's a WorldPosition type object.

                #WE MAY NOT NEED TO GET THE INDEX OR NAMESPACE HERE:
                ns = wp_msg.get_sender()
                index = self.robot_namespaces.index(ns)
                
                #ser_obj_pos = data_msg.data[self.ns_bytes_length:] #We don't need to deserialize it
                # Send all the robots to the object's pose and stop following paths:
                for i, output in enumerate(self.outputs_wps):
                    obj_pose = wp_msg.get_world_position()
                    print(f"NAVIGATOR_OP -> Sending {self.robot_namespaces[i]} to object's position in: {obj_pose}")
                    #output.send(ser_obj_pos)
                    output.send(obj_pose)
    
    def finalize(self) -> None:
        return None



def register():
    return Navigator
