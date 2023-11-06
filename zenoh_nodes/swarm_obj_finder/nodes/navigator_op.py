from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import asyncio, time
from math import sqrt

from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
from tf2_msgs.msg import TFMessage
import tf2_ros, rclpy


import sys, os, inspect
currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe()))
    )
sys.path.insert(0, currentdir)
from comms_utils import *
from geom_utils import *



INPUT_NEXT_WP        = "NextWP"
INPUT_WORLD_OBJ_POSE = "WorldObjPose"
INPUTS_TFS           = ["TF1", "TF2"]

OUTPUT_WP_REQ       = "WPRequest"
OUTPUTS_ROBOT_POSES = ["RobotPose1", "RobotPose2"]
OUTPUTS_WPS         = ["Waypoint1", "Waypoint2"]

#MODES
SEARCH_MODE, APPROACH_MODE = 0, 1



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
        self.obj_safe_dist = float(
            configuration.get("obj_safe_dist", 0.3)
            )
        self.master_timeout = float(
            configuration.get("master_timeout", 5.0)
            )
        self.approach_timeout = float(
            configuration.get("approach_timeout", 30.0)
            )
        
        # Single inputs:
        self.input_next_wp = inputs.take(
            INPUT_NEXT_WP,
            WorldPosition,
            get_world_pos_msg_deserializer(self.ns_bytes_length)
            )
        self.input_world_pos = inputs.take(
            INPUT_WORLD_OBJ_POSE,
            WorldPosition,
            get_world_pos_msg_deserializer(self.ns_bytes_length)
            )

        # Single outputs:
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
            self.inputs_tfs.append(
                inputs.take(in_tf, TFMessage, get_ros2_deserializer(TFMessage))
                )
            # Listed outputs:
            self.outputs_robot_poses.append(
                outputs.take(out_rob_pose, PoseStamped, ser_ros2_msg)
                )
            self.outputs_wps.append(
                outputs.take(out_wp, PoseStamped, ser_ros2_msg)
                )

        # Other attributes needed:
        self.pending = list()
        self.first_time = True
        self.buffer_core = tf2_ros.BufferCore(Duration(sec=1, nanosec=0))
        self.current_wps = [[PoseStamped(), time.time(), -1.0]] * self.robot_num
        self.goal_manager = GoalManager(self.robot_namespaces)
        self.last_obj_update_ts = None
        self.master_sight_time = 0.0
        self.obj_pose = PoseStamped()
        self.mode = SEARCH_MODE

        self.print_once = True

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
        if not any(t.get_name() == INPUT_NEXT_WP for t in task_list):
            task_list.append(
                asyncio.create_task(get_input_func(INPUT_NEXT_WP,
                                                   self.input_next_wp)(),
                                    name=INPUT_NEXT_WP)
            )
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
                await self.output_wp_req.send(ns)
            self.first_time = False

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )
        self.pending = list(pending)
        for d in done:
            (who, data_msg) = d.result()

            #if not "TF" in who:
            #    print(who)

            # Get the next waypoint:
            if who == INPUT_NEXT_WP and self.mode == SEARCH_MODE:
                msg = data_msg.get_data()
                ns = msg.get_sender()
                index = self.robot_namespaces.index(ns)
                current_wp = msg.get_world_position()

                self.current_wps[index] = [current_wp, time.time(), -1.0]
                x, y = get_xy_from_pose(self.current_wps[index][0])
                #print(
                #    f"NAVIGATOR_OP\t| Sending {self.robot_namespaces[index]} "
                #    f"to the next waypoint: {round(x, 2)}, {round(y, 2)}"
                #    )
                #await self.outputs_wps[index].send(current_wp)

            if who in INPUTS_TFS:
                index = int(who[-1]) -1 # Who should be TF1, TF2, ...
                ns = self.robot_namespaces[index]
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

                            # Send robot's TFs to obj_pos_infer_op node:
                            robot_pose = PoseStamped()
                            robot_pose.pose.position.x = \
                                new_tf.transform.translation.x
                            robot_pose.pose.position.y = \
                                new_tf.transform.translation.y
                            robot_pose.pose.orientation = \
                                new_tf.transform.rotation
                            await self.outputs_robot_poses[index].send(
                                robot_pose
                                )

                            x_dist = new_tf.transform.translation.x - \
                                self.current_wps[index][0].pose.position.x
                            y_dist = new_tf.transform.translation.y - \
                                self.current_wps[index][0].pose.position.y
                            dist = math.hypot(x_dist, y_dist)
                            self.current_wps[index][2] = dist
                            if (dist < self.goal_checker_min_dist):
                                ns = self.robot_namespaces[index]
                                await self.output_wp_req.send(ns)
                            
                        except Exception as e:
                            pass
                            #print(e) ### DEBUG

            # Get the object's 3D pose:
            if who == INPUT_WORLD_OBJ_POSE:
                self.mode = APPROACH_MODE
                # A master is picked and it sends the position to the rest of
                # robots that has not reached the goal yet to stop following
                # the paths and start approaching to the object's pose:
                wp_msg = data_msg.get_data() # It's a WorldPosition type object.
                ns = wp_msg.get_sender()
                self.obj_pose = wp_msg.get_world_position()
                # Header needed for Nav2 planner server:
                self.obj_pose.header.frame_id = "map"
                self.last_obj_update_ts = time.time()

                # If the master doesn't exist this robot will be the master:
                print(not self.goal_manager.master_exists(),
                      not self.goal_manager.has_reached_goal(ns))
                if not self.goal_manager.master_exists() and \
                    not self.goal_manager.has_reached_goal(ns):
                    print(
                        f"NAVIGATOR_OP\t| New master selected: "
                        f"\033[0;32m{ns}\033[0m"
                        )
                    self.goal_manager.set_master(ns)
                
                # If this robot is the master, it will send the goal to the
                # other robots that haven't reached the goal yet:
                if ns == self.goal_manager.get_master():
                    print(
                        f"NAVIGATOR_OP\t| Sending robots to the object position"
                        )
                    for robot_ns, output in zip(self.robot_namespaces,
                                                self.outputs_wps):
                        if not self.goal_manager.has_reached_goal(robot_ns):
                            pass
                            #await output.send(self.obj_pose)

                # Set if the goal has been reached to stop receiving the object
                # position (if this robot is the master let that role free for
                # the next robot):
                robot_dist = math.hypot(self.obj_pose.pose.position.x,
                                        self.obj_pose.pose.position.z)
                if robot_dist < self.obj_safe_dist:
                    self.goal_manager.set_reached(ns)
                    print(f"NAVIGATOR_OP\t| \033[0;32mObject reached!\033[0m")
                    if ns == self.goal_manager.get_master():
                        self.goal_manager.free_master()



            # If 5s has passed without any position received from master the
            # its role is assumed by another robot. If 30s has passed, the mode
            # is switched to search the object again:
            if self.mode == APPROACH_MODE:
                # If the master robot haven't seen the object for more than 5s,
                # let its role free for the next robot:
                time_elapsed = time.time() - self.last_obj_update_ts
                if time_elapsed > self.master_timeout \
                    and self.goal_manager.master_exists():
                    self.goal_manager.free_master()
                    print(f"NAVIGATOR_OP\t| \033[0;31mmaster freed\033[0m")
                # If more than 60s search the object again:
                elif time_elapsed > self.approach_timeout:
                    self.mode = SEARCH_MODE
                    # Send all robots their last waypoiny (where they stoped
                    # searching to start approaching the object):
                    for i, output in enumerate(self.outputs_wps):
                        #await output.send(self.current_wps[i][0])
                        pass
                    print(
                        f"NAVIGATOR_OP\t| \033[0;31mObject's not at sight "
                        f"anymore, returning to search mode again\033[0m"
                        )
            
            # TESTING: robotX sees the object, becomes master and stops seeing
            # the object, if robotX sees the object again it becomes master, but
            # if RobotY sees the object instead, it doesn't become master
            # (because it no longer enter in approach mdoe from other robot).
            
            # BUG: Once the master is selected, it no longer enters in approach
            # mode from the other robots, and that's why they can no longer be
            # the master.

            # DEBUG: print info every 5 seconds:
            t = time.time()
            freq = 5
            if int(t) % freq == 0 and self.print_once:
                print("Mode: search" if self.mode == SEARCH_MODE else "Mode: approach")
                self.print_once = False
            if int(t) % freq == 1:
                self.print_once = True

    def finalize(self) -> None:
        return None



def register():
    return Navigator
