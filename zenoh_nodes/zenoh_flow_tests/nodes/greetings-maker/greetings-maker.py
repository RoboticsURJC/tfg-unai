#
# Copyright (c) 2022 ZettaScale Technology
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Contributors:
#   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
#

from zenoh_flow.interfaces import Operator
from zenoh_flow import Inputs, Outputs
from zenoh_flow.types import Context
from typing import Dict, Any

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.type_support import check_for_type_support

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan



def ser_ros2_msg(ros2_msg) -> bytes:
    check_for_type_support(type(ros2_msg))
    return _rclpy.rclpy_serialize(ros2_msg, type(ros2_msg))

def deser_ros2_msg(ser_ros2_msg: bytes, ros2_type):
    check_for_type_support(ros2_type)
    return _rclpy.rclpy_deserialize(ser_ros2_msg, ros2_type)

def deser_laserscan_msg(ser_ros2_msg: bytes):
    check_for_type_support(LaserScan)
    return _rclpy.rclpy_deserialize(ser_ros2_msg, LaserScan)


class GreetingsMaker(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Inputs,
        outputs: Outputs,
    ):
        print(f"Context: {context}")
        print(f"Configuration: {configuration}")
        self.output = outputs.take("greeting", str, lambda s: bytes(s, "utf-8"))
        self.output_wp = outputs.take("goal_pose_wp", PoseStamped, serializer=ser_ros2_msg)

        #self.in_stream = inputs.take("name", str, lambda buf: buf.decode("utf-8"))
        self.in_stream = inputs.take("name", LaserScan, deserializer=deser_laserscan_msg)

        if self.in_stream is None:
            raise ValueError("No input 'name' found")
        if self.output is None:
            raise ValueError("No output 'greeting' found")

    def finalize(self) -> None:
        return None

    async def iteration(self) -> None:
        message = await self.in_stream.recv()
        name = message.get_data()
        if name is not None:
            greetings = self.generate_greetings("Gabriele")
            await self.output.send(greetings)
            #name=LaserScan()
            print(name)
        wp = PoseStamped()
        wp.pose.position.x = 0.5
        wp.pose.position.y = 0.5
        wp.pose.position.z = 0.0
        await self.output_wp.send(wp)
        print("WP SENT")
        

        return None

    def generate_greetings(self, name: str) -> str:
        greetings_dict = {
            "Sofia": "Ciao, {}!\n",
            "Leonardo": "Ciao, {}!\n",
            "Lucia": "¡Hola, {}!\n",
            "Martin": "¡Hola, {}!\n",
            "Jade": "Bonjour, {}!\n",
            "Gabriele": "Ciao, PaaS manager!\n",
        }

        greet = greetings_dict.get(name, "Hello, {}!\n")
        return greet.format(name)


def register():
    return GreetingsMaker
