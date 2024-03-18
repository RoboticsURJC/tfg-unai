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
from rclpy.clock import Clock

from sensor_msgs.msg import LaserScan, Image

import numpy as np, cv2



def ser_ros2_msg(ros2_msg) -> bytes:
    check_for_type_support(type(ros2_msg))
    return _rclpy.rclpy_serialize(ros2_msg, type(ros2_msg))

def deser_ros2_msg(ser_ros2_msg: bytes, ros2_type):
    check_for_type_support(ros2_type)
    return _rclpy.rclpy_deserialize(ser_ros2_msg, ros2_type)

def deser_laserscan_msg(ser_ros2_msg: bytes):
    check_for_type_support(LaserScan)
    return _rclpy.rclpy_deserialize(ser_ros2_msg, LaserScan)



#DEBUG_IMG_MARGIN  = -70 #[%] # Negative number to zoom in
#DEBUG_IMG_SCALE   = 200 #[px]
DEBUG_IMG_MARGIN  = 5   #[%] # Negative number to zoom in
DEBUG_IMG_SCALE   = 50 #[px] # before 100 (sim)
WHITE_COLOR_PIXEL = 255

class PolarCoord():
    def __init__(self, radius: float, theta: float) -> None:
        self.r = radius
        self.theta = theta #azimuth, measured as 0[rad] starting from north direction

    def to_cartesian(self) -> 'CartesianCoord': #'' needed for forward reference
        x = self.r * np.cos(self.theta)
        y = self.r * np.sin(self.theta)
        return CartesianCoord(x, y)

class CartesianCoord():
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y
    
    def to_polar(self) -> PolarCoord:
        radius = np.sqrt(self.x**2 + self.y**2)
        theta = np.arctan2(self.y, self.x)
        return PolarCoord(radius, theta)
        
class LidarCircleDetector(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Inputs,
        outputs: Outputs,
    ):
        self.input = inputs.take(
            "lidar_in", LaserScan, deserializer=deser_laserscan_msg
            )
        self.output = outputs.take(
            "img_out", Image, serializer=ser_ros2_msg
            )

        if self.input is None:
            raise ValueError("No input 'lidar_in' found")
        if self.output is None:
            raise ValueError("No output 'img_out' found")
        
        self.inverted = True

    def finalize(self) -> None:
        return None

    def get_bg_img(self, size: int) -> np.ndarray:
        return np.zeros((size, size), dtype=np.uint8)

    def get_cart_points(self, laser_msg: LaserScan) -> list:
        cartesian_points = list()
        for i, range in enumerate(laser_msg.ranges):
            # Calculate the angle and ensure it's within the range [0, 2*pi)
            if not np.isinf(range):
                angle = laser_msg.angle_min + i * laser_msg.angle_increment
                angle = np.mod(angle, 2 * np.pi)
                polar_point = PolarCoord(range, angle)
                if laser_msg.range_min < range < laser_msg.range_max:
                    cartesian_points.append(polar_point.to_cartesian())
        return cartesian_points

    def draw_points(self, bg_img: np.ndarray, cart_points: list) -> np.ndarray:
        pixel_centre = int(bg_img.shape[0] / 2)
        drew_img = cv2.circle(
            bg_img,
            (pixel_centre, pixel_centre),
            1,
            WHITE_COLOR_PIXEL,
            2
            )
        for cart_point in cart_points:
            pixel_x = pixel_centre + int(DEBUG_IMG_SCALE * cart_point.x)
            pixel_y = pixel_centre + int(DEBUG_IMG_SCALE * cart_point.y)
            drew_img[pixel_y, pixel_x] = WHITE_COLOR_PIXEL
        return drew_img

    def detect_circles(self, image: np.ndarray) -> list:
        #blurred_img = image
        blurred_img = cv2.GaussianBlur(image, (5, 5), 0) # before (sim) :image, (5, 5), 2
        circles = cv2.HoughCircles(
            blurred_img,
            cv2.HOUGH_GRADIENT_ALT,
            dp=1,
            minDist=10,
            param1=1,
            param2=1,
            minRadius=1,
            maxRadius=100)
            #before (sim): 1, 10, 1, 10, 1, 50
        print(circles)
        # If circles are found, draw them
        if circles is None:
            return blurred_img
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(image, (x, y), r, WHITE_COLOR_PIXEL, 2)  # Draw the circle outline
            cv2.circle(image, (x, y), 2, WHITE_COLOR_PIXEL, 3)  # Draw the center of the circle
        return image

    def arr2img(self, img_arr: np.ndarray) -> Image:
        img_msg = Image()

        # Define mappings for number of color channels to encoding:
        channel_mappings = {
            1: "mono8",
            3: "rgb8"
        }
        
        # Determine the number of color channels in the image and check if it's
        # supported:
        num_channels = img_arr.shape[2] if len(img_arr.shape) == 3 else 1
        if num_channels not in channel_mappings:
            raise ValueError("Unsupported number of channels in the input image")
        
        # Populate the Image message fields
        img_msg.height, img_msg.width = img_arr.shape[:2]
        img_msg.encoding = channel_mappings[num_channels]
        img_msg.step = num_channels * img_msg.width
        img_msg.data = img_arr.flatten().tolist() # Flatten array and convert to list
        
        return img_msg

    async def iteration(self) -> None:
        ser_msg = await self.input.recv()
        laser_msg = ser_msg.get_data()
        if self.inverted:
            laser_msg.ranges.reverse()

        non_inf_ranges = [r for r in laser_msg.ranges if not np.isinf(r)]
        max_laser_range = max(non_inf_ranges)
        #bg_img = self.get_bg_img(2 * int(laser_msg.range_max * DEBUG_IMG_SCALE * (1 + DEBUG_IMG_MARGIN / 100)))
        bg_img = self.get_bg_img(2 * int(max_laser_range * DEBUG_IMG_SCALE * (1 + DEBUG_IMG_MARGIN / 100))) # Better results
        cart_points = self.get_cart_points(laser_msg)
        laser_img = self.draw_points(bg_img, cart_points)
        debug_img = self.detect_circles(laser_img)

        debug_img_msg = self.arr2img(debug_img)
        await self.output.send(debug_img_msg)

        return None



def register():
    return LidarCircleDetector
