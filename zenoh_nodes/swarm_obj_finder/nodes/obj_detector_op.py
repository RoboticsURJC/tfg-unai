from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import asyncio, numpy as np, cv2, struct

from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge

import sys, os, inspect
currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe()))
    )
sys.path.insert(0, currentdir)
from comms_utils import *
from message_utils import CentroidMessage



INPUTS_IMAGES = ["Image1", "Image2"]
INPUTS_DEPTHS = ["Depth1", "Depth2"]

OUTPUT_OBJ_DETECTED = "ObjDetected"
OUTPUTS_DEBUG_IMGS  = ["DebugImgFiltered1", "DebugImgFiltered2"]



class ObjDetector(Operator):
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
        self.int_bytes_length = int(configuration.get("int_bytes_length", 4))

        self.lower_threshold = np.array(list(configuration.get(
            "lower_color_filter_threshold", [0, 195, 75]))
            )
        self.upper_threshold = np.array(list(configuration.get(
            "upper_color_filter_threshold", [16, 225, 105]))
            )
        self.pix_step = configuration.get("pix_step", 30)
        
        # Single outputs:
        self.output_obj_detected = outputs.take(
            OUTPUT_OBJ_DETECTED,
            CentroidMessage,
            get_ctrd_msg_serializer(self.ns_bytes_length, self.int_bytes_length)
            )
        
        self.inputs_imgs = list()
        self.inputs_depths = list()
        self.outputs_debug_imgs = list()
        for in_img, in_depth, out_debug_img in zip(INPUTS_IMAGES,
                                                   INPUTS_DEPTHS,
                                                   OUTPUTS_DEBUG_IMGS):
            # Listed inputs:
            self.inputs_imgs.append(
                inputs.take(in_img, Image, get_ros2_deserializer(Image))
                )
            self.inputs_depths.append(
                inputs.take(in_depth,
                            PointCloud2,
                            get_ros2_deserializer(PointCloud2)
                            )
                )
            # Listed outputs:
            self.outputs_debug_imgs.append(
                outputs.take(out_debug_img, Image, ser_ros2_msg)
                )
        
        # Other attributes needed:
        self.bridge = CvBridge()
        self.pending = list()
        self.depths = [PointCloud2()] * self.robot_num

    def get_point_cloud_values(self, pc: PointCloud2, size: int) -> tuple:
        height, width = size

        ### self.depths[index].data goes from 32 bytes to 32 bytes, where the
        ### first 4 are the x coord, the next 4 are the y, the next 4 are the z
        ### and the other 20 divides in 4 void bytes are the values rgb:
        x_offset = pc.fields[0].offset
        y_offset = pc.fields[1].offset
        z_offset = pc.fields[2].offset
        rgb_offset = pc.fields[3].offset
        
        # Redimension to 320x240x32:
        points = np.reshape(pc.data, (height, width, pc.point_step))
        
        endianness = '>' if pc.is_bigendian else '<'
        # Format info of the fields in https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html:
        """
        uint8 INT8    = 1
        uint8 UINT8   = 2
        uint8 INT16   = 3
        uint8 UINT16  = 4
        uint8 INT32   = 5
        uint8 UINT32  = 6
        uint8 FLOAT32 = 7
        uint8 FLOAT64 = 8
        """
        # List that contains the format character and the bytes size:
        size_list = [['b', 1], ['B', 1], ['h', 2], ['H', 2],
                        ['i', 4], ['I', 4], ['f', 4], ['d', 8]]
        depth_format_strings = (
            endianness + size_list[pc.fields[0].datatype - 1][0],
            endianness + size_list[pc.fields[1].datatype - 1][0],
            endianness + size_list[pc.fields[2].datatype - 1][0])
        
        depths = (
            points[
                :,
                :,
                x_offset : x_offset + size_list[pc.fields[0].datatype - 1][1]
                ], 
            points[
                :,
                :,
                y_offset : y_offset + size_list[pc.fields[1].datatype - 1][1]
                ], 
            points[
                :,
                :,
                z_offset : z_offset + size_list[pc.fields[2].datatype - 1][1]
                ]
            ) # 4 Bytes each (float32)
        img = points[
            :,
            :,
            rgb_offset : rgb_offset + size_list[pc.fields[3].datatype - 1][1]
            ] # 3 Bytes (R, G and B).
        
        return (img, depths, depth_format_strings)

    def detect_object(self, pc: PointCloud2) -> tuple:

        height, width = (240, 320) # 240 x 320 x 3
        ### Getting RGB needed values from pointcloud:
        # In this case RGB values are already integers, so deserialization is
        # not needed:
        rgb_vals, depth_vals_ser, depth_fstrs = self.get_point_cloud_values(
            pc,
            (height, width)
            )
        x_depth_values, y_depth_values, z_depth_values = depth_vals_ser
        x_fstr, y_fstr, z_fstr = depth_fstrs

        ### Image filtering:
        hsv_img = cv2.cvtColor(rgb_vals, cv2.COLOR_RGB2HSV)

        #mask = cv2.inRange(hsv_img, self.lower_threshold, self.upper_threshold)
        #result = cv2.bitwise_and(img, img, mask = mask)
        
        x_pix_step = self.pix_step
        y_pix_step = self.pix_step
        centroid = [0, 0]
        centroid_msg = None
        num = 0
        # Iterate only through a few spaced pixels to reduce compute time:
        for i in range(0, width, x_pix_step):
            for j in range(0, height, y_pix_step):
                # Color filter:
                if ((self.lower_threshold < hsv_img[j, i]).all() and
                    (hsv_img[j, i] < self.upper_threshold).all()):
                    centroid[0] += i
                    centroid[1] += j
                    num += 1
                    cv2.circle(hsv_img, (i, j), 5, (255, 255, 0), 2) #DEBUG

        ### When object is detected:
        if num != 0:
            centroid[0] /= num
            centroid[1] /= num

            ### Create debug image:
            # Put a different circle in the centroid.
            cv2.circle(
                hsv_img,
                (int(centroid[0]), int(centroid[1])),
                10,
                (255, 0, 255),
                -1) #DEBUG

            ### Get depth values:
            #struct.unpack() returns a one element tuple, so we get the index 0:
            x_val = struct.unpack(
                x_fstr,
                bytes(x_depth_values[int(centroid[0]), int(centroid[1]), :])
                )[0]
            y_val = struct.unpack(
                y_fstr,
                bytes(y_depth_values[int(centroid[0]), int(centroid[1]), :])
                )[0]
            z_val = struct.unpack(
                z_fstr,
                bytes(z_depth_values[int(centroid[0]), int(centroid[1]), :])
                )[0]

            centroid_msg = CentroidMessage(x_val, y_val)

        rgb_img = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2RGB)
        debug_img_msg = self.bridge.cv2_to_imgmsg(rgb_img, encoding='rgb8')

        return (centroid_msg, debug_img_msg)

    def create_task_list(self):
        task_list = [] + self.pending
        # For every listed input append an async task to task_list:
        for i, (in_img, in_depth) in enumerate(zip(INPUTS_IMAGES,
                                                   INPUTS_DEPTHS)):
            if not any(t.get_name() == in_img for t in task_list):
                task_list.append(
                    asyncio.create_task(
                        get_input_func(in_img, self.inputs_imgs[i])(),
                        name=in_img
                    )
                )
            if not any(t.get_name() == in_depth for t in task_list):
                task_list.append(
                    asyncio.create_task(
                        get_input_func(in_depth, self.inputs_depths[i])(),
                        name=in_depth
                    )
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

            if who in INPUTS_DEPTHS:
                index = int(who[-1]) -1
                point_cloud_msg = data_msg.get_data()
                self.depths[index] = point_cloud_msg
                centroid_msg, debug_img_msg = self.detect_object(point_cloud_msg)
                await self.outputs_debug_imgs[index].send(debug_img_msg)
                if centroid_msg != None:
                    centroid_msg.set_founder(self.robot_namespaces[index])
                    print(f"OBJ_DETECTOR_OP -> object detected by: {centroid_msg.get_founder()} in {centroid_msg.get_centroid()}")
                    await self.output_obj_detected.send(centroid_msg)

            #TODO: get the camera info from the topic /robot*/intel_realsense_r200_depth/depth/camera_info

            """
            if who in INPUTS_IMAGES:
                # We take the last character of who, because it will conatin
                # "Image1", "Image2", ..., so substracting 1 we have the index
                # of the input/output or namespace lists:
                index = int(who[-1]) -1
                
                img_msg = data_msg.get_data()
                # Get the cv2 image from the ROS2 message:
                img = self.bridge.imgmsg_to_cv2(img_msg,
                                                desired_encoding='passthrough')
                # Apply a color filter to search the object:
                centroid_msg, debug_img_msg = self.detect_object_old(img, index)
                #await self.outputs_debug_imgs[index].send(debug_img_msg)

                # If the object is detected, its centroid won't be None:
                if centroid_msg != None:
                    centroid_msg.set_founder(self.robot_namespaces[index])
                    print(f"OBJ_DETECTOR_OP -> object detected by: {centroid_msg.get_founder()} in {centroid_msg.get_centroid()}")
                    await self.output_obj_detected.send(centroid_msg)
            """
        
        return None

    def finalize(self) -> None:
        return None



def register():
    return ObjDetector
