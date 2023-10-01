from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import asyncio, numpy as np, cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import sys, os, inspect
currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe()))
    )
sys.path.insert(0, currentdir)
from comms_utils import *
from message_utils import CentroidMessage



INPUTS_IMAGES = ["Image1", "Image2"]

OUTPUT_OBJ_DETECTED = "ObjDetected"
OUTPUTS_DEBUG_IMGS  = ["DebugImgFiltered1", "DebugImgFiltered2"]



class PathsPlanner(Operator):
    def __init__(
        self,
        context: Context,
        configuration: Dict[str, Any],
        inputs: Dict[str, Input],
        outputs: Dict[str, Output],
    ):
        configuration = {} if configuration is None else configuration
        # Get configuration values:
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
        
        # Single outputs:
        self.output_obj_detected = outputs.take(
            OUTPUT_OBJ_DETECTED,
            CentroidMessage,
            get_ctrd_msg_serializer(self.ns_bytes_length, self.int_bytes_length)
            )
        
        self.inputs_imgs = list()
        self.outputs_debug_imgs = list()
        for in_img, out_debug_img in zip(INPUTS_IMAGES, OUTPUTS_DEBUG_IMGS):
            # Listed inputs:
            self.inputs_imgs.append(
                inputs.take(in_img, Image, get_ros2_deserializer(Image))
                )
            # Listed outputs:
            self.outputs_debug_imgs.append(
                outputs.take(out_debug_img, Image, ser_ros2_msg)
                )
        
        # Other attributes needed:
        self.bridge = CvBridge()
        self.pending = list()

    def detect_object(self, img: np.ndarray,
                      x_pix_step: int, y_pix_step: int,
                      lower_threshold: np.array,
                      upper_threshold: np.array) -> tuple:
        height, width, _ = img.shape # 1920 x 1080
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        #mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)
        #result = cv2.bitwise_and(img, img, mask = mask)
        
        centroid = [0, 0]
        centroid_msg = None
        num = 0
        # Iterate only through a few spaced pixels to reduce comuting:
        for i in range(0, width, x_pix_step):
            for j in range(0, height, y_pix_step):
                # Color filter:
                if ((lower_threshold < hsv_img[j, i]).all() and
                    (hsv_img[j, i] < upper_threshold).all()):
                    centroid[0] += i
                    centroid[1] += j
                    num += 1
                    cv2.circle(img, (i, j), 10, (255, 255, 0), -1) #DEBUG

        if num != 0:
            centroid[0] /= num
            centroid[1] /= num
            centroid_msg = CentroidMessage(centroid[0], centroid[1])

        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        return (centroid_msg, img_msg)

    def create_task_list(self):
        task_list = [] + self.pending
        # For every listed input append an async task to task_list:
        for i, in_tf in enumerate(INPUTS_IMAGES):
            if not any(t.get_name() == in_tf for t in task_list):
                task_list.append(
                    asyncio.create_task(
                        get_input_func(in_tf, self.inputs_imgs[i])(), name=in_tf
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
                centroid_msg, debug_img_msg = self.detect_object(img, 100, 100,
                                                         self.lower_threshold,
                                                         self.upper_threshold)
                await self.outputs_debug_imgs[index].send(debug_img_msg)

                # If the object is detected, its centroid won't be None:
                if centroid_msg != None:
                    centroid_msg.set_founder(self.robot_namespaces[index])
                    #TESTING: NEVER SEND (TO SEE IF THEY FOLLOW THE PATH)
                    print(f"OBJ_DETECTOR_OP -> object detected by: {centroid_msg.get_founder()} in {centroid_msg.get_centroid()}")
                    #await self.output_obj_detected.send(centroid_msg)
        
        return None

    def finalize(self) -> None:
        return None



def register():
    return PathsPlanner
