from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import asyncio, numpy as np, cv2, yaml

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import sys, os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, currentdir)
from comms_utils import *



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
        
        self.inputs_imgs = list()
        self.outputs_debug_imgs = list()
        for in_img, out_debug_img in zip(INPUTS_IMAGES, OUTPUTS_DEBUG_IMGS):
            # Listed inputs:
            self.inputs_imgs.append(inputs.get(in_img, None))
            # Listed outputs:
            self.outputs_debug_imgs.append(outputs.get(out_debug_img, None))
        # Single outputs:
        self.output_obj_detected = outputs.get(OUTPUT_OBJ_DETECTED, None)

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
        self.robot_namespaces = list(configuration.get("robot_namespaces",
                                                       ["robot1", "robot2"]))
        self.ns_bytes_length = int(configuration.get("ns_bytes_length", 64))     
        self.int_bytes_length = int(configuration.get("int_bytes_length", 4))

        self.lower_threshold = np.array(list(configuration.get("lower_color_filter_threshold",
                                                       [0, 195, 75])))
        self.upper_threshold = np.array(list(configuration.get("upper_color_filter_threshold",
                                                       [16, 225, 105])))
        
        # Other attributes needed:
        self.bridge = CvBridge()
        self.pending = list()

    def detect_object(self, img: np.ndarray, x_pix_step: int, y_pix_step: int,
                      lower_threshold: np.array, upper_threshold: np.array) -> tuple:
        height, width, _ = img.shape # 1920 x 1080
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        #mask = cv2.inRange(hsv_img, lower_threshold, upper_threshold)
        #result = cv2.bitwise_and(img, img, mask = mask)
        
        centroid = [0, 0]
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
        else:
            centroid = None

        img_msg = self.bridge.cv2_to_imgmsg(img, encoding='rgb8')
        return (centroid, ser_ros2_msg(img_msg))

    def create_task_list(self):
        task_list = [] + self.pending
        # For every listed input append an async task to the task_list:
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
                index = int(who[-1]) -1 # Who should be Image1, Image2, ...
                # Get the cv2 image from the ROS2 message:
                img_msg = deser_ros2_msg(data_msg.data, Image)
                img = self.bridge.imgmsg_to_cv2(img_msg,
                                                desired_encoding='passthrough')
                # Apply filter to search the object:
                centroid, ser_debug_img = self.detect_object(img, 100, 100,
                                                         self.lower_threshold,
                                                         self.upper_threshold)
                await self.outputs_debug_imgs[index].send(ser_debug_img)

                # If the object is detecter centroid won't be None:
                if centroid != None:
                    ser_msg = ser_string(self.robot_namespaces[index],
                                         self.ns_bytes_length)
                    ser_msg += ser_int_list(centroid,
                                            self.int_bytes_length)
                    await self.output_obj_detected.send(ser_msg)
                    print(f"OBJ_DETECTOR_OP -> object detected by: {self.robot_namespaces[index]}")
        
        return None

    def finalize(self) -> None:
        return None



def register():
    return PathsPlanner
