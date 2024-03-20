from zenoh_flow.interfaces import Operator
from zenoh_flow import Input, Output
from zenoh_flow.types import Context
from typing import Dict, Any

import asyncio, numpy as np, cv2, struct

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge

from math import hypot
import sys, os, inspect
currentdir = os.path.dirname(
    os.path.abspath(inspect.getfile(inspect.currentframe()))
    )
sys.path.insert(0, currentdir)
from comms_utils import *
from message_utils import CentroidMessage, trace



INPUTS_CAM_INFOS = ["CamInfo1", "CamInfo2"]
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
        self.float_bytes_length = int(configuration.get("float_bytes_length", 8))

        self.lower_threshold = np.array(list(configuration.get(
            "lower_color_filter_threshold", [0, 195, 75]))
            )
        self.upper_threshold = np.array(list(configuration.get(
            "upper_color_filter_threshold", [16, 225, 105]))
            )
        self.pix_step = configuration.get("pix_step", 30)
        self.max_depth_detection = configuration.get("max_depth_detection", 1.7)
        
        # Single outputs:
        self.output_obj_detected = outputs.take(
            OUTPUT_OBJ_DETECTED,
            CentroidMessage,
            get_ctrd_msg_serializer(
                self.ns_bytes_length,
                self.float_bytes_length
                )
            )
        
        self.inputs_cam_infos = list()
        self.inputs_depths = list()
        self.outputs_debug_imgs = list()
        for in_cam_info, in_depth, out_debug_img in zip(INPUTS_CAM_INFOS,
                                                        INPUTS_DEPTHS,
                                                        OUTPUTS_DEBUG_IMGS):
            # Listed inputs:
            #self.inputs_cam_infos.append(
            #    inputs.take(in_cam_info,
            #                CameraInfo,
            #                get_ros2_deserializer(CameraInfo)
            #                )
            #    )
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
        self.first_time = True
        self.bridge = CvBridge()
        self.pending = list()
        self.cam_infos = [CameraInfo()] * self.robot_num
        self.depths = [PointCloud2()] * self.robot_num

    def get_point_cloud_values(self, pc: PointCloud2, size: int) -> tuple:
        height, width = size
        print("getting values")
        ### Data step is 32 bytes, where the fisrt 12 are the x, y and z coords
        ### (4 bytes each) and so on with y and z coords. The other 20 are 4
        ### void bytes, 3 rgb values bytes and the rest void bytes again.
        datatypes = list()
        offsets = list()
        for field in pc.fields:
            datatypes.append(field.datatype)
            offsets.append(field.offset)
        x_datatype, y_datatype, z_datatype, rgb_datatype = datatypes
        x_offset, y_offset, z_offset, rgb_offset = offsets

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
            endianness + size_list[x_datatype - 1][0],
            endianness + size_list[y_datatype - 1][0],
            endianness + size_list[z_datatype - 1][0]
            )
        depths = (
            points[:, :, x_offset : x_offset + size_list[x_datatype - 1][1]], 
            points[:, :, y_offset : y_offset + size_list[y_datatype - 1][1]], 
            points[:, :, z_offset : z_offset + size_list[z_datatype - 1][1]]
            ) # 4 Bytes each (float32)
        img = points[
            :, :, rgb_offset : rgb_offset + size_list[rgb_datatype - 1][1]
            ] # 3 Bytes (R, G and B).
        
        return (img, depths, depth_format_strings)

    def detect_object(self, pc: PointCloud2, shape: tuple) -> tuple:
        #height, width = shape # 240 x 320 x 3
        ### Getting RGB needed values from pointcloud:
        # In this case RGB values are already integers, so deserialization is
        # not needed:
        rgb_vals, depth_vals_ser, depth_fstrs = self.get_point_cloud_values(
            pc,
            (pc.height, pc.width) # shape: 480 x 640 x 3
            )
        x_depth_vals_ser, y_depth_vals_ser, z_depth_vals_ser = depth_vals_ser
        x_fstr, y_fstr, z_fstr = depth_fstrs

        ### Image filtering:
        hsv_img = cv2.cvtColor(rgb_vals, cv2.COLOR_RGB2HSV)

        #mask = cv2.inRange(hsv_img, self.lower_threshold, self.upper_threshold)
        #result = cv2.bitwise_and(img, img, mask = mask)
        
        x_pix_step = self.pix_step
        y_pix_step = self.pix_step
        centroid_msg = None
        points = list()
        # Iterate only through a few spaced pixels to reduce compute time:
        for i in range(0, pc.width, x_pix_step):
            for j in range(0, pc.height, y_pix_step):
                # Color filter:
                if ((self.lower_threshold <= hsv_img[j, i]).all() and
                    (hsv_img[j, i] <= self.upper_threshold).all()):
                    points.append((i, j))
                    cv2.circle(hsv_img, (i, j), 3, (0, 0, 255), -1)
                    cv2.circle(hsv_img, (i, j), 4, (0, 0, 0), 1)

        ### When object is detected:
        if len(points) > 0:
            points = np.array(points)

            ### Get depth values:
            depth_vals = list()
            for point in points:
                # The function struct.unpack() returns a tuple with a single
                # element, so we get the index 0 to get the value inside of it:
                depth_vals.append([
                    struct.unpack(
                        x_fstr, bytes(x_depth_vals_ser[point[1], point[0], :])
                        )[0],
                    struct.unpack(
                        y_fstr, bytes(y_depth_vals_ser[point[1], point[0], :])
                        )[0],
                    struct.unpack(
                        z_fstr, bytes(z_depth_vals_ser[point[1], point[0], :])
                        )[0]
                ])
            depth_vals = np.array(depth_vals)
            x_val = np.mean(depth_vals[:, 0])
            y_val = np.mean(depth_vals[:, 1])
            z_vals = [i for i in depth_vals[:, 2] if i < self.max_depth_detection]
            # In case there's no z value within the detecting range centroid_msg
            # variable is still None:
            centroid_color = (0, 255, 255) # Red
            if len(z_vals) > 0:
                z_val = np.mean(z_vals)
                centroid_msg = CentroidMessage(x_val, y_val, z_val)
                centroid_color = (50, 255, 255) # Green

            ### Create debug image:
            centroid = (int(np.mean(points[:, 0])), int(np.mean(points[:, 1])))
            # Put a different circle in the centroid.
            cv2.circle(hsv_img, centroid, 9, centroid_color, -1)
            cv2.circle(hsv_img, centroid, 10, (30, 255, 255), 2)
            cv2.putText(
                hsv_img, "Centroid", (centroid[0] - 33, centroid[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA
                )

        rgb_img = cv2.cvtColor(hsv_img, cv2.COLOR_HSV2RGB)
        debug_img_msg = self.bridge.cv2_to_imgmsg(rgb_img, encoding='rgb8')

        return (centroid_msg, debug_img_msg)

    def create_task_list(self):
        task_list = [] + self.pending
        # For every listed input append an async task to task_list:
        for i, in_depth in enumerate(INPUTS_DEPTHS):
            if not any(t.get_name() == in_depth for t in task_list):
                task_list.append(
                    asyncio.create_task(
                        get_input_func(in_depth, self.inputs_depths[i])(),
                        name=in_depth
                    )
                )
        return task_list

    async def iteration(self) -> None:
        # Get the cam info of each robot only once:
        #if self.first_time:
        #    for i in range(self.robot_num):
        #        cam_info_msg = await self.inputs_cam_infos[i].recv()
        #        self.cam_infos[i] = cam_info_msg.get_data()
        #    self.first_time = False
        
        print("outside first time")

        (done, pending) = await asyncio.wait(
            self.create_task_list(),
            return_when=asyncio.FIRST_COMPLETED,
        )
        print("after pending")
        self.pending = list(pending)
        print("obj detector iter")
        for d in done:
            (who, data_msg) = d.result()

            if who in INPUTS_DEPTHS:
                print("msg received")
                # Who will be Depth1 or Depth2, so index will be 0 or 1.
                index = get_ns_index(who) - 1
                point_cloud_msg = data_msg.get_data()
                self.depths[index] = point_cloud_msg
                #shape = (self.cam_infos[index].height,
                #         self.cam_infos[index].width)
                shape=[480, 640]
                centroid_msg, debug_img_msg = self.detect_object(
                    point_cloud_msg, shape
                    )
                await self.outputs_debug_imgs[index].send(debug_img_msg)
                print("sending debug image")
                if centroid_msg != None:
                    centroid_msg.set_founder(self.robot_namespaces[index])
                    cent_x, _, cent_z = centroid_msg.get_centroid()
                    h = hypot(cent_x, cent_z)
                    if h < self.max_depth_detection:
                        trace("OBJ_DETECTOR_OP",
                              (f"Object detected in: ({round(cent_x, 2)}, "
                               f"{round(cent_z, 2)}) from "
                               f"{centroid_msg.get_founder()}"))
                        await self.output_obj_detected.send(centroid_msg)
        return None

    def finalize(self) -> None:
        return None



def register():
    return ObjDetector
