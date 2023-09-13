from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
#from rclpy.clock import Clock
from rclpy.time import Time



def get_marker(def_dict: dict) -> Marker:
    marker = Marker()
    # The main difference between them is that:
    marker.header.stamp = Time().to_msg() # This is the latest available transform in the buffer.
    #marker.header.stamp = Clock().now().to_msg()# but this fetches the frame at the exact moment (from rclpy.clock import Clock).
    marker.header.frame_id = def_dict.get("frame_id", "map")
    marker.ns = def_dict.get("ns", "default_marker_ns")
    marker.id = def_dict.get("id", 0)
    marker.frame_locked = def_dict.get("frame_locked", False)
    marker.type = def_dict.get("type", Marker.ARROW)
    marker.action = Marker.ADD
    marker.lifetime = Duration(sec=def_dict.get("lifetime_s", 0),
                               nanosec=def_dict.get("lifetime_ns", 0))
    pose = def_dict.get("pose") # [x, y, yaw(quat)]
    marker.pose.position.x, marker.pose.position.y = pose[:2] # z = 0
    marker.pose.orientation = pose[2]
    marker.scale.x, marker.scale.y, marker.scale.z = def_dict.get("scale")
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = def_dict.get("color_rgba")
    return marker