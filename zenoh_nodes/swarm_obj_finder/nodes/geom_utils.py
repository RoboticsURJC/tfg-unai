import math
from geometry_msgs.msg import Quaternion, PoseStamped



def euler2quat(rpy: tuple) -> Quaternion:
    roll, pitch, yaw = rpy
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def quat2euler(q: Quaternion) -> tuple:
    x = math.atan2(2 * (q.w*q.x + q.y*q.z), (1 - 2 * (q.x**2 + q.y**2)))
    y = -math.pi/2 + 2* math.atan2(math.sqrt(1+2*(q.w*q.y-q.x*q.z)), math.sqrt(1-2*(q.w*q.y-q.x*q.z)))
    z = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))
    return (x, y, z)


def get_xy_from_pose(pose: PoseStamped):
    return (pose.pose.position.x, pose.pose.position.y)