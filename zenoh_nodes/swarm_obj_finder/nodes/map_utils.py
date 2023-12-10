import cv2, numpy as np
from math import pi

from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock

from cv_bridge import CvBridge
from comms_utils import *
from geom_utils import *



def factorize(n: int) -> list:
    prime_numbers = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41,
                        43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97]
    factors = []
    if n < 2:
        return factors
    
    for p in prime_numbers:
        while n % p == 0:
            factors.append(p)
            n /= p
        if (n == 1):
            break
    return factors

def get_squarest_distribution(factors: list) -> list:
    distribution = [1, 1]
    for f in reversed(factors): # Starts from the highest to the lowest.
        if distribution[0] < distribution[1]:
            # The factor is multiplied by the lowest member of the distribution:
            distribution[0] *= f
        else:
            distribution[1] *= f
    return distribution

def get_division_shape(div_num) -> list:
    factors = factorize(div_num)

    if len(factors) == 0: # For only one robot
        return [1, 1]
    elif len(factors) == 1:
        factors.append(1)
        return factors
    elif len(factors) == 2:
        return factors
    else: # More than one single configuration
        return get_squarest_distribution(factors)

def get_map_upper_bound(itpr_map_img: np.ndarray, thresholds: tuple) -> int:
    height, width = itpr_map_img.shape
    for j in range(height):
        for i in range(width):
            if not (thresholds[0] <
                    itpr_map_img[j, i] <
                    thresholds[1]):
                return j            

def get_map_lower_bound(itpr_map_img: np.ndarray, thresholds: tuple) -> int:
    height, width = itpr_map_img.shape
    for j in range(height-1, 0, -1):
        for i in range(width):
            if not (thresholds[0] <
                    itpr_map_img[j, i] <
                    thresholds[1]):
                return j

def get_map_left_bound(itpr_map_img: np.ndarray, thresholds: tuple) -> int:
    height, width = itpr_map_img.shape
    for i in range(width):
        for j in range(height):
            if not (thresholds[0] <
                    itpr_map_img[j, i] <
                    thresholds[1]):
                return i

def get_map_right_bound(itpr_map_img: np.ndarray, thresholds: tuple) -> int:
    height, width = itpr_map_img.shape
    for i in range(width-1, 0, -1):
        for j in range(height):
            if not (thresholds[0] <
                    itpr_map_img[j, i] <
                    thresholds[1]):
                return i
    
def divide_map(itpr_map_img: np.ndarray, map_img: np.ndarray,
               div_num: int, thresholds: tuple,
               debug: bool) -> tuple:
    left_top_point = [
        get_map_left_bound(itpr_map_img, thresholds),
        get_map_upper_bound(itpr_map_img, thresholds)]
    right_bot_point = [
        get_map_right_bound(itpr_map_img, thresholds),
        get_map_lower_bound(itpr_map_img, thresholds)]
    reduced_width = right_bot_point[0] - left_top_point[0]
    reduced_height = right_bot_point[1] - left_top_point[1]

    division_shape = get_division_shape(div_num)
    if reduced_width > reduced_height:
        if division_shape[0] > division_shape[1]:
            division_shape = list(reversed(division_shape))
    else:
        if division_shape[0] < division_shape[1]:
            division_shape = list(reversed(division_shape))

    x_shift = reduced_width / division_shape[0]
    y_shift = reduced_height / division_shape[1]
    bboxes = [] # Map divided into N bboxes:
    for i in range(division_shape[0]):
        for j in range(division_shape[1]):
            bboxes.append([(round(left_top_point[0] + x_shift * i),
                            round(left_top_point[1] + y_shift * j)),
                            (round(left_top_point[0] + x_shift * (i+1)),
                            round(left_top_point[1] + y_shift * (j+1)))])

    ### DEBUG:
    if debug:
        debug_div_img = np.array(map_img) # Copy the img.
        for i in range(division_shape[0] + 1):
            cv2.line(debug_div_img,
                        [round(left_top_point[0] + (x_shift*i)),
                        round(left_top_point[1])],
                        [round(left_top_point[0] + (x_shift*i)),
                        round(left_top_point[1] + reduced_height)],
                        0, 1)
        for j in range(division_shape[1] + 1):
            cv2.line(debug_div_img,
                        [round(left_top_point[0]),
                        round(left_top_point[1] + y_shift*j)],
                        [round(left_top_point[0] + reduced_width),
                        round(left_top_point[1] + y_shift*j)],
                        0, 1)
        for bbox in bboxes:
            for point in bbox:
                cv2.circle(debug_div_img, point, 2, 0, 2, -1)
        for i in range(division_shape[0] + 1):
            for j in range(division_shape[1] + 1):
                cv2.circle(debug_div_img,
                            (round(left_top_point[0] + (x_shift*i)),
                            round(left_top_point[1] + y_shift*j)),
                            2, 0, 2, -1)
        cv_bridge = CvBridge()
        debug_img_msg = cv_bridge.cv2_to_imgmsg(debug_div_img)
        return (bboxes, debug_img_msg)
    ###

    return bboxes

def map2world(map_pose: PoseStamped, origin: list, res: int) -> PoseStamped:
    world_pose = PoseStamped()
    world_pose.header.frame_id = "map" # Header needed for Nav2 planner server.
    world_pose.header.stamp = Clock().now().to_msg()
    world_pose.pose.position.x = (map_pose.pose.position.x + origin[0]) * res
    world_pose.pose.position.y = (map_pose.pose.position.y + origin[1]) * res
    world_pose.pose.orientation = map_pose.pose.orientation
    return world_pose

def img2map(img_pix: tuple, img_shape: tuple) -> tuple:
    img_x, img_y = img_pix
    img_height, img_width = img_shape
    map_pos = (float(img_x - (img_width/2)),
                float(-(img_y - (img_height/2))))
    return map_pos

def is_near_wall(point: tuple, itpr_map_img: np.ndarray,
                 margin: int, occupied_thresh: int) -> bool:
    x, y = point
    width, height = itpr_map_img.shape
    for i in range(max(x - margin, 0), min(x + margin, width)):
        for j in range(max(y - margin, 0), min(y + margin, height)):
            if itpr_map_img[j, i] > occupied_thresh:
                return True
    return False

def get_path_from_area(area: list, itpr_map_img: np.ndarray, thresholds: tuple,
                       wp_world_separation: float, wp_safe_space: int,
                       origin: list, resolution: int, inverted=False) -> list:
    p1, p2 = area
    vertical_range = list(range(int(p1[1]),
                                int(p2[1]),
                                round(wp_world_separation)))
    orientations = [(0, 0, 3*pi/2), (0, 0, pi/2)]
    if inverted:
        vertical_range.reverse()
    ori_index = 0
    #new_img = self.map_img ### DEBUG

    path = list()
    for x in range(int(p1[0]), int(p2[0]), round(wp_world_separation)):
        for y in vertical_range:
            wp = PoseStamped()
            wp.header.frame_id = "map"
            wp.pose.position.x, wp.pose.position.y = img2map((x, y),
                                                             itpr_map_img.shape)
            wp.pose.orientation = euler2quat(
                orientations[ori_index % len(orientations)]
                )
            
            #cv2.circle(new_img, (x, y), 2, 0, -1) ### DEBUG
            if (itpr_map_img[y, x] < thresholds[0] and
                not is_near_wall((x, y), itpr_map_img,
                                 wp_safe_space, thresholds[1])):
                path.append(map2world(wp, origin, resolution))
        vertical_range.reverse()
        ori_index += 1
    #cv2.imwrite("/tmp/test_points_map_img.png", new_img) ### DEBUG
    if inverted:
        path.reverse()
    return path
