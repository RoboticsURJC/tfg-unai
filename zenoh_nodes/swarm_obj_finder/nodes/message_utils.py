from geometry_msgs.msg import PoseStamped
import time


class CentroidMessage:
    def __init__(self, x:float=0.0, y:float=0.0, z:float=0.0,
                 founder:str="") -> None:
        self.x = x
        self.y = y
        self.z = z
        self.found_by = founder


    def set_centroid(self, x:float, y:float, z:float):
        self.x = x
        self.y = y
        self.z = z

    def get_centroid(self):
        return (self.x, self.y, self.z)


    def set_founder(self, founder: str):
        self.found_by = founder

    def get_founder(self):
        return self.found_by



class WorldPosition:
    def __init__(self, world_pos:PoseStamped=PoseStamped(), name:str="") -> None:
        self.world_position = world_pos
        self.sender = name


    def set_world_position(self, world_pos:PoseStamped=PoseStamped()) -> None:
        self.world_position = world_pos

    def get_world_position(self) -> PoseStamped:
        return self.world_position


    def set_sender(self, name:str="") -> None:
        self.sender = name

    def get_sender(self) -> str:
        return self.sender


class GoalManager:
    def __init__(self, namespaces: list):
        # For each namespace in the dictionary a bool and a timestamp are
        # assigned that indicates if that namespace has reached the object and
        # the last time it was seen respectively:
        self.goals_info = dict()
        for ns in namespaces:
            self.goals_info.update({ns: [False, None]})
        self.master = None

    def __str__(self) -> str:
        return str(self.goals_info)

    def master_exists(self) -> bool:
        return self.master != None

    def set_master(self, name: str) -> None:
        self.master = name

    def free_master(self) -> None:
        self.master = None

    def get_master(self) -> str:
        return self.master

    def has_reached_goal(self, name: str) -> bool:
        return self.goals_info.get(name)[0]

    def set_reached(self, name: str) -> None:
        self.goals_info[name][0] = True

    def update_ts(self, name: str) -> None:
        self.goals_info[name][1] = time.time()

    def get_ts(self, name: str) -> float:
        return self.goals_info[name][1]


def get_ns_index(name):
    word = list(name)
    word.reverse()
    for i, char in enumerate(word):
        if not char.isdigit():
            return int(name[-i:])

def trace(node_name: str, text: str):
    print(f"{round(time.time(), 3)}\t| {node_name}\t| {text}")
