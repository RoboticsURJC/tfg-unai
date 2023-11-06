from geometry_msgs.msg import PoseStamped



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
        self.namespaces = namespaces
        # A PoseStamped object and a bool that indicates if that namespace has
        # reached the object for each namespace in a dictionary:
        self.goals_reached = dict(
            zip(self.namespaces, [False] * len(self.namespaces))
            )
        self.master = None

    def master_exists(self) -> bool:
        return self.master != None

    def set_master(self, name: str) -> None:
        self.master = name

    def free_master(self) -> None:
        self.master = None

    def get_master(self) -> str:
        return self.master

    def has_reached_goal(self, name: str) -> bool:
        return self.goals_reached.get(name, None)

    def set_reached(self, name: str) -> None:
        self.goals_reached[name] = True

