from geometry_msgs.msg import PoseStamped



class CentroidMessage:
    def __init__(self, x:float=0.0, y:float=0.0, founder:str="") -> None:
        self.x = x
        self.y = y
        self.found_by = founder


    def set_centroid(self, x:float, y:float):
        self.x = x
        self.y = y

    def get_centroid(self):
        return (self.x, self.y)


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

    def get_world_position(self):
        return self.world_position


    def set_sender(self, name:str="") -> None:
        self.sender = name

    def get_sender(self):
        return self.sender
