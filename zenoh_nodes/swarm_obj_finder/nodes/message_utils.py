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

