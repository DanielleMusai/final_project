class Point:
    def __init__(self, x, y, left=False, right=False, front=False, back=False):
        self.x = x
        self.y = y

        self.inf_left = left
        self.inf_right = right
        self.inf_front = front
        self.inf_back = back

    def create_from_point(self, other_point):
        new_x = other_point.x
        new_y = other_point.y

        inf_left = other_point.inf_left
        inf_right = other_point.inf_right
        inf_front = other_point.inf_front
        inf_back = other_point.inf_back
        return Point(new_x, new_y, inf_left, inf_right, inf_front, inf_back)

    def __str__(self):
        return f"({self.x}, {self.y})"
