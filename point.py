class Point:
    def __init__(self, x, y, left=False, right=False, front=False, back=False,movment_direction=None,main_movment_direction=None,following_wall_direction=None):
        self.x = x
        self.y = y

        self.inf_left = left
        self.inf_right = right
        self.inf_front = front # up
        self.inf_back = back # down

        self.movment_direction = movment_direction
        self.main_movment_direction = main_movment_direction
        self.following_wall_direction = following_wall_direction

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_inf_left(self):
        return self.inf_left

    def get_inf_right(self):
        return self.inf_right

    def get_inf_front(self):
        return self.inf_front

    def get_inf_back(self):
        return self.inf_back

    def get_movment_direction(self):
        return  self.movment_direction

    def get_wall_direction(self):
        return  self.following_wall_direction

    def get_main_movment_direction(self):
        return  self.main_movment_direction

    def create_from_point(self, other_point):
        new_x = other_point.x
        new_y = other_point.y

        inf_left = other_point.inf_left
        inf_right = other_point.inf_right
        inf_front = other_point.inf_front
        inf_back = other_point.inf_back
        movment_direction = other_point.movment_direction
        following_wall_direction = other_point.following_wall_direction
        main_movment_direction = other_point.main_movment_direction
        return Point(new_x, new_y, inf_left, inf_right, inf_front, inf_back,movment_direction,main_movment_direction,following_wall_direction)

    def __str__(self):
        return f"({self.x}, {self.y})"
