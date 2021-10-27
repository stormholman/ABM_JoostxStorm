class Traffic_agent(object):
    def __init__(self, id, x_pos, y_pos, xy_pos, intersection, neighbours, heading):
        self.id = id
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.xy_pos = xy_pos
        self.intersection = intersection
        self.neighbours = neighbours
        self.heading = heading
