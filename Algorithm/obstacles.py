class Obstacle:
    def __init__(self, location, direction):
        self.location = location
        self.direction = direction

    def get_location(self):
        return self.location

    def get_direction(self):
        return self.direction

    def set_location(self, location):
        self.location = location

    def set_direction(self, direction):
        self.direction = direction