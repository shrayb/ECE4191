class Map:
    def __init__(self, boundary=None, obstacles=None, destinations=None, pickup_location=None):
        self.boundary = boundary
        self.obstacles = obstacles
        self.destinations = destinations
        self.pickup_location = pickup_location
