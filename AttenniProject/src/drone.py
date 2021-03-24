from point import *
class Drone:
    def __init__(self, ID, depot, battery_capacity):
        self.ID = ID
        self.depot = Point(depot, 0)
        self.battery_capacity = int(battery_capacity)
        # trajectory is a list of dict points and position
        self.trajectory = []
        # ordered_trajectory is an ordered list of points
        self.ordered_trajectory = []
    def add_point_in_trajectory(self, target, pos):
        self.trajectory += [{'target': target, 'position':pos}]
    def compute_trajectory(self):
        self.ordered_trajectory = [self.depot]
        for target in sorted(self.trajectory, key=lambda k: k['position']):
            self.ordered_trajectory += [target['target']]
        self.ordered_trajectory += [self.depot]
    def print_trajectory(self):
        print('Trajectory of drone {}: {}'.format(self.ID,[target.ID for target in self.ordered_trajectory]))
