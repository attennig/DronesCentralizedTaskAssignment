class Point:
    def __init__(self, ID, inspection_time):
        self.ID = ID
        self.inspection_time = int(inspection_time)
        self.travelling_costs = []
           
    def add_travelling_cost(self, dest, cost):
        self.travelling_costs += [(dest, int(cost))]

    def get_cost(self, dest, a=1.0, b=1.0):
        # this method computes w_ij = a F_i + b l_ij
        # where F_i is the inspection time of i (self)
        # and l_ij is the travelling cost from i (self) to j (dest)
        return a*self.inspection_time + b*[tc[1] for tc in self.travelling_costs if tc[0].ID == dest.ID][0]
