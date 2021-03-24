from drone import *
from point import *
def load_drones(input_path):
    drones = []
    file_content = open(input_path)
    content = file_content.read()
    lines = content.splitlines()[1:]
    for line in lines:
        ID, dep, b = line.split(', ')
        drones.append(Drone(ID, dep, b))
    return drones

def load_targets(input_path):
    target_points = []
    file_content = open(input_path)
    content = file_content.read()
    lines = content.splitlines()[1:]
    for line in lines:
        ID, time = line.split(', ')
        target_points.append(Point(ID, time))
    return target_points

def load_costs(input_path, points):
    file_content = open(input_path)
    content = file_content.read()
    lines = content.splitlines()[1:]
    for line in lines:
        src, dst, cost = line.split(', ')
        target_src = [t for t in points if t.ID == src][0]
        target_dst = [t for t in points if t.ID == dst][0]
        target_src.add_travelling_cost(target_dst, cost)
        target_dst.add_travelling_cost(target_src, cost)