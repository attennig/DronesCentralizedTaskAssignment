import sys
import os
import cplex

from data_loader import *
from drone import *
from point import *

def set_objective_function_N(lp_problem, drones, targets):
    # 
    #Min
    lp_problem.objective.set_sense(lp_problem.objective.sense.maximize)
    i = 0
    #variables involved in objective function
    # sum of deltas

    for target in targets:
        var_name ='delta_{}'.format(target.ID)
        lp_problem.variables.add(obj=[1], ub=[1], lb=[0], names=[var_name])
        lp_problem.variables.set_types(i, lp_problem.variables.type.binary)
        i = i + 1
    '''
    for target in targets:
        var_name ='NOTdelta_{}'.format(target.ID)
        lp_problem.variables.add(obj=[1], ub=[1], lb=[0], names=[var_name])
        lp_problem.variables.set_types(i, lp_problem.variables.type.binary)
        i = i + 1
    '''
    '''
    var_name ='T_max'
    lp_problem.variables.add(obj=[-1], lb=[0], names=[var_name])
    lp_problem.variables.set_types(i, lp_problem.variables.type.integer)
    i = i + 1
    '''

    #variables not involved in objective function
    for drone in drones:
        for target_i in targets + [drone.depot]:
            for target_j in targets + [drone.depot]:
                if target_i != target_j:
                    var_name ='x{}_{},{}'.format(drone.ID, target_i.ID, target_j.ID)
                    #trav_cost = target_i.get_cost(target_j)
                    lp_problem.variables.add(obj=[0], ub=[1], lb=[0], names=[var_name])
                    lp_problem.variables.set_types(i, lp_problem.variables.type.binary)
                    i = i + 1
    for drone in drones:
        for target in targets:
            var_name = 'z{}_{}'.format(drone.ID, target.ID)
            lp_problem.variables.add(obj=[0], ub=[len(targets)], lb=[1], names=[var_name])
            lp_problem.variables.set_types(i, lp_problem.variables.type.integer)
            i = i + 1

def set_objective_function_A(lp_problem, drones, targets):
    # implements (a) (g) (e)
    lp_problem.objective.set_sense(lp_problem.objective.sense.maximize)
    i = 0
    for drone in drones:
        for target_i in targets + [drone.depot]:
            for target_j in targets + [drone.depot]:
                if target_i != target_j:
                    var_name ='x{}_{},{}'.format(drone.ID, target_i.ID, target_j.ID)
                    lp_problem.variables.add(obj=[1], ub=[1], lb=[0], names=[var_name])
                    lp_problem.variables.set_types(i, lp_problem.variables.type.binary)
                    i = i + 1
    for drone in drones:
        for target in targets:
            var_name = 'z{}_{}'.format(drone.ID, target.ID)
            lp_problem.variables.add(obj=[0], ub=[len(targets)], lb=[1], names=[var_name])
            lp_problem.variables.set_types(i, lp_problem.variables.type.integer)
            i = i + 1

def add_target_flow_constraints(lp_problem, drones, targets):
    # implements (b)
    for drone in drones:
        for point_j in targets + [drone.depot]:
            var_names=[]
            row_coef=[]
            for point_i in targets + [drone.depot]:
                if (point_i.ID != point_j.ID):
                    var_names += ['x{}_{},{}'.format(drone.ID, point_i.ID, point_j.ID)]
                    row_coef += [1]
            for point_k in targets + [drone.depot]:
                if (point_j.ID != point_k.ID):
                    var_names += ['x{}_{},{}'.format(drone.ID, point_j.ID, point_k.ID)]
                    row_coef += [-1]
            lp_problem.linear_constraints.add(lin_expr=[[var_names, row_coef]], rhs=[0], names=['c_d_{}_in_out_{}'.format(drone.ID, point_j.ID)])

def add_depot_constraints(lp_problem, drones, targets):
    # implements (c)
    for drone in drones:
        var_names=[]
        row_coef=[]
        for target in targets:
            #sumj xu_duj = 1
            var_names += ['x{}_{},{}'.format(drone.ID, drone.depot.ID, target.ID)]
            row_coef += [1]
        lp_problem.linear_constraints.add(lin_expr=[[var_names, row_coef]], rhs=[1], names=['c_depot_{}'.format(drone.ID)])

def add_no_cycles_constraints(lp_problem, drones, targets):
    # implements (d)  
    for drone in drones:
        for target_i in targets:
            for target_j in targets:
                if target_i.ID != target_j.ID:
                    var_names=[]
                    row_coef=[]
                    var_names += ['z{}_{}'.format(drone.ID, target_j.ID)]
                    row_coef += [1]
                    var_names += ['z{}_{}'.format(drone.ID, target_i.ID)]
                    row_coef += [-1]
                    var_names += ['x{}_{},{}'.format(drone.ID, target_i.ID, target_j.ID)]
                    row_coef += [float(-(1+len(targets)))]
                    kterm = float(-(len(targets)))
                    lp_problem.linear_constraints.add(lin_expr=[[var_names, row_coef]], senses='G', rhs=[kterm], names=['c_{}_{}follows{}'.format(drone.ID, target_i.ID, target_j.ID)])

def add_trajectory_constraints(lp_problem, drones, targets):
    # (b)
    add_target_flow_constraints(lp_problem, drones, targets)
    # (c)
    add_depot_constraints(lp_problem, drones, targets)
    # (d)
    add_no_cycles_constraints(lp_problem, drones, targets)

def add_energy_consumption_constraints(lp_problem, drones, targets):
    # implements (f)
    for drone in drones:
        var_names=[]
        row_coef=[]
        for target_i in targets + [drone.depot]:
            for target_j in targets + [drone.depot]:
                if target_i.ID != target_j.ID:
                    var_names += ['x{}_{},{}'.format(drone.ID, target_i.ID, target_j.ID)]
                    row_coef += [target_i.get_cost(target_j)]
        lp_problem.linear_constraints.add(lin_expr=[[var_names, row_coef]], rhs=[drone.battery_capacity], senses='L', names=['c_battery_{}'.format(drone.ID)])

def add_constraint_on_delta(lp_problem, drones, targets):
    for target_i in targets:
        var_names = []
        var_coef = []
        var_names += ['delta_{}'.format(target_i.ID)]
        var_coef += [1]
        for drone in drones:
            for target_j in targets + [drone.depot]:
                if target_i.ID != target_j.ID:
                    var_names += ['x{}_{},{}'.format(drone.ID, target_i.ID, target_j.ID)]
                    var_coef += [-1]
        lp_problem.linear_constraints.add(lin_expr=[[var_names, var_coef]], senses='L', rhs=[0], names=['c_delta_{}'.format(target_i.ID)])
    '''
    for target in targets:
        var_names = ['NOTdelta_{}'.format(target.ID), 'delta_{}'.format(target.ID)]
        var_coef = [1, 1]
        lp_problem.linear_constraints.add(lin_expr=[[var_names, var_coef]], rhs=[1], names=['c_NOTdelta_{}'.format(target.ID)])
    '''

def add_time_constraint(lp_problem, drones, targets):
    for drone in drones:
        var_names = []
        var_coef = []
        var_names += ['T_max']
        var_coef += [1]
        for target_i in targets:
            for target_j in targets + [drone.depot]:
                if target_i.ID != target_j.ID:
                    var_names += ['x{}_{},{}'.format(drone.ID, target_i.ID, target_j.ID)]
                    var_coef += [-target_i.inspection_time]
        lp_problem.linear_constraints.add(lin_expr=[[var_names, var_coef]], senses='G', rhs=[0], names=['c_time_{}'.format(drone.ID)])

def set_up_modelA(lp_problem, drones, targets):
    # (a) (g) (e)
    set_objective_function_A(lp_problem, drones, targets)
    # (b) (c) (d)
    add_trajectory_constraints(lp_problem, drones, targets)
    # (f)
    add_energy_consumption_constraints(lp_problem, drones, targets)
def set_up_modelN(lp_problem, drones, targets): 
    set_objective_function_N(lp_problem, drones, targets)
    add_trajectory_constraints(lp_problem, drones, targets)
    add_energy_consumption_constraints(lp_problem, drones, targets)
    add_constraint_on_delta(lp_problem, drones, targets)
    #add_time_constraint(lp_problem, drones, targets)

def extract_solution(lp_problem, drones, targets):
    print("Solution value  = ", lp_problem.solution.get_objective_value())
    numcols = lp_problem.variables.get_num()
    values = lp_problem.solution.get_values()
    #print(values)
    names = lp_problem.variables.get_names()
    #print(names)
    results_x = []
    results_z = []
    for j in range(numcols):
        if names[j][0] == 'x' and values[j] != 0:
            results_x += [{'name': names[j], 'value': values[j]}]
        if names[j][0] == 'z':
            results_z += [{'name': names[j], 'value': values[j]}]
    for result_x in results_x:
        drone_id = result_x['name'].split('_')[0][1]
        target_id = result_x['name'].split('_')[1].split(',')[0]
        if target_id[0] != 'p': 
            target_pos = [res['value'] for res in results_z if res['name'] == 'z{}_{}'.format(drone_id, target_id)][0]
            drone = [drone for drone in drones if drone.ID == drone_id][0]
            target = [target for target in targets if target.ID == target_id][0]
            drone.add_point_in_trajectory(target, target_pos)    
    for drone in drones:
        drone.compute_trajectory()
        drone.print_trajectory()

def milp(drones, targets, depots, model):
    lp_problem = cplex.Cplex()
    if model == 'A':
        set_up_modelA(lp_problem, drones, targets)
    elif model == 'N':
        set_up_modelN(lp_problem, drones, targets)
    lp_problem.write('plan.lp')
    lp_problem.solve()
    extract_solution(lp_problem, drones, targets)

def usage():
    print("Usage: project.py folder model")
    print("\twhere\n\t\tfolder is the name of some folder present in ../data/\n\t\tmodel is the letter 'A' (model A in assignment) or 'N' (new model)")
    print(" Exiting...")
    sys.exit(-1)

if __name__ == "__main__":
    if len(sys.argv) != 3 or str(sys.argv[1]) not in os.listdir('../data/') or (str(sys.argv[2]) != 'A' and str(sys.argv[2]) != 'N'):
        usage()

    folder = str(sys.argv[1])
    input_path = '../data/{}/'.format(folder)
    model = str(sys.argv[2])

    #load data
    DRONES = load_drones('{}drones.in'.format(input_path))
    POINTS = []

    DEPOTS = []
    for drone in DRONES:
        DEPOTS += [drone.depot]
    POINTS += DEPOTS

    TARGETS = load_targets('{}targets.in'.format(input_path))
    POINTS += TARGETS

    load_costs('{}travelling_costs.in'.format(input_path), POINTS)


    milp(DRONES, TARGETS, DEPOTS, model)

    
