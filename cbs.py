from independent import run_independent_planner
from single_agent_planner import simple_single_agent_astar
import heapq
from random import random


def run_CBS(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)

    root = {'cost': 0,
            'constraints': [],
            'paths': [ac.total_path for ac in aircraft_lst],
            'collisions': 0}

    root['cost'] = get_sum_of_cost(root['paths'])
    root['collisions'] = detect_collisions(root['paths'])

    open_list = []
    push_node(open_list, root)

    while open_list:
        p = pop_node(open_list)
        if not p['collisions']:
            for ac in aircraft_lst:
                if ac.status == "taxiing":
                    ac.total_path = p['paths'][ac.id]
                    ac.path_to_goal = ac.total_path[1:]
                    next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                    ac.from_to = [ac.total_path[0][0], next_node_id]
            return

        constraints = []
        # for ac1 in aircraft_lst:
        #     for ac2 in aircraft_lst:
        #         if ac1 != ac2:
        #             if ac1.position == (1.5, 5) or ac1.position == (
        #             1.5, 4):  # don't let aircraft depart at the same time from rw 1 or 2
        #                 if ac2.position == (1.5, 5) or ac2.position == (1.5, 4):
        #                     if ac1.weight_class == "heavy" and ac2.weight_class == "small":  # smallest aircraft gives way
        #                         constraints.append({'agent': ac1.id, 'loc': 1, 'timestep': t + 0.5})
        #                         constraints.append({'agent': ac1.id, 'loc': 2, 'timestep': t + 0.5})
        #                     elif ac1.weight_class == "small" and ac2.weight_class == "heavy":
        #                         constraints.append({'agent': ac2.id, 'loc': 1, 'timestep': t + 0.5})
        #                         constraints.append({'agent': ac2.id, 'loc': 2, 'timestep': t + 0.5})
        #                     else:
        #                         if ac1.id < ac2.id:  # largest id gives way
        #                             constraints.append({'agent': ac2.id, 'loc': 1, 'timestep': t + 0.5})
        #                             constraints.append({'agent': ac2.id, 'loc': 2, 'timestep': t + 0.5})
        #                         else:
        #                             constraints.append({'agent': ac1.id, 'loc': 1, 'timestep': t + 0.5})
        #                             constraints.append({'agent': ac1.id, 'loc': 2, 'timestep': t + 0.5})
        #                     print("constr", constraints)
        #                     # simple_single_agent_astar(ac1, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, constraints,
        #                     #           ac1.lastdifferentnode)
        #                     # simple_single_agent_astar(ac2, nodes_dict, ac2.from_to[0], ac2.goal, heuristics, t, constraints,
        #                     #           ac2.lastdifferentnode)
        #
        #             if ac1.position == (
        #             1.5, 4):  # small ac waits 0.5 sec for vertex turbulence to disappear caused by heavy ac
        #                 if ac2.position == (2, 5) and ac2.path_to_goal[0][0] == 95:
        #                     constraints.append({'agent': ac2.id, 'loc': 1, 'timestep': t + 1})
        #                     # simple_single_agent_astar(ac2, nodes_dict, ac2.from_to[0], ac2.goal, heuristics, t, constraints,
        #                     #           ac2.lastdifferentnode)
        #                     print("Aircraft", ac2.id, "waits due to heavy vortex from aircraft", ac1.id)

        for collision in p['collisions']:
            (constraint1, constraint2) = (standard_splitting(collision))
            constraints.append(constraint1)
            constraints.append(constraint2)

            for constraint in constraints:
                q = {'cost': 0, 'constraints': list(p['constraints']), 'paths': [], 'collisions': []}
                q['constraints'].append(constraint)

                for path in p['paths']:
                    if len(path) > 1:
                        q['paths'].append(path)

                ac = aircraft_lst[constraint['agent']]

                if ac.from_to[0] != 0:
                    from_node = ac.from_to[0]
                else:
                    from_node = ac.start

                succes, path = simple_single_agent_astar(ac.id, nodes_dict, from_node, ac.goal, heuristics, t,
                                                         q['constraints'])
                if succes:
                    q['paths'][ac.id] = list(path)
                    q['cost'] = get_sum_of_cost(q['paths'])
                    q['collisions'] = detect_collisions(q['paths'])
                    push_node(open_list, q)
                else:
                    print("No solution")
    print("No solution")
    return


def standard_splitting(collision):
    if isinstance(collision['loc'], list):
        # [{'agent': 1, 'loc': [74.0, 24.0], 'timestep': 4.0}, {'agent': 4, 'loc': [24.0, 74.0], 'timestep': 4.0}]

        constraint1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['time']}
        constraint2 = {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]],
                       'timestep': collision['time']}
    else:
        constraint1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['time']}
        constraint2 = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['time']}

    return constraint1, constraint2


def detect_collisions(paths):
    totalCollisions = []
    for i in range(len(paths)):
        for j in range(i + 1, len(paths)):
            firstCollision = detect_first_collision(paths[i], paths[j])
            if firstCollision:
                if isinstance(firstCollision, list):
                    totalCollisions.append({'a1': i, 'a2': j,
                                            'loc': [firstCollision[0][0], firstCollision[1][0]],
                                            'time': firstCollision[1][1]})
                else:
                    totalCollisions.append({'a1': i, 'a2': j,
                                            'loc': firstCollision[0],
                                            'time': firstCollision[1]})
    return totalCollisions


def detect_first_collision(path1, path2):
    for timestep in range(min(len(path1), len(path2))):
        if path1[timestep] == path2[timestep]:
            return path1[timestep]
        elif timestep < min(len(path1), len(path2)) - 1:
            # print('checking:')
            # print(path1[timestep], path2[timestep + 1])
            # print(path1[timestep + 1], path2[timestep])
            if path1[timestep][0] == path2[timestep + 1][0] and path1[timestep + 1][0] == path2[timestep][0]:
                return [path1[timestep], path1[timestep + 1]]
    return None


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def push_node(open_list, node):
    heapq.heappush(open_list, (len(node['collisions']), node['cost'], len(node['constraints']), random(), node))


def pop_node(open_list):
    _, _, _, _, node = heapq.heappop(open_list)
    return node
