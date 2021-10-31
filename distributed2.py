import time as timer
import heapq
import random
import copy
from single_agent_planner import simple_single_agent_astar

def run_distributed_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, traffic_agents):
    constraints = []
    for ac1 in aircraft_lst:
        observations = []
        if ac1.spawntime == t or ac1.departtime == t:
            ac1.position = nodes_dict[ac1.start]["xy_pos"]
            ac1.plan_independent(nodes_dict, edges_dict, heuristics, t)
        # Create awareness
        for ac2 in aircraft_lst:
            if ac1 != ac2:
                if ac1.position and ac1.fieldofview and ac2.position and ac2.fieldofview:
                    if ac1.heading == 0:
                        if 0 < ac2.position[0] - ac1.position[0] <= ac1.fieldofview[0][1] - ac1.position[1] + 0.1:
                            if abs(ac2.position[1] - ac1.position[1] / abs(ac2.position[0] - ac1.position[0])) <= (1.5/1.5):
                                ac1.observations.append(ac2)
                    if ac1.heading == 270:
                        if 0 < ac2.position[0] - ac1.position[0] <= ac1.fieldofview[0][0] - ac1.position[0] + 0.1:
                            if abs(ac2.position[1] - ac1.position[1]) / abs(ac2.position[0] - ac1.position[0]) <= (1.5/1.5):
                                ac1.observations.append(ac2)
                    if ac1.heading == 180:
                        if 0 < ac1.position[1] - ac2.position[1] <= ac1.position[1] - ac1.fieldofview[0][1] + 0.1:
                            if abs(ac2.position[0] - ac1.position[0] / abs(ac1.position[1] - ac2.position[1])) <= (1.5/1.5):
                                ac1.observations.append(ac2)
                    if ac1.heading == 90:
                            # print(ac1.position[0] - ac2.position[0])
                            # print("<=")
                            # print(ac1.position[0] - ac1.fieldofview[0][0] + 0.1)
                            # print(0 < ac1.position[0] - ac2.position[0] <= ac1.position[0] - ac1.fieldofview[0][0] + 0.1)

                        if 0 < ac1.position[0] - ac2.position[0] <= ac1.position[0] - ac1.fieldofview[0][0] + 0.1:
                            if abs(ac2.position[1] - ac1.position[1]) / abs(ac1.position[0] - ac2.position[0]) <= (1.5/1.5):
                                ac1.observations.append(ac2)

        nextintersection = find_next_intersection(ac1)
        nextlinkage = find_next_linkage(ac1)

        #intersection_id = nextintersection['id']

        for observation in ac1.observations:
            if nextintersection: # if ac1 has a next intersection
                if nextintersection == find_next_intersection(observation):
                    print(ac1.id, ac2.id, "intersection conflict at", nextintersection)
                    # print(ac1.path_to_goal)
                    # print(observation.path_to_goal)
                    # print(nextintersection)
                    # print(find_next_intersection(observation))
                    # print("found intersection conflict between ac", ac1.id, "and", observation.id, 'at', nextintersection['id'])

                    ac1_can_detour = run_astar(ac1, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t,
                               [{'agent': ac1.id, 'loc': nextintersection['id'], 'timestep': t + 0.5},
                                {'agent': ac1.id, 'loc': nextintersection['id'], 'timestep': t + 1}],
                              ac1.lastdifferentnode, False)

                    ac2_can_detour = run_astar(ac2, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t,
                                               [{'agent': ac2.id, 'loc': nextintersection['id'], 'timestep': t + 0.5},
                                                {'agent': ac2.id, 'loc': nextintersection['id'], 'timestep': t + 1}],
                                       ac1.lastdifferentnode, False)

                    if ac1_can_detour == False or ac2_can_detour == False:
                        if ac1_can_detour:
                            ac1 = loser
                            ac2 = winner
                        elif ac2_can_detour:
                            ac2 = loser
                            ac1 = winner
                        else:
                            print("Neither", ac1.id, ac1.position, "nor", ac2.id, ac2.position, "can detour for conflict at", nextintersection)

                    else:
                        winner, loser = determine_right_of_way(ac1, observation, nextintersection)
                    # print(loser.id, 'giving way')
                    constraints.append({'agent': loser.id, 'loc': nextintersection['id'], 'timestep': t + 0.5})
                    constraints.append({'agent': loser.id, 'loc': nextintersection['id'], 'timestep': t + 1})

                    # print("Added constraint:", {'agent': loser.id, 'loc': nextintersection['id'], 'timestep': t + 1})
                    # print(constraints)

                    run_astar(loser, nodes_dict, loser.from_to[0], loser.goal, heuristics, t, constraints, loser.lastdifferentnode)

                    nextintersection = find_next_intersection(ac1)
                    nextlinkage = find_next_linkage(ac1)

            if nextlinkage: # if ac1 has a next intersection
                if nextlinkage == find_next_linkage(observation):
                    print(ac1.id, ac2.id, "linkage conflict at", nextlinkage)
                    # print(ac1.path_to_goal)
                    # print(observation.path_to_goal)
                    # print(nextlinkage)
                    # print(find_next_linkage(observation))
                    # print("found linkage conflict between ac", ac1.id, "and", observation.id, 'at', nextlinkage['id'])

                    ac1_can_detour = run_astar(ac1, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, constraints,
                              ac1.lastdifferentnode, False)

                    ac2_can_detour = run_astar(ac2, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, constraints,
                                       ac1.lastdifferentnode, False)

                    if ac1_can_detour == False or ac2_can_detour == False:
                        if ac1_can_detour:
                            ac1 = loser
                            ac2 = winner
                        elif ac2_can_detour:
                            ac2 = loser
                            ac1 = winner
                        else:
                            print("Neither", ac1.id, "nor", ac2.id,  "can detour for conflict at", nextlinkage)
                    else:
                        winner, loser = determine_right_of_way(ac1, observation, nextlinkage)
                    # print(loser.id, 'giving way')
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 0.5})
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 1})
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 1.5})
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 2})
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 2.5})
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 3})

                    # print("added constraint:", {'agent': winner.id, 'loc': nextlinkage['id'], 'timestep': t + 0.5})
                    # print("added constraint:", {'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 0.5})
                    # print(constraints)

                    run_astar(winner, nodes_dict, winner.from_to[0], winner.goal, heuristics, t, constraints, winner.lastdifferentnode)

                    run_astar(loser, nodes_dict, loser.from_to[0], loser.goal, heuristics, t, constraints, loser.lastdifferentnode)

                    nextintersection = find_next_intersection(ac1)
                    nextlinkage = find_next_linkage(ac1)


        # append to memory. Add weights to expected paths of remembered aircraft. Resolve conflicts that may occur

        # for otheraircraft in aircraft_lst:
        #     if ac != otheraircraft:
        #         if ac.heading == 0:
        #             if otheraircraft.position[1] > ac.position[1]:
        #                 if otheraircraft.position[1] - ac.position[1]

def determine_right_of_way(ac1, ac2, conflictnode):
    #print("Determining right of way between", ac1.id, "and", ac2.id)
    # print(ac1.path_to_goal)
    # print(conflictnode)
    # print([node for node in ac1.path_to_goal if node[0] == conflictnode['id']])
    node_ac1 = [node for node in ac1.path_to_goal if node[0] == conflictnode['id']][0]
    node_ac2 = [node for node in ac2.path_to_goal if node[0] == conflictnode['id']][0]
    index1 = ac1.path_to_goal.index(node_ac1)
    index2 = ac2.path_to_goal.index(node_ac2)

    if node_ac1[0] == 11 or node_ac1[0] == 12:  # give priority to aircraft leaving runways
        if ac1.id < ac2.id:
            return ac2, ac1 # winner, loser
        else:
            return ac1, ac2

    if index1 == index2:
        if ac1.id < ac2.id:
            return ac1, ac2 # winner, loser
        else:
            return ac2, ac1
    else:
        if index1 < index2:
            return ac1, ac2 # winner, loser
        else:
            return ac2, ac1

# def appendobservation(ac1,ac2):
#     ac1.observations.append({'agent': ac2.id,
#                              'loc': ac2.from_to[0],
#                              'heading': ac2.heading,
#                              'position': ac2.position})

def run_astar(ac, nodes_dict, from_to, goal, heuristics, t, constraints, lastdifferentnode, changepath = True):
    success, path = simple_single_agent_astar(ac.id, nodes_dict, from_to, goal, heuristics, t,
                                              constraints, lastdifferentnode)
    if success and changepath:
        ac.total_path = path
        ac.path_to_goal = path[1:]
        next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
        ac.from_to = [path[0][0], next_node_id]
        print("New path path for AC", ac.id, ":", path)

    return success

def find_next_intersection(ac):
    # print(ac.id)
    for node in ac.path_to_goal:
        if ac.nodes_dict[node[0]]['type'] == "intersection":
            # print(node[0])
            # print(ac.nodes_dict[node[0]])
            if ac.nodes_dict[node[0]]['id'] == node[0]:
                return ac.nodes_dict[node[0]]

def find_next_linkage(ac):
    # print(ac.id)
    for node in ac.path_to_goal:
        if ac.nodes_dict[node[0]]['type'] == "between":
            # print(node[0])
            # print(ac.nodes_dict[node[0]])
            if ac.nodes_dict[node[0]]['id'] == node[0]:

                return ac.nodes_dict[node[0]]

def resolveconflicts():
    pass