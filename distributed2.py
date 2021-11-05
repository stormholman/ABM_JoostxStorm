import time as timer
import heapq
import random
import copy
from single_agent_planner import simple_single_agent_astar

def run_distributed_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, traffic_agents):
    constraints = []

    for ac1 in aircraft_lst:
        if ac1.status != "departed":
            ac1.observations = []
            if ac1.spawntime == t:
                ac1.position = nodes_dict[ac1.start]["xy_pos"]
                ac1.plan_independent(nodes_dict, edges_dict, heuristics, t)

            if ac1.departtime == t:
                ac1.plan_independent(nodes_dict, edges_dict, heuristics, t)

            # Create awareness
            nodes_in_view= set()
            for neighbor in ac1.nodes_dict[ac1.from_to[1]]["neighbors"]:
                if neighbor != ac1.from_to[0] and neighbor != ac1.from_to[1]:
                    nodes_in_view.add(neighbor)
                    for nextneighbor in ac1.nodes_dict[neighbor]["neighbors"]:
                        if nextneighbor not in nodes_in_view:
                            nodes_in_view.add(nextneighbor)

            for ac2 in aircraft_lst:
                if ac2.status != "departed":
                    if ac1 != ac2:
                        if ac2.from_to[0] in nodes_in_view:
                            ac1.observations.append(ac2)
                        if ac2 not in ac1.observations:
                            if ac1.position and ac1.fieldofview and ac2.position and ac2.fieldofview:
                                if in_view(ac1.position, ac1.fieldofview[0], ac1.fieldofview[1], ac2.position):
                                    ac1.observations.append(ac2)
                            elif ac2.from_to[0] == ac1.goal: #ac1 knows if gate is occupied
                                ac1.observations.append(ac2)

                        # if ac1.position == (1.5, 5) or ac1.position == (1.5, 4): # don't let aircraft depart avyt the same time from rw 1 or 2
                        #     if ac2.position == (1.5, 5) or ac2.position == (1.5, 4):
                        #         if ac1.weight_class == "Heavy" and ac2.weight_class == "Small":  # smallest aircraft gives way
                        #             constraints.append({'agent': ac1.id, 'loc': 1, 'timestep': t + 0.5})
                        #             constraints.append({'agent': ac1.id, 'loc': 2, 'timestep': t + 0.5})
                        #         elif ac1.weight_class == "Small" and ac2.weight_class == "Heavy":
                        #             constraints.append({'agent': ac2.id, 'loc': 1, 'timestep': t + 0.5})
                        #             constraints.append({'agent': ac2.id, 'loc': 2, 'timestep': t + 0.5})
                        #         else:
                        #             if ac1.id < ac2.id:  # largest id gives way
                        #                 constraints.append({'agent': ac2.id, 'loc': 1, 'timestep': t + 0.5})
                        #                 constraints.append({'agent': ac2.id, 'loc': 2, 'timestep': t + 0.5})
                        #             else:
                        #                 constraints.append({'agent': ac1.id, 'loc': 1, 'timestep': t + 0.5})
                        #                 constraints.append({'agent': ac1.id, 'loc': 2, 'timestep': t + 0.5})
                        #         # print("constr", constraints)
                        #         run_astar(ac1, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, constraints,
                        #                   ac1.lastdifferentnode)
                        #         run_astar(ac2, nodes_dict, ac2.from_to[0], ac2.goal, heuristics, t, constraints,
                        #                   ac2.lastdifferentnode)

                        # if ac1.position == (1.5, 4): # small ac waits 0.5 sec for vertex turbulence to disappear caused by heavy ac
                        #     if ac2.position == (2, 5) and ac2.path_to_goal[0][0] == 95:
                        #         constraints.append({'agent': ac2.id, 'loc': 1, 'timestep': t + 1})
                        #         run_astar(ac2, nodes_dict, ac2.from_to[0], ac2.goal, heuristics, t, constraints,
                        #                   ac2.lastdifferentnode)
                        #         #print("Aircraft", ac2.id, "waits due to heavy vortex from aircraft", ac1.id)


                nextintersection = find_next_intersection(ac1)
                nextlinkage = find_next_linkage(ac1)

                for observation in ac1.observations:
                    if 44.0 in observation.from_to:

                        if observation.heading == 0:
                            constraints.append({'agent': ac1.id, 'loc': 4, 'timestep': t + 0.5})
                        if observation.heading == 180:
                            constraints.append({'agent': ac1.id, 'loc': 5, 'timestep': t + 0.5})

                        run_astar(ac1, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, constraints,
                                  ac1.lastdifferentnode)

                        nextintersection = find_next_intersection(ac1)
                        nextlinkage = find_next_linkage(ac1)


                    if observation.from_to[0] == ac1.goal:
                        for neighbor in ac1.nodes_dict[ac1.goal]["neighbors"]:
                            for nextneighbour in ac1.nodes_dict[neighbor]["neighbors"]:
                                constraints.append({'agent': ac1.id, 'loc': nextneighbour, 'timestep': t + 0.5})
                            run_astar(ac1, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, constraints,
                                          ac1.lastdifferentnode)

                            nextintersection = find_next_intersection(ac1)
                            nextlinkage = find_next_linkage(ac1)


                    if ac1.heading == observation.heading:
                        if ac1.from_to[1] == observation.from_to[0]:
                            constraints.append({'agent': ac1.id, 'loc': ac1.from_to[1], 'timestep': t + 0.5})
                            run_astar(ac1, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, constraints,
                                      ac1.lastdifferentnode)

                            nextintersection = find_next_intersection(ac1)
                            nextlinkage = find_next_linkage(ac1)

                    if nextintersection: # if ac1 has a next intersection
                        if observation.from_to[0] in {97, 34, 35, 36, 98} and ac1.goal == observation.from_to[1]:
                            for neighbor in ac1.nodes_dict[ac1.goal]['neighbors']:
                                constraints.append({'agent': ac1.id, 'loc': neighbor, 'timestep': t + 0.5})
                            run_astar(ac1, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, constraints,
                                      ac1.lastdifferentnode)

                            nextintersection = find_next_intersection(ac1)
                            nextlinkage = find_next_linkage(ac1)

                        elif nextintersection == find_next_intersection(observation):
                            # ac1_can_detour = run_astar(ac1, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, # discuss
                            #            [{'agent': ac1.id, 'loc': nextintersection['id'], 'timestep': t + 0.5},
                            #             {'agent': ac1.id, 'loc': nextintersection['id'], 'timestep': t + 1}],
                            #           ac1.lastdifferentnode, False)
                            #
                            # ac2_can_detour = run_astar(ac2, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t,
                            #                            [{'agent': ac2.id, 'loc': nextintersection['id'], 'timestep': t + 0.5},
                            #                             {'agent': ac2.id, 'loc': nextintersection['id'], 'timestep': t + 1}],
                            #                    ac1.lastdifferentnode, False)
                            #
                            # # check if ac2 can detour (for situations near 104/107)
                            # if ac1_can_detour == False or ac2_can_detour == False:
                            #     print("-------------------------------------------------------------------------------")
                            #     if ac1_can_detour:
                            #         ac1 = loser
                            #         ac2 = winner
                            #     elif ac2_can_detour:
                            #         ac2 = loser
                            #         ac1 = winner
                            #     else:
                            #         print("Neither", ac1.id, ac1.position, "nor", ac2.id, ac2.position, "can detour for conflict at", nextintersection)
                            #
                            # else:
                            #
                            winner, loser = determine_right_of_way(ac1, observation, nextintersection)

                            # review ###########
                            constraints.append({'agent': loser.id, 'loc': nextintersection['id'], 'timestep': t + 0.5})
                            constraints.append({'agent': loser.id, 'loc': nextintersection['id'], 'timestep': t + 1})

                            if ac1.id == 8:
                                if observation.id == 1:
                                    print(constraints)
                                    print(loser.from_to)

                            run_astar(loser, nodes_dict, loser.from_to[0], loser.goal, heuristics, t, constraints, loser.lastdifferentnode)

                            nextintersection = find_next_intersection(ac1)
                            nextlinkage = find_next_linkage(ac1)

                    if nextlinkage: # if ac1 has a next intersection
                        if nextlinkage == find_next_linkage(observation):

                            # ac1_can_detour = run_astar(ac1, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, constraints,
                            #           ac1.lastdifferentnode, False)
                            #
                            # ac2_can_detour = run_astar(ac2, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, constraints,
                            #                    ac1.lastdifferentnode, False)
                            #
                            # if ac1_can_detour == False or ac2_can_detour == False:
                            #     if ac1_can_detour:
                            #         ac1 = loser
                            #         ac2 = winner
                            #     elif ac2_can_detour:
                            #         ac2 = loser
                            #         ac1 = winner
                            #     else:
                            #         print("Neither", ac1.id, "nor", ac2.id,  "can detour for conflict at", nextlinkage)
                            # else:
                            winner, loser = determine_right_of_way(ac1, observation, nextlinkage)
                            if ac1.id == 8:
                                if observation.id == 1:
                                    print(t)
                                    print(nextlinkage)
                                    print(find_next_linkage(observation))
                                    print(loser.id, 'giving way')
                            constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t})
                            constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 0.5})
                            constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 1})
                            constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 1.5})
                            constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 2})
                            constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 2.5})
                            constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 3}) # discuss
                            if ac1.id == 8:
                                if observation.id == 1:
                                    print(constraints)
                                    print(loser.from_to)
                            # print(constraints)

                            run_astar(loser, nodes_dict, loser.from_to[0], loser.goal, heuristics, t, constraints, loser.lastdifferentnode)

                            nextintersection = find_next_intersection(ac1)
                            nextlinkage = find_next_linkage(ac1)



def determine_right_of_way(ac1, ac2, conflictnode):
    #print("Determining right of way between", ac1.id, "and", ac2.id)
    # print(ac1.path_to_goal)
    # print(conflictnode)
    # print([node for node in ac1.path_to_goal if node[0] == conflictnode['id']])

    node_ac1 = [node for node in ac1.total_path if node[0] == conflictnode['id']][0]
    node_ac2 = [node for node in ac2.total_path if node[0] == conflictnode['id']][0]
    index1 = ac1.total_path.index(node_ac1)
    index2 = ac2.total_path.index(node_ac2)

    if node_ac1[0] == 11 or node_ac1[0] == 12:  # give priority to aircraft leaving runways
        if ac1.id < ac2.id:
            return ac2, ac1 # winner, loser
        else:
            return ac1, ac2

    if index1 == index2:
        if ac1.weight_class == "Heavy" and ac2.weight_class == "Heavy": # smallest aircraft gives way
            return ac1, ac2 # winner, loser
        elif ac1.weight_class == "Small" and ac2.weight_class == "Small":
            return ac2, ac1
        else:
            if ac1.id < ac2.id: # largest id gives way
                return ac1, ac2  # winner, loser
            else:
                return ac2, ac1
    else:
        if index1 < index2: # largest index gives way (ac that has been waiting the least amount of time)
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
        #print("New path path for AC", ac.id, ":", path)

    return success

def find_next_intersection(ac):
    for node in ac.total_path:
        if ac.nodes_dict[node[0]]['type'] == "intersection":
            if ac.nodes_dict[node[0]]['id'] == node[0]:
                return ac.nodes_dict[node[0]]

def find_next_linkage(ac):
    for node in ac.total_path:
        if ac.nodes_dict[node[0]]['type'] == "between":
            if ac.nodes_dict[node[0]]['id'] == node[0]:
                return ac.nodes_dict[node[0]]


# A function to check whether point P(x, y)
# lies inside the triangle formed by
# A(x1, y1), B(x2, y2) and C(x3, y3)
# https://www.geeksforgeeks.org/check-whether-a-given-point-lies-inside-a-triangle-or-not/
def in_view(c1, c2, c3, point):
    x1, y1 = c1[0], c1[1]
    x2, y2 = c2[0], c2[1]
    x3, y3 = c3[0], c3[1]
    x, y = point[0], point[1]

    # A utility function to calculate area
    # of triangle formed by (x1, y1),
    # (x2, y2) and (x3, y3)
    def area(x1, y1, x2, y2, x3, y3):

        return abs((x1 * (y2 - y3) + x2 * (y3 - y1)
                    + x3 * (y1 - y2)) / 2.0)

    # Calculate area of triangle ABC
    A = area(x1, y1, x2, y2, x3, y3)

    # Calculate area of triangle PBC
    A1 = area(x, y, x2, y2, x3, y3)

    # Calculate area of triangle PAC
    A2 = area(x1, y1, x, y, x3, y3)

    # Calculate area of triangle PAB
    A3 = area(x1, y1, x2, y2, x, y)

    # Check if sum of A1, A2 and A3
    # is same as A
    if (A == A1 + A2 + A3):
        return True
    else:
        return False
