import time as timer
import heapq
import random
import copy
from single_agent_planner import simple_single_agent_astar
# # from run_me import node_to_xy
# ######################################################################################################################################
#                                                         # Distributed Runner
# ######################################################################################################################################
# constraints = []
# def run_distributed_planner(aircraft_lst, nodes_dict, heuristics, t, traffic_agents):
#     radar_current_states = []
#     for ac in aircraft_lst:
#
#         if ac.spawntime == t:  # maybe not necessary
#             start_node = ac.start  # node from which planning should be done
#             # print('start_node agent', ID, start_node, 'time', t)
#             ac.status = "taxiing"
#             ac.position = nodes_dict[ac.start]["xy_pos"]
#             # print('root constraints', root['constraints'])
#             success, path = simple_single_agent_astar(ac.id, nodes_dict,  ac.start, ac.goal, heuristics, t, constraints)
#
#             if path is None:
#                 raise BaseException('No solutions')
#             # print('path', root['paths'])
#
#             if success:
#                 ac.path_to_goal = path[1:]
#                 # print("ac_path", ac.path_to_goal, 'time', self.t)
#                 # self.push_locations_to_radar(get_location(ac.path_to_goal, self.t), ID)
#                 next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
#                 ac.from_to = [path[0][0], next_node_id]
#                 print("Path AC", ac.id, ":", path)
#             else:
#                 raise Exception("No solution found for", ac.id)
#             # Check the path
#             if path[0][1] != t:
#                 raise Exception("Something is wrong with the timing of the path planning")
#
#         if ac.status == "taxiing" or ac.status == "at_gate":
#             radar_current_state = {"aircraft": ac.id,
#                                    "xyposition": ac.position,
#                                    "loc": ac.from_to[0],
#                                    "hdg": ac.heading,
#                                    "time": t}
#             radar_current_states.append(radar_current_state)
#
#         if traffic_identification_intersection_nodes(traffic_agents, radar_current_states):
#             for x in traffic_identification_intersection_nodes(traffic_agents, radar_current_states):
#                 for constraint in constraints:
#                     constraints.append(x)
#             print(constraints)
#             success, path = simple_single_agent_astar(x['agent'], nodes_dict,  ac.from_to[0], ac.goal, heuristics, t, constraints)
#
#             if path is None:
#                 raise BaseException('No solutions')
#             # print('path', root['paths'])
#
#             if success:
#                 ac.path_to_goal = path[1:]
#                 # print("ac_path", ac.path_to_goal, 'time', self.t)
#                 # self.push_locations_to_radar(get_location(ac.path_to_goal, self.t), ID)
#                 next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
#                 ac.from_to = [path[0][0], next_node_id]
#                 print("Path AC", ac.id, ":", path)
#             else:
#                 raise Exception("No solution found for", ac.id)
#             # Check the path
#             if path[0][1] != t:
#                 raise Exception("Something is wrong with the timing of the path planning")
#
#         if traffic_identification_between_nodes(traffic_agents, radar_current_states):
#             for y in traffic_identification_between_nodes(traffic_agents, radar_current_states):
#                 constraints.append(y)
#             print(constraints)
#             success, path = simple_single_agent_astar(y['agent'], nodes_dict, ac.from_to[0], ac.goal, heuristics, t, constraints)
#
#             if path is None:
#                 raise BaseException('No solutions')
#             # print('path', root['paths'])
#
#             if success:
#                 ac.path_to_goal = path[1:]
#                 # print("ac_path", ac.path_to_goal, 'time', self.t)
#                 # self.push_locations_to_radar(get_location(ac.path_to_goal, self.t), ID)
#                 next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
#                 ac.from_to = [path[0][0], next_node_id]
#                 print("Path AC", ac.id, ":", path)
#             else:
#                 raise Exception("No solution found for", ac.id)
#             # Check the path
#             if path[0][1] != t:
#                 raise Exception("Something is wrong with the timing of the path planning")
#
#         if constraints:
#             print('constraints',constraints)
#
#
#
#
#
#
# ######################################################################################################################################
#                                                         # Intersection Nodes
# ######################################################################################################################################
# # Determine if an aircraft is in the area of a traffic agent and also is heading towards it
# # Identify if multiple ac's are heading to 1 intersection
#
# ###########################################################################################################
# # Ik zou  blijven werken met aircraft objects. Maak hiervan een dict met het aircraft object en het traffic agent object.
# #
# # Voorbeeld:
# #
# def xtraffic_identification(traffic_agents, aircraft_lst, t):
#     approach_agent_list = []
#     approach_between_node_list = []
#     for ac in aircraft_lst:  # loop through ac's in the global radar
#         ac_loc = ac.from_to[0]
#         for traffic_agent in traffic_agents:  # loop through existing traffic agents at intersection points
#             for neighbor in traffic_agent.heading:  # loop through neighbors of traffic agents
#                 neighbor_node = neighbor[0]
#                 heading_neighbor = neighbor[1]
#                 if ac_loc == neighbor_node:  # if an ac in the neighborhood of a traffic agent
#                     if heading_neighbor == 0:  # flip heading with 180 degrees to achieve "to" instead of "from" a node
#                         heading_neighbor = 180
#                     elif heading_neighbor == 180:
#                         heading_neighbor = 0
#                     elif heading_neighbor == 270:
#                         heading_neighbor = 90
#                     else:
#                         heading_neighbor = 270
#                     if ac.heading == heading_neighbor:  # if an ac is going towards an intersection point
#
#                         approach_agent_list.append({'ac': ac, 'traffic_agent': traffic_agent}) #<----
#
#
#                 if ac.position == traffic_agent.xy_pos:  # if an ac is at the position of a traffic agent
#                     approach_agent_list.append({'ac': ac, 'traffic_agent': traffic_agent})
#
# ######################################################################################################################################
#
# def traffic_identification_intersection_nodes(traffic_agents, radar_current_states):
#     approach_agent_list = []
#     constraints_intersections = []
#     for radar_object in radar_current_states:  # loop through ac's in the global radar
#         for traffic_agent in traffic_agents:  # loop through existing traffic agents at intersection points
#             for neighbor in traffic_agent.heading:  # loop through neighbors of traffic agents
#                 neighbor_node = neighbor[0]
#                 heading_neighbor = neighbor[1]
#                 if radar_object['loc'] == neighbor_node:  # if an ac in the neighborhood of a traffic agent
#                     if heading_neighbor == 0:  # flip heading with 180 degrees to achieve "to" instead of "from" a node
#                         heading_neighbor = 180
#                     elif heading_neighbor == 180:
#                         heading_neighbor = 0
#                     elif heading_neighbor == 270:
#                         heading_neighbor = 90
#                     else:
#                         heading_neighbor = 270
#                     if radar_object['hdg'] == heading_neighbor:  # if an ac is going towards an intersection point
#                         # print(neighbor, traffic_agent['id'])
#                         approach_agent_list.append({'time': radar_object['time'], 'node': radar_object['loc'],
#                                                     'xyposition': radar_object['xyposition'],
#                                                     'heading': radar_object['hdg'],
#                                                     'aircraft': radar_object['aircraft'],
#                                                     'traffic_agent_id': traffic_agent.id,
#                                                     'traffic_agent_xypos': traffic_agent.xy_pos})
#
#                         # print('hdg', heading_neighbor ,{'time': radar_object['time'], 'node': radar_object['loc'], 'xyposition': radar_object['xyposition'],
#                         #                         'aircraft': radar_object['aircraft'], 'traffic_agent': traffic_agent['id']})
#
#
#     for first_ac in approach_agent_list:  # identify ac's heading towards the same traffic agent
#         for sec_ac in approach_agent_list:
#             if first_ac != sec_ac:
#                 if first_ac['traffic_agent_id'] == sec_ac['traffic_agent_id']:
#                     # print('t-1 to collision', first_ac, sec_ac)
#                     x = grant_priority_intersections(first_ac, sec_ac)
#                     if x:
#                         constraints_intersections.append(x)  # if two acs are heading towards the same node
#
#     if constraints_intersections:
#         # print('constraints', constraints_intersections)
#         return constraints_intersections
#
# # Define priority
# # Add a constraint to the submissive ac
# def grant_priority_intersections(first_ac, sec_ac):
#     pos_first_ac = first_ac['xyposition']
#     pos_sec_ac = sec_ac['xyposition']
#     delta_x = pos_first_ac[0] - pos_sec_ac[0]
#     delta_y = pos_first_ac[1] - pos_sec_ac[1]
#     Relative_incoming_ac_comes_from = None
#     constraint = []
#
#     if first_ac['heading'] == 180:  # identify if sec_ac is left, right, front of first_ac
#         if delta_x == -0.5 and delta_y == 0.5:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "L")
#             Relative_incoming_ac_comes_from = "L"
#         if delta_x == 0 and delta_y == 1:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "F")
#             Relative_incoming_ac_comes_from = "F"
#         if delta_x == 0.5 and delta_y == 0.5:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "R")
#             Relative_incoming_ac_comes_from = "R"
#
#     elif first_ac['heading'] == 90:
#         if delta_x == 0.5 and delta_y == -0.5:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "R")
#             Relative_incoming_ac_comes_from = "R"
#         if delta_x == 0.5 and delta_y == 0.5:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "L")
#             Relative_incoming_ac_comes_from = "L"
#         if delta_x == 1 and delta_y == 0:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "F")
#             Relative_incoming_ac_comes_from = "F"
#
#     elif first_ac['heading'] == 0:
#         if delta_x == 0 and delta_y == -1:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "F")
#             Relative_incoming_ac_comes_from = "F"
#         if delta_x == -0.5 and delta_y == -0.5:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "R")
#             Relative_incoming_ac_comes_from = "R"
#         if delta_x == 0.5 and delta_y == -0.5:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "L")
#             Relative_incoming_ac_comes_from = "L"
#
#     else:
#         if delta_x == -0.5 and delta_y == -0.5:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "L")
#             Relative_incoming_ac_comes_from = "L"
#         if delta_x == -1 and delta_y == 0:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "F")
#             Relative_incoming_ac_comes_from = "F"
#         if delta_x == -0.5 and delta_y == 0.5:
#             print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "R")
#             Relative_incoming_ac_comes_from = "R"
#
#     if Relative_incoming_ac_comes_from == "R":
#         print('aircraft', first_ac['aircraft'], 'should stop')
#         constraints.append({'agent': sec_ac['aircraft'], 'loc': sec_ac['traffic_agent_id'], 'timestep': first_ac['time'] + 0.5})
#
#         constraints.append({'agent': first_ac['aircraft'], 'loc': first_ac['node'], 'timestep': first_ac['time'] + 1})
#
#     if Relative_incoming_ac_comes_from == "F":
#         if first_ac['heading'] == 90 or first_ac['heading'] == 180:
#             print('aircraft', sec_ac['aircraft'], 'should stop')
#             constraints.append({'agent': first_ac['aircraft'], 'loc': first_ac['traffic_agent_id'],
#                           'timestep': sec_ac['time'] + 0.5})
#             constraints.append({'agent': sec_ac['aircraft'], 'loc': first_ac['node'],
#                           'timestep': sec_ac['time'] + 1})
#         if sec_ac['heading'] == 90 or sec_ac['heading'] == 180:
#             print('aircraft', first_ac['aircraft'], 'should stop')
#             constraints.append({'agent': sec_ac['aircraft'], 'loc': sec_ac['traffic_agent_id'],
#                           'timestep': sec_ac['time'] + 0.5})
#             constraints.append({'agent': first_ac['aircraft'], 'loc': sec_ac['node'],
#                           'timestep': first_ac['time'] + 1})
#     return constraints
#
# ######################################################################################################################################
#                                                         # Between Nodes
# ######################################################################################################################################
#
# def traffic_identification_between_nodes(traffic_agents, radar_current_states):
#     approach_between_node_list = []
#     constraints_betweens = []
#
#     for radar_object in radar_current_states:  # loop through ac's in the global radar
#         for traffic_agent in traffic_agents:  # loop through existing traffic agents at intersection points
#             if radar_object[
#                 'xyposition'] == traffic_agent.xy_pos:  # if an ac is at the position of a traffic agent
#                 approach_between_node_list.append({'time': radar_object['time'], 'node': radar_object['loc'],
#                                                    'xyposition': radar_object['xyposition'],
#                                                    'heading': radar_object['hdg'],
#                                                    'aircraft': radar_object['aircraft'],
#                                                    'traffic_agent_id': traffic_agent.id,
#                                                    'traffic_agent_xypos': traffic_agent.xy_pos})
#
#     approach_between_node_list_clean = []  # clean list from duplicates
#     for x in approach_between_node_list:
#         if x not in approach_between_node_list_clean:
#             approach_between_node_list_clean.append(x)
#
#     for first_ac in approach_between_node_list_clean:  # identify ac's heading towards the same traffic agent
#         for sec_ac in approach_between_node_list_clean:
#             if first_ac != sec_ac:
#                 delta_x = first_ac['xyposition'][0] - sec_ac['xyposition'][0]
#                 delta_y = first_ac['xyposition'][1] - sec_ac['xyposition'][1]
#                 if delta_x == 0 and delta_y == 1:
#                     if first_ac['heading'] == 0 and sec_ac['heading'] == 180:
#                         # print('t-1 to collision', first_ac, sec_ac)
#                         x = grant_priority_between_nodes(first_ac, sec_ac)
#                         if x:
#                             for i in x:
#                                 constraints_betweens.append(i)
#                 if delta_x == 0 and delta_y == -1:
#                     if first_ac['heading'] == 180 and sec_ac['heading'] == 0:
#                         # print('t-1 to collision', first_ac, sec_ac)
#                         x = grant_priority_between_nodes(first_ac, sec_ac)
#                         if x:
#                             for i in x:
#                                 constraints_betweens.append(i)
#                 if delta_y == 0 and delta_x == 1:
#                     if first_ac['heading'] == 90 and sec_ac['heading'] == 270:
#                         # print('t-1 to collision', first_ac, sec_ac)
#                         x = grant_priority_between_nodes(first_ac, sec_ac)
#                         if x:
#                             for i in x:
#                                 constraints_betweens.append(i)
#                 if delta_y == 0 and delta_x == -1:
#                     if first_ac['heading'] == 270 and sec_ac['heading'] == 90:
#                         # print('t-1 to collision', first_ac, sec_ac)
#                         x = grant_priority_between_nodes(first_ac, sec_ac)
#                         if x:
#                             for i in x:
#                                 constraints_betweens.append(i)
#
#     if constraints_betweens:
#         return constraints_betweens
#
#
#
#
# def grant_priority_between_nodes(first_ac, sec_ac):
#     constraint = []
#     if first_ac['heading'] == 90 or first_ac['heading'] == 180:
#         print('aircraft', sec_ac['aircraft'], 'should stop')
#     constraint.append({'agent': sec_ac['aircraft'], 'loc': sec_ac['traffic_agent_id'], #node must be between node
#                        'timestep': sec_ac['time'] + 0.5})
#     constraint.append({'agent': sec_ac['aircraft'], 'loc': sec_ac['node'],
#                        'timestep': sec_ac['time'] + 0.5})
#
#     if sec_ac['heading'] == 90 or sec_ac['heading'] == 180:
#         print('aircraft', first_ac['aircraft'], 'should stop')
#         constraint.append({'agent': first_ac['aircraft'], 'loc': first_ac['traffic_agent_id'], #node must be between node
#                            'timestep': first_ac['time'] + 0.5})
#         constraint.append({'agent': first_ac['aircraft'], 'loc': first_ac['node'],
#                            'timestep': first_ac['time'] + 0.5})
#
#     return constraint
#
#
#
#
#
#
# ######################################################################################################################################
# # if first_ac
# # self.constraints.append({'aircraft': y, 'node': [path[x][0]], 'timestep': path[x][1]})
# # pass
#
# # def run_Individual(aircraft_list, t):
#
# # traffic_identification()
#
# # for ac in aircraft_lst:
# #     ID = ac.id
# #
# #     if ac.spawntime == t:  # maybe not necessary
# #         start_node = ac.start  # node from which planning should be done
# #         # print('start_node agent', ID, start_node, 'time', t)
# #         goal_node = ac.goal
# #         ac.status = "taxiing"
# #         ac.position = self.nodes_dict[ac.start]["xy_pos"]
# #         # print('root constraints', root['constraints'])
# #         success, path = astar_CBS(self.nodes_dict, start_node, goal_node, self.heuristics, self.t, ID, self.constraints)
# #
# #         if path is None:
# #             raise BaseException('No solutions')
# #         # print('path', root['paths'])
# #
# #
# #         if success:
# #             ac.path_to_goal = path[1:]
# #             # print("ac_path", ac.path_to_goal, 'time', self.t)
# #             # self.push_locations_to_radar(get_location(ac.path_to_goal, self.t), ID)
# #             next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
# #             ac.from_to = [path[0][0], next_node_id]
# #             print("Path AC", ac.id, ":", path)
# #         else:
# #             raise Exception("No solution found for", ac.id)
# #         # Check the path
# #         if path[0][1] != self.t:
# #             raise Exception("Something is wrong with the timing of the path planning")
#
# # start_time = timer.time()
# # ac.CPU_time = timer.time() - start_time
# # print("CPU time (s):    {:.2f}".format(ac.CPU_time))
#
# # """
# # Implement prioritized planner here
# # """
# #
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
                    # print(ac1.path_to_goal)
                    # print(observation.path_to_goal)
                    # print(nextintersection)
                    # print(find_next_intersection(observation))
                    # print("found intersection conflict between ac", ac1.id, "and", observation.id, 'at', nextintersection['id'])

                    winner, loser = determine_right_of_way(ac1, observation, nextintersection)
                    # print(loser.id, 'giving way')
                    constraints.append({'agent': loser.id, 'loc': nextintersection['id'], 'timestep': t + 0.5})
                    constraints.append({'agent': loser.id, 'loc': nextintersection['id'], 'timestep': t + 1})

                    # print("Added constraint:", {'agent': loser.id, 'loc': nextintersection['id'], 'timestep': t + 1})
                    # print(constraints)
                    run_astar(loser, nodes_dict, loser.from_to[0], loser.goal, heuristics, t, constraints)

                    nextintersection = find_next_intersection(ac1)
                    nextlinkage = find_next_linkage(ac1)

            if nextlinkage: # if ac1 has a next intersection
                if nextlinkage == find_next_linkage(observation):
                    # print(ac1.path_to_goal)
                    # print(observation.path_to_goal)
                    # print(nextlinkage)
                    # print(find_next_linkage(observation))
                    # print("found linkage conflict between ac", ac1.id, "and", observation.id, 'at', nextlinkage['id'])

                    winner, loser = determine_right_of_way(ac1, observation, nextlinkage)
                    # print(loser.id, 'giving way')
                    constraints.append({'agent': winner.id, 'loc': nextlinkage['id'], 'timestep': t + 0.5})
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 0.5})
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 1})
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 1.5})
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 2})
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 2.5})
                    constraints.append({'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 3})

                    # print("added constraint:", {'agent': winner.id, 'loc': nextlinkage['id'], 'timestep': t + 0.5})
                    # print("added constraint:", {'agent': loser.id, 'loc': nextlinkage['id'], 'timestep': t + 0.5})
                    # print(constraints)

                    run_astar(winner, nodes_dict, winner.from_to[0], winner.goal, heuristics, t, constraints)

                    run_astar(loser, nodes_dict, loser.from_to[0], loser.goal, heuristics, t, constraints)

                    nextintersection = find_next_intersection(ac1)
                    nextlinkage = find_next_linkage(ac1)


        # append to memory. Add weights to expected paths of remembered aircraft. Resolve conflicts that may occur

        # for otheraircraft in aircraft_lst:
        #     if ac != otheraircraft:
        #         if ac.heading == 0:
        #             if otheraircraft.position[1] > ac.position[1]:
        #                 if otheraircraft.position[1] - ac.position[1]

def determine_right_of_way(ac1, ac2, conflictnode):
    print("Determining right of way between", ac1.id, "and", ac2.id)
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

def run_astar(ac, nodes_dict, from_to, goal, heuristics, t, constraints):
    success, path = simple_single_agent_astar(ac.id, nodes_dict, from_to, goal, heuristics, t,
                                              constraints)
    if success:
        ac.total_path = path
        ac.path_to_goal = path[1:]
        next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
        ac.from_to = [path[0][0], next_node_id]

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