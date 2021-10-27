import time as timer
import heapq
import random
import copy
from single_agent_planner import simple_single_agent_astar


# from run_me import node_to_xy

def run_distributed_planner(aircraft_lst, nodes_dict, heuristics, t, traffic_agents):
    radar_current_states = []
    for ac in aircraft_lst:
        constraints = []
        if ac.status == "taxiing" or ac.status == "at_gate":
            radar_current_state = {"aircraft": ac.id,
                                   "xyposition": ac.position,
                                   "loc": ac.from_to[0],
                                   "hdg": ac.heading,
                                   "time": t}
            radar_current_states.append(radar_current_state)

        traffic_identification(traffic_agents, radar_current_states)

        if ac.spawntime == t:  # maybe not necessary
            start_node = ac.start  # node from which planning should be done
            # print('start_node agent', ID, start_node, 'time', t)
            goal_node = ac.goal
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            # print('root constraints', root['constraints'])
            success, path = simple_single_agent_astar(ac.id, nodes_dict, start_node, goal_node, heuristics, t, constraints)

            if path is None:
                raise BaseException('No solutions')
            # print('path', root['paths'])

            if success:
                ac.path_to_goal = path[1:]
                # print("ac_path", ac.path_to_goal, 'time', self.t)
                # self.push_locations_to_radar(get_location(ac.path_to_goal, self.t), ID)
                next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                ac.from_to = [path[0][0], next_node_id]
                print("Path AC", ac.id, ":", path)
            else:
                raise Exception("No solution found for", ac.id)
            # Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")


# Determine if an aircraft is in the area of a traffic agent and also is heading towards it
# Identify if multiple ac's are heading to 1 intersection

###########################################################################################################
# Ik zou  blijven werken met aircraft objects. Maak hiervan een dict met het aircraft object en het traffic agent object.
#
# Voorbeeld:
#
# def traffic_identification(traffic_agents, aircraft_lst, t):
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

#                         approach_agent_list.append({'ac': ac, 'traffic_agent': traffic_agent}) <------------------

#
#                 if ac.position == traffic_agent.xy_pos:  # if an ac is at the position of a traffic agent
#                     approach_agent_list.append({'ac': ac, 'traffic_agent': traffic_agent})
######################################################################################################################################
def traffic_identification(traffic_agents, radar_current_states):
    approach_agent_list = []
    approach_between_node_list = []
    for radar_object in radar_current_states:  # loop through ac's in the global radar
        for traffic_agent in traffic_agents:  # loop through existing traffic agents at intersection points
            for neighbor in traffic_agent.heading:  # loop through neighbors of traffic agents
                neighbor_node = neighbor[0]
                heading_neighbor = neighbor[1]
                if radar_object['loc'] == neighbor_node:  # if an ac in the neighborhood of a traffic agent
                    if heading_neighbor == 0:  # flip heading with 180 degrees to achieve "to" instead of "from" a node
                        heading_neighbor = 180
                    elif heading_neighbor == 180:
                        heading_neighbor = 0
                    elif heading_neighbor == 270:
                        heading_neighbor = 90
                    else:
                        heading_neighbor = 270
                    if radar_object['hdg'] == heading_neighbor:  # if an ac is going towards an intersection point
                        # print(neighbor, traffic_agent['id'])
                        approach_agent_list.append({'time': radar_object['time'], 'node': radar_object['loc'],
                                                    'xyposition': radar_object['xyposition'],
                                                    'heading': radar_object['hdg'],
                                                    'aircraft': radar_object['aircraft'],
                                                    'traffic_agent_id': traffic_agent.id,
                                                    'traffic_agent_xypos': traffic_agent.xy_pos})

                        # print('hdg', heading_neighbor ,{'time': radar_object['time'], 'node': radar_object['loc'], 'xyposition': radar_object['xyposition'],
                        #                         'aircraft': radar_object['aircraft'], 'traffic_agent': traffic_agent['id']})
                if radar_object['xyposition'] == traffic_agent.xy_pos:  # if an ac is at the position of a traffic agent
                    approach_between_node_list.append({'time': radar_object['time'], 'node': radar_object['loc'],
                                                       'xyposition': radar_object['xyposition'],
                                                       'heading': radar_object['hdg'],
                                                       'aircraft': radar_object['aircraft'],
                                                       'traffic_agent_id': traffic_agent.id,
                                                       'traffic_agent_xypos': traffic_agent.xy_pos})

    # print('between', approach_between_node_list)
    for first_ac in approach_agent_list:  # identify ac's heading towards the same traffic agent
        for sec_ac in approach_agent_list:
            if first_ac != sec_ac:
                if first_ac['traffic_agent_id'] == sec_ac['traffic_agent_id']:
                    # print('t-1 to collision', first_ac, sec_ac)
                    grant_priority_intersections(first_ac, sec_ac)  # if two acs are heading towards the same node

    approach_between_node_list_clean = []  # clean list from duplicates
    for x in approach_between_node_list:
        if x not in approach_between_node_list_clean:
            approach_between_node_list_clean.append(x)

    for first_ac in approach_between_node_list_clean:  # identify ac's heading towards the same traffic agent
        for sec_ac in approach_between_node_list_clean:
            if first_ac != sec_ac:
                delta_x = first_ac['xyposition'][0] - sec_ac['xyposition'][0]
                delta_y = first_ac['xyposition'][1] - sec_ac['xyposition'][1]
                if delta_x == 0 and delta_y == 1:
                    if first_ac['heading'] == 0 and sec_ac['heading'] == 180:
                        # print('t-1 to collision', first_ac, sec_ac)
                        grant_priority_between_nodes(first_ac, sec_ac)
                if delta_x == 0 and delta_y == -1:
                    if first_ac['heading'] == 180 and sec_ac['heading'] == 0:
                        # print('t-1 to collision', first_ac, sec_ac)
                        grant_priority_between_nodes(first_ac, sec_ac)
                if delta_y == 0 and delta_x == 1:
                    if first_ac['heading'] == 90 and sec_ac['heading'] == 270:
                        # print('t-1 to collision', first_ac, sec_ac)
                        grant_priority_between_nodes(first_ac, sec_ac)
                if delta_y == 0 and delta_x == -1:
                    if first_ac['heading'] == 270 and sec_ac['heading'] == 90:
                        # print('t-1 to collision', first_ac, sec_ac)
                        grant_priority_between_nodes(first_ac, sec_ac)

                        # print('t-1 to collision', first_ac, sec_ac)
                        # self.grant_priority_between_nodes(first_ac, sec_ac)


def grant_priority_between_nodes(first_ac, sec_ac):
    # print(first_ac['aircraft'], first_ac['heading'], 'sec', sec_ac['aircraft'], sec_ac['heading'] )
    if first_ac['heading'] == 90 or first_ac['heading'] == 180:
        print('aircraft', sec_ac['aircraft'], 'should stop')
    if sec_ac['heading'] == 90 or sec_ac['heading'] == 180:
        print('aircraft', first_ac['aircraft'], 'should stop')


# Define priority
# Add a constraint to the submissive ac
def grant_priority_intersections(first_ac, sec_ac):
    traffic_agent = first_ac['traffic_agent_id']  # or sec_ac['traffic_agent']
    pos_agent = first_ac['traffic_agent_xypos']  # or sec_ac['traffic_agent_xypos']
    pos_first_ac = first_ac['xyposition']
    pos_sec_ac = sec_ac['xyposition']
    delta_x = pos_first_ac[0] - pos_sec_ac[0]
    delta_y = pos_first_ac[1] - pos_sec_ac[1]
    # print(first_ac['aircraft'], first_ac['heading'], 'first', pos_first_ac, 'sec', pos_sec_ac, 'deltax', delta_x, 'deltay', delta_y)
    Relative_incoming_ac_comes_from = None

    if first_ac['heading'] == 180:  # identify if sec_ac is left, right, front of first_ac
        if delta_x == -0.5 and delta_y == 0.5:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "L")
            Relative_incoming_ac_comes_from = "L"
        if delta_x == 0 and delta_y == 1:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "F")
            Relative_incoming_ac_comes_from = "F"
        if delta_x == 0.5 and delta_y == 0.5:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "R")
            Relative_incoming_ac_comes_from = "R"

    elif first_ac['heading'] == 90:
        if delta_x == 0.5 and delta_y == -0.5:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "R")
            Relative_incoming_ac_comes_from = "R"
        if delta_x == 0.5 and delta_y == 0.5:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "L")
            Relative_incoming_ac_comes_from = "L"
        if delta_x == 1 and delta_y == 0:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "F")
            Relative_incoming_ac_comes_from = "F"

    elif first_ac['heading'] == 0:
        if delta_x == 0 and delta_y == -1:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "F")
            Relative_incoming_ac_comes_from = "F"
        if delta_x == -0.5 and delta_y == -0.5:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "R")
            Relative_incoming_ac_comes_from = "R"
        if delta_x == 0.5 and delta_y == -0.5:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "L")
            Relative_incoming_ac_comes_from = "L"

    else:
        if delta_x == -0.5 and delta_y == -0.5:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "L")
            Relative_incoming_ac_comes_from = "L"
        if delta_x == -1 and delta_y == 0:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "F")
            Relative_incoming_ac_comes_from = "F"
        if delta_x == -0.5 and delta_y == 0.5:
            print("aircraft", first_ac['aircraft'], "has", "aircraft", sec_ac['aircraft'], "on the",  "R")
            Relative_incoming_ac_comes_from = "R"

    if Relative_incoming_ac_comes_from == "R":
        print('aircraft', first_ac['aircraft'], 'should stop')
    if Relative_incoming_ac_comes_from == "F":
        if first_ac['heading'] == 90 or first_ac['heading'] == 180:
            print('aircraftt', sec_ac['aircraft'], 'should stop')
        if sec_ac['heading'] == 90 or sec_ac['heading'] == 180:
            print('aircraft', first_ac['aircraft'], 'should stop')

        # if first_ac
        # self.constraints.append({'aircraft': y, 'node': [path[x][0]], 'timestep': path[x][1]})
        # pass

    # def run_Individual(aircraft_list, t):

    # traffic_identification()

    # for ac in aircraft_lst:
    #     ID = ac.id
    #
    #     if ac.spawntime == t:  # maybe not necessary
    #         start_node = ac.start  # node from which planning should be done
    #         # print('start_node agent', ID, start_node, 'time', t)
    #         goal_node = ac.goal
    #         ac.status = "taxiing"
    #         ac.position = self.nodes_dict[ac.start]["xy_pos"]
    #         # print('root constraints', root['constraints'])
    #         success, path = astar_CBS(self.nodes_dict, start_node, goal_node, self.heuristics, self.t, ID, self.constraints)
    #
    #         if path is None:
    #             raise BaseException('No solutions')
    #         # print('path', root['paths'])
    #
    #
    #         if success:
    #             ac.path_to_goal = path[1:]
    #             # print("ac_path", ac.path_to_goal, 'time', self.t)
    #             # self.push_locations_to_radar(get_location(ac.path_to_goal, self.t), ID)
    #             next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
    #             ac.from_to = [path[0][0], next_node_id]
    #             print("Path AC", ac.id, ":", path)
    #         else:
    #             raise Exception("No solution found for", ac.id)
    #         # Check the path
    #         if path[0][1] != self.t:
    #             raise Exception("Something is wrong with the timing of the path planning")

    # start_time = timer.time()
    # ac.CPU_time = timer.time() - start_time
    # print("CPU time (s):    {:.2f}".format(ac.CPU_time))

# """
# Implement prioritized planner here
# """
#
# def run_distributed_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
#     for ac1 in aircraft_lst:
#         if ac1.spawntime == t:
#             ac1.position = nodes_dict[ac1.start]["xy_pos"]
#             ac1.plan_independent(nodes_dict, edges_dict, heuristics, t)
#
#         # Create awareness
#         for ac2 in aircraft_lst:
#             if ac1 != ac2:
#                 if ac1.position and ac1.fieldofview and ac2.position and ac2.fieldofview:
#                     if ac1.heading == 0:
#                         if 0 < ac2.position[1] - ac1.position[1] <= ac1.fieldofview[0][1] - ac1.position[1] + 0.1:
#                             if abs(ac2.position[1] - ac1.position[1] / abs(ac2.position[0] - ac1.position[0])) <= (1.5/1.5):
#                                 print(ac2.id, "in view of", ac1.id)
#                     if ac1.heading == 270:
#                         if 0 < ac2.position[0] - ac1.position[0] <= ac1.fieldofview[0][0] - ac1.position[0] + 0.1:
#                             if abs(ac2.position[1] - ac1.position[1]) / abs(ac2.position[0] - ac1.position[0]) <= (1.5/1.5):
#                                 print(ac2.id, "in view of", ac1.id)
#                     if ac1.heading == 180:
#                         if 0 < ac1.position[1] - ac2.position[1] <= ac1.position[1] - ac1.fieldofview[0][1] + 0.1:
#                             if abs(ac2.position[0] - ac1.position[0] / abs(ac1.position[1] - ac2.position[1])) <= (1.5/1.5):
#                                 print(ac2.id, "in view of", ac1.id)
#                     if ac1.heading == 90:
#                         if 0 < ac1.position[0] - ac2.position[0] <= ac1.position[0] - ac1.fieldofview[0][0] + 0.1:
#                             if abs(ac2.position[1] - ac1.position[1]) / abs(ac1.position[0] - ac2.position[0]) <= (1.5/1.5):
#                                 print(ac2.id, "in view of", ac1.id)
#
#         # append to memory. Add weights to expected paths of remembered aircraft. Resolve conflicts that may occur
#
#         # for otheraircraft in aircraft_lst:
#         #     if ac != otheraircraft:
#         #         if ac.heading == 0:
#         #             if otheraircraft.position[1] > ac.position[1]:
#         #                 if otheraircraft.position[1] - ac.position[1]
#
#
#
#     return
