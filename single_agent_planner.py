"""
This file contains single agent planner functions  that can be used by other planners.
Consider functions in this file as supporting functions.
"""

import heapq
import networkx as nx


def calc_heuristics(graph, nodes_dict):
    """
    Calculates the exact heuristic dict (shortest distance between two nodes) to be used in A* search.
    INPUT:
        - graph = networkX graph object
        - nodes_dict = dictionary with nodes and node properties
    RETURNS:
        - heuristics = dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
    """

    heuristics = {}
    for i in nodes_dict:
        heuristics[nodes_dict[i]["id"]] = {}
        for j in nodes_dict:
            path, path_length = heuristicFinder(graph, nodes_dict[i]["id"], nodes_dict[j]["id"])
            if path == False:
                pass
            else:
                heuristics[nodes_dict[i]["id"]][nodes_dict[j]["id"]] = path_length
    return heuristics


def heuristicFinder(graph, start_node, goal_node):
    """
    Finds exact distance between start_node and goal_node using the NetworkX graph.
    INPUT:
        - graph = networkX graph object
        - start_node, goal_node [int] = node_ids of start and goal node
    RETURNS:
        - path = list with node_ids that are part of the shortest path
        - path_length = length of the shortest path
    """
    try:
        path = nx.dijkstra_path(graph, start_node, goal_node, weight="weight")
        path_length = nx.dijkstra_path_length(graph, start_node, goal_node)
    except:
        path = False
        path_length = False
        raise Exception('Heuristic cannot be calculated: No connection between', start_node, "and", goal_node)
    return path, path_length

def build_constraint_table(constraints, agent):
    constraint_table = []
    for constraint in constraints:
        if constraint['agent'] == agent:
                constraint_table.append((constraint['loc'], constraint['timestep']))
    return constraint_table
#
# def is_constrained(curr_loc, next_loc, next_time, constraint_table):
#     for node in constraint_table:
#         if isinstance(node[0], list):
#             print(node)
#             if node[0][0] == curr_loc and node[0][1] == next_loc and node[1] == next_time:
#                 # print((next_loc, next_time))
#                 # print('table', constraint_table)
#                 # print('edge')
#                 return True
#         elif node == (next_loc, next_time): # vertex
#             # print(([curr_loc, next_loc], next_time))
#             # print('table', constraint_table)
#             # print('vertex')
#             return True
#         else:
#             return False

def is_constrained(constraints, agent, curr_loc, next_loc, next_time):
    for constraint in constraints:
        # print(agent)
        # print(constraint)
        if constraint['agent'] == agent:

            # print(constraint['loc'])
            # print(curr_loc)
            # print(next_loc)
            if isinstance(constraint['loc'], list):
                if constraint['loc'][0] == curr_loc and constraint['loc'][1] == next_loc and constraint['timestep'] == next_time:
                    return True
            elif constraint['loc'] == next_loc:
                # print(constraint['loc'])
                # print(next_loc)
                if constraint['timestep'] == next_time:
                    # print(constraint['timestep'])
                    # print(next_time)
                    return True

def simple_single_agent_astar(agent, nodes_dict, from_node, goal_node, heuristics, time_start, constraints = [],
                              lastdifferentnode = None):
    # def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time. 
        - Hint: do you need more inputs?
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep) pairs -> example [(37, 1), (101, 2)]. Empty list if success == False.
    """

    from_node_id = from_node
    goal_node_id = goal_node
    time_start = time_start
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    # print(from_node_id)
    # print(goal_node_id)
    # print(heuristics)
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    # constraint_table = build_constraint_table(constraints, agent)

    print('Running astar for agent', agent, ', constraints: ', constraints)
    while len(open_list) > 0:
        # print(len(open_list))
        curr = pop_node(open_list)
        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr)

        nodes_dict[curr['loc']]["neighbors"].add(curr['loc'])  # Also consider "waiting"

        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            child = {'loc': neighbor,
                     'g_val': curr['g_val'] + 0.5,
                     'h_val': heuristics[neighbor][goal_node_id],
                     'parent': curr,
                     'timestep': curr['timestep'] + 0.5}

            if is_constrained(constraints, agent, curr['loc'], child['loc'], child['timestep']):
                continue

            # Ensure aircraft cannot reverse direction. The while loop finds the last different node if the agent
            # has been waiting.

            ### Legacy (need to change prioritized/cbs as well) ###
            if curr['parent']:
                lastDifferentNode1 = curr['parent']
                while lastDifferentNode1['parent'] and lastDifferentNode1['loc'] == curr['loc']:
                    lastDifferentNode1 = lastDifferentNode1['parent']

                # If child location reverses the aircraft do not consider that child node.
                if child['loc'] == lastDifferentNode1['loc']:
                    continue

            ### New ###
            if child['loc'] == lastdifferentnode:
                continue


            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
    print("No path found for ", agent, ", " + str(len(closed_list)) + " nodes visited")
    return False, []  # Failed to find solutions


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def get_path(goal_node):
    """Construct path if goal node is reached"""
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['timestep']))
        curr = curr['parent']
    path.reverse()
    # print(path)
    return path
