"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

import os
import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
import random
from single_agent_planner import calc_heuristics
from visualization import map_initialization, map_running
from Aircraft import Aircraft
from Traffic_agent import Traffic_agent
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from distributed2 import run_distributed_planner
from cbs import run_CBS

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

traffic_agents = []

#Parameters that can be changed:
simulation_time = 100
planner = "Distributed" #choose which planner to use (currently only Independent is implemented)

#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.05 #set at 0.1 as default

#%%Function definitions
def import_layout(nodes_file, edges_file, traffic_agents):
    """
    Imports layout information from xlsx files and converts this into dictionaries.
    INPUT:
        - nodes_file = xlsx file with node input data
        - edges_file = xlsx file with edge input data
    RETURNS:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - start_and_goal_locations = dictionary with node ids for arrival runways, departure runways and gates 
    """
    gates_xy = []   #lst with (x,y) positions of gates
    rwy_dep_xy = [] #lst with (x,y) positions of entry points of departure runways
    rwy_arr_xy = [] #lst with (x,y) positions of exit points of arrival runways
    
    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)
    
    #Create nodes_dict from df_nodes
    nodes_dict = {}
    for i, row in df_nodes.iterrows():
        node_properties = {"id": row["id"],
                           "x_pos": row["x_pos"],
                           "y_pos": row["y_pos"],
                           "xy_pos": (row["x_pos"],row["y_pos"]),
                           "type": row["type"],
                           "neighbors": set(),
                           "heading": set()
                           }
        node_id = row["id"]
        nodes_dict[node_id] = node_properties
        
        #Add node type
        if row["type"] == "rwy_d":
            rwy_dep_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "rwy_a":
            rwy_arr_xy.append((row["x_pos"],row["y_pos"]))
        elif row["type"] == "gate":
            gates_xy.append((row["x_pos"],row["y_pos"]))

        if row["type"] == "intersection":
            traffic_agents.append(Traffic_agent(node_properties["id"],
                                                node_properties["x_pos"],
                                                node_properties["y_pos"],
                                                node_properties["xy_pos"],
                                                node_properties["type"],
                                                node_properties["neighbors"],
                                                node_properties["heading"]))

    #Specify node ids of gates, departure runways and arrival runways in a dict
    start_and_goal_locations = {"gates": gates_xy, 
                                "dep_rwy": rwy_dep_xy,
                                "arr_rwy": rwy_arr_xy}

    # Create edges_dict from df_edges
    edges_dict = {}
    edges_dict2 = {}
    for i, row in df_edges.iterrows():
        edge_id = (row["from"], row["to"])
        edge_id2 = (row["from"], row["to"], row["heading"])
        from_node = edge_id[0]
        to_node = edge_id[1]
        start_end_pos = (nodes_dict[from_node]["xy_pos"], nodes_dict[to_node]["xy_pos"])
        edge_properties = {"id": edge_id,
                           "from": row["from"],
                           "to": row["to"],
                           "length": row["length"],
                           "weight": row["length"],
                           "start_end_pos": start_end_pos,
                           "heading": row['heading']
                           }

        edges_dict[edge_id] = edge_properties
        edges_dict2[edge_id2] = edge_properties
        # edges_properties.append(edge_properties) #generate list of edgesproperties
    # print(edges_dict)
    # Add neighbor nodes to nodes_dict based on edges between nodes
    for edge2 in edges_dict2:
        # print('edge2', edge2)
        from_node = edge2[0]
        to_node = edge2[1]
        heading = edge2[2]
        nodes_dict[from_node]["heading"].add((to_node, heading))

    # Add neighbor nodes to nodes_dict based on edges between nodes
    for edge in edges_dict:
        # print('edge',edge)
        from_node = edge[0]
        to_node = edge[1]
        nodes_dict[from_node]["neighbors"].add(to_node)
    
    return nodes_dict, edges_dict, start_and_goal_locations

def xy_to_node(xy):
    for x in range(1, len(nodes_dict)+1):
        # print(len(nodes_dict))
        if nodes_dict[x]['xy_pos'] == xy:
            return nodes_dict[x]['id']

def create_graph(nodes_dict, edges_dict, plot_graph = True):
    """
    Creates networkX graph based on nodes and edges and plots 
    INPUT:
        - nodes_dict = dictionary with nodes and node properties
        - edges_dict = dictionary with edges annd edge properties
        - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
    RETURNS:
        - graph = networkX graph object
    """
    
    graph = nx.DiGraph() #create directed graph in NetworkX
    
    #Add nodes and edges to networkX graph
    for node in nodes_dict.keys():
        graph.add_node(node, 
                       node_id = nodes_dict[node]["id"],
                       xy_pos = nodes_dict[node]["xy_pos"],
                       node_type = nodes_dict[node]["type"])
        
    for edge in edges_dict.keys():
        graph.add_edge(edge[0], edge[1], 
                       edge_id = edge,
                       from_node =  edges_dict[edge]["from"],
                       to_node = edges_dict[edge]["to"],
                       weight = edges_dict[edge]["length"])
    
    #Plot networkX graph
    if plot_graph:
        plt.figure()
        node_locations = nx.get_node_attributes(graph, 'xy_pos')
        nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)
        
    return graph

def aircraftplanner():
    for ac in aircraft_lst:
        if ac.from_to != [0, 0]:
            ac.start = ac.from_to[0] # Review
    if planner == "Independent":
        run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
    elif planner == "Prioritized":
        run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
    elif planner == "CBS":
        run_CBS(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
    elif planner == "Distributed":
        run_distributed_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, traffic_agents)

    else:
        raise Exception("Planner:", planner, "is not defined.")

#%% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file, traffic_agents)
graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)

aircraft_lst = []   #List which can contain aircraft agents

if visualization:
    map_properties = map_initialization(nodes_dict, edges_dict) #visualization properties

# =============================================================================
# 1. While loop and visualization
# =============================================================================
 
#Start of while loop    
running=True
escape_pressed = False
time_end = simulation_time
dt = 0.1 #should be factor of 0.5 (0.5/dt should be integer)
t= 0
numberOfAircraft = 0
availableGates = [98, 36, 35, 34, 97]
occupiedGates = []
seedtype = 0
boarding_time = 3
gatechoice = 0

print("Simulation Started")
while running:
    t= round(t,2)

    if planner == "Distributed" and t % 0.5 == 0:
        aircraftplanner()

    #Check conditions for termination
    if t >= time_end or escape_pressed: 
        running = False
        pg.quit()
        print("Simulation Stopped")
        break 
    
    #Visualization: Update map if visualization is true
    if visualization:
        current_states = {} #Collect current states of all aircraft
        for ac in aircraft_lst:
            if ac.status != "departed":
                current_states[ac.id] = {"ac_id": ac.id,
                                         "xy_pos": ac.position,
                                         "heading": ac.heading,
                                         "fieldofview": ac.fieldofview,
                                         "observations": ac.observations}
        escape_pressed = map_running(map_properties, current_states, t)
        timer.sleep(visualization_speed)

    seedtype += 2
    if (t % 2 == 0): # use this
        if len(availableGates) > 0:
            gatechoice += 1
            if gatechoice == 5:
                gatechoice = 0

            random.seed(seedtype)
            entry = random.choice([37, 38])
            goal = availableGates[gatechoice]
            #availableGates.remove(goal)
            #occupiedGates.append(goal)

            ac = Aircraft(numberOfAircraft, 'A', entry, goal, t, nodes_dict)
            aircraft_lst.append(ac)
            aircraftplanner()
            numberOfAircraft += 1

    for ac in aircraft_lst: # implement time for boarding
        if ac.status == "at_gate":
            ac.status = "boarding {}".format(t)
    for ac in aircraft_lst:
        if ac.status == "boarding {}".format(t-boarding_time):
            ac.status = "boarding completed"

    for ac in aircraft_lst:
        if ac.status == "boarding completed":
            ac.status = "taxiing"
            ac.departtime = t
            availableGates.append(ac.gate)
            aircraftplanner()

    #Move the aircraft that are taxiing
    for ac in aircraft_lst:

        if ac.status == "taxiing" or ac.status == "at_gate":
            ac.move(dt, t)


    t = t + dt
          
# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?
#test github
