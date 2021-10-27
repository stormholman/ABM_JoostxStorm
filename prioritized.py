"""
Implement prioritized planner here
"""

def run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    constraints = []
    for ac in aircraft_lst:
        if ac.spawntime == t or ac.departtime == t:
            if ac.spawntime == t:
                ac.position = nodes_dict[ac.start]["xy_pos"]
            for otheraircraft in aircraft_lst:
                for i in range(len(otheraircraft.path_to_goal) - 1):
                    constraints.append(
                        {'agent': ac.id,
                         'loc': otheraircraft.path_to_goal[i][0],
                         'timestep': otheraircraft.path_to_goal[i][1]})  # Vertex
                    constraints.append(
                        {'agent': ac.id,
                         'loc': [otheraircraft.path_to_goal[i + 1][0], otheraircraft.path_to_goal[i][0]],
                         'timestep': otheraircraft.path_to_goal[i + 1][1]})  # Edge
            ac.status = "taxiing"
            ac.plan_prioritized(nodes_dict, edges_dict, heuristics, t, constraints)
    return