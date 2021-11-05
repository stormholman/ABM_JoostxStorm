"""
Implement prioritized planner here
"""

def run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    constraints = []
    # for ac1 in aircraft_lst:
    #     for ac2 in aircraft_lst:
    #         if ac1 != ac2:
    #             if ac1.position == (1.5, 5) or ac1.position == (
    #                     1.5, 4):  # don't let aircraft depart at the same time from rw 1 or 2
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
    #                     ac1.plan_prioritized(nodes_dict, edges_dict, heuristics, t, constraints)
    #                     ac2.plan_prioritized(nodes_dict, edges_dict, heuristics, t, constraints)
    #                     # simple_single_agent_astar(ac1, nodes_dict, ac1.from_to[0], ac1.goal, heuristics, t, constraints,
    #                     #           ac1.lastdifferentnode)
    #                     # simple_single_agent_astar(ac2, nodes_dict, ac2.from_to[0], ac2.goal, heuristics, t, constraints,
    #                     #           ac2.lastdifferentnode)
    #
    #             if ac1.position == (
    #                     1.5, 4):  # small ac waits 0.5 sec for vertex turbulence to disappear caused by heavy ac
    #                 if ac2.position == (2, 5) and ac2.path_to_goal[0][0] == 95:
    #                     constraints.append({'agent': ac2.id, 'loc': 1, 'timestep': t + 1})
    #                     ac2.plan_prioritized(nodes_dict, edges_dict, heuristics, t, constraints)
    #                     # simple_single_agent_astar(ac2, nodes_dict, ac2.from_to[0], ac2.goal, heuristics, t, constraints,
    #                     #           ac2.lastdifferentnode)
    #                     print("Aircraft", ac2.id, "waits due to heavy vortex from aircraft", ac1.id)
    
    
    # if len(ac.path_to_goal) > 2 and len(otheraircraft.path_to_goal) > 2:
#     if ac.path_to_goal[-2][1] == otheraircraft.path_to_goal[-2][1]:
#         if ac.weight_class == "Heavy" and otheraircraft.weight_class == "Small":  # smallest aircraft gives way
#             constraints.append({'agent': ac.id, 'loc': ac.path_to_goal[-1][0], 'timestep': ac.path_to_goal[-1][1]})
#         elif ac.weight_class == "Small" and otheraircraft.weight_class == "Heavy":
#             constraints.append({'agent': otheraircraft.id, 'loc': otheraircraft.path_to_goal[-1][0], 'timestep': otheraircraft.path_to_goal[-1][1]})
#         else:
#             if ac.id < otheraircraft.id:  # largest id gives way
#                 constraints.append({'agent': otheraircraft.id, 'loc': otheraircraft.path_to_goal[-1][0], 'timestep': otheraircraft.path_to_goal[-1][1]})
#             else:
#                 constraints.append({'agent': ac.id, 'loc': ac.path_to_goal[-1][0],
#                                     'timestep': ac.path_to_goal[-1][1]})

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
