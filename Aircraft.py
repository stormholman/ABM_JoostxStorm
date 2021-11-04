from single_agent_planner import simple_single_agent_astar
import math
import random


class Aircraft(object):
    """Aircraft class, should be used in the creation of new aircraft."""

    def __init__(self, flight_id, a_d, start_node, goal_node, spawn_time, nodes_dict, weight_class):
        """
        Initalisation of aircraft object.
        INPUT:
            - flight_id: [int] unique id for this aircraft
            - a_d: [str] "a" for arrival flight and "d" for departure flight
            - start_node: node_id of start node
            - goal_node: node_id of goal node
            - spawn_time: spawn_time of a/c
            - nodes_dict: copy of the nodes_dict
        """

        # Fixed parameters
        self.speed = 1  # how much a/c moves per unit of t
        self.id = flight_id  # flight_id
        self.type = a_d  # arrival or departure (A/D)
        self.spawntime = spawn_time  # spawntime
        self.start = start_node  # start_node_id
        self.goal = goal_node  # goal_node_id
        self.nodes_dict = nodes_dict  # keep copy of nodes dict
        self.waittime = 3
        self.gatearrival = None
        self.departtime = None
        self.weight_class = weight_class
        if self.weight_class == "heavy":
            self.boarding_time = 4
        if self.weight_class == "small":
            self.boarding_time = 2

        # Route related
        self.status = "taxiing"
        self.gate = goal_node  # (gate, arrival time)
        self.route = [start_node]
        self.total_path = []  # path to goal including current node
        self.path_to_goal = []  # planned path left from current location
        self.from_to = [0, 0]
        self.lastdifferentnode = None
        self.constraints = []

        # State related
        self.heading = 0
        self.position = (0, 0)  # xy position on map
        self.fieldofview = None
        self.observations = []

    def get_heading(self, xy_start, xy_next):
        """
        Determines heading of an aircraft based on a start and end xy position.
        INPUT:
            - xy_start = tuple with (x,y) position of start node
            - xy_next = typle with (x,y) position of next node
        RETURNS:
            - heading = heading of aircraft in degrees
        """

        if xy_start[0] == xy_next[0]:  # moving up or down
            if xy_start[1] > xy_next[1]:  # moving down
                heading = 180
            elif xy_start[1] < xy_next[1]:  # moving up
                heading = 0
            else:
                heading = self.heading

        elif xy_start[1] == xy_next[1]:  # moving right or left
            if xy_start[0] > xy_next[0]:  # moving left
                heading = 90
            elif xy_start[0] < xy_next[0]:  # moving right
                heading = 270
            else:
                heading = self.heading
        else:
            raise Exception("Invalid movement")

        self.heading = heading

    def move(self, dt, t):
        """
        Moves an aircraft between from_node and to_node and checks if to_node or goal is reached.
        INPUT:
            - dt =
            - t =
        """

        # Determine nodes between which the ac is moving
        from_node = self.from_to[0]
        to_node = self.from_to[1]
        xy_from = self.nodes_dict[from_node]["xy_pos"]  # xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"]  # xy position of to node
        distance_to_move = self.speed * dt  # distance to move in this timestep

        if t % 0.5 == 0:
            self.route.append(to_node)

        # Update position with rounded values
        if xy_to[0] - xy_from[0] == 0 and xy_to[1] - xy_from[1] == 0:
            x_normalized = 0
            y_normalized = 0
        else:
            x = xy_to[0] - xy_from[0]
            y = xy_to[1] - xy_from[1]
            x_normalized = x / math.sqrt(x ** 2 + y ** 2)
            y_normalized = y / math.sqrt(x ** 2 + y ** 2)
        posx = round(self.position[0] + x_normalized * distance_to_move, 2)  # round to prevent errors
        posy = round(self.position[1] + y_normalized * distance_to_move, 2)  # round to prevent errors
        self.position = (posx, posy)
        self.get_heading(xy_from, xy_to)

        # Create field of view
        if self.heading == 0:
            self.fieldofview = [(self.position[0] - 1.5, self.position[1] + 1.5),
                                (self.position[0] + 1.5, self.position[1] + 1.5)]
        if self.heading == 90:
            self.fieldofview = [(self.position[0] - 1.5, self.position[1] - 1.5),
                                (self.position[0] - 1.5, self.position[1] + 1.5)]
        if self.heading == 180:
            self.fieldofview = [(self.position[0] - 1.5, self.position[1] - 1.5),
                                (self.position[0] + 1.5, self.position[1] - 1.5)]
        if self.heading == 270:
            self.fieldofview = [(self.position[0] + 1.5, self.position[1] - 1.5),
                                (self.position[0] + 1.5, self.position[1] + 1.5)]

        # Remember last node before present node. Used to restrict aircraft from turning around.
        if from_node != to_node:
            self.lastdifferentnode = from_node

        # Check if goal is reached or if to_node is reached
        if self.position == xy_to and self.path_to_goal[0][
            1] == t + dt:  # If with this move its current to node is reached
            if self.position == self.nodes_dict[self.goal]["xy_pos"]:  # if the final goal is reached
                if self.type == "A" and self.status != "at_gate":
                    self.status = "at_gate"
                    self.gatearrival = t
                    self.from_to[0] = self.from_to[1]
                    if self.weight_class == "heavy":
                        self.goal = 2
                    if self.weight_class == "small":
                        self.goal = 1
                    self.type = "D"

                    # remove node 56 as a route option. This is to prevent congestion at runway exits.
                    self.nodes_dict[11]['neighbors'] = {11.0, 55.0, 62.0}
                    self.nodes_dict[12]['neighbors'] = {12.0, 57.0, 63.0}

                    # # remove node 44 to prevent "clogged taxiways" (this is the easy way out)
                    # self.nodes_dict[4]['neighbors'] = {43.0, 4.0, 95.0}
                    # self.nodes_dict[5]['neighbors'] = {96.0, 5.0, 45.0}

                elif self.type == "D":
                    self.status = "departed"
                    self.departtime = t
                    print('Aircraft', self.id, self.status, ". Total route: ")
                    print(self.route, print("(Length: ", len(self.route), ", Time spend: ",
                                            round(self.departtime - self.spawntime + 0.1), ")"))

            else:  # current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]

                new_from_id = self.from_to[1]  # new from node
                new_next_id = self.path_to_goal[0][0]  # new to node

                if new_from_id != self.from_to[0]:
                    self.last_node = self.from_to[0]

                self.from_to = [new_from_id, new_next_id]  # update new from and to node
                self.total_path = remaining_path

    def plan_independent(self, nodes_dict, edges_dict, heuristics, t):
        """
        Plans a path for taxiing aircraft assuming that it knows the entire layout.
        Other traffic is not taken into account.
        INPUT:
            - nodes_dict: copy of the nodes_dict
            - edges_dict: edges_dict with current edge weights
        """
        if self.status == "taxiing":
            start_node = self.start  # node from which planning should be done
            goal_node = self.goal  # node to which planning should be done
            success, path = simple_single_agent_astar(self.id, nodes_dict, start_node, goal_node, heuristics, t, [])

            if success:
                self.total_path = path
                self.path_to_goal = path[1:]
                next_node_id = self.path_to_goal[0][0]  # next node is first node in path_to_goal
                self.from_to = [path[0][0], next_node_id]
                # print("Path AC", self.id, ":", path)
            else:
                raise Exception("No solution found for", self.id)

            # Check the path
            if path[0][1] != t:
                raise Exception("Something is wrong with the timing of the path planning")

    def plan_prioritized(self, nodes_dict, edges_dict, heuristics, t, constraints):
        if self.type == "A":
            start_node = self.start  # node from which planning should be done
            goal_node = self.goal  # node to which planning should be done

        if self.type == "D":
            start_node = self.gate
            goal_node = self.goal

        success, path = simple_single_agent_astar(self.id, nodes_dict, start_node, goal_node, heuristics, t,
                                                  constraints)
        if success:
            self.path_to_goal = path[1:]
            self.route.append(path)
            next_node_id = self.path_to_goal[0][0]  # next node is first node in path_to_goal
            self.from_to = [path[0][0], next_node_id]
        else:
            raise Exception("No solution found for", self.id)
        # Check the path
        if path[0][1] != t:
            raise Exception("Something is wrong with the timing of the path planning")

        def plan_distributed(self, nodes_dict, edges_dict, heuristics, t):
            pass