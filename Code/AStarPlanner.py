import numpy as np
import heapdict

class AStarPlanner(object):
    def __init__(self, planning_env):
        self.planning_env = planning_env
        # used for visualizing the expanded nodes
        # make sure that this structure will contain a list of positions (states, numpy arrays) without duplicates
        self.expanded_nodes = []
        self.parent = []
        self.gList = {}
        self.open = heapdict.heapdict()
        self.close = heapdict.heapdict()
        self.counter = 0

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''

        # initialize an empty plan.
        plan = []
        # TODO: Task 4.3
        h_weight = 1
        env = self.planning_env
        self.parent = [None for _ in range((env.xlimit[1]+1) * (env.ylimit[1]+1))]
        initial_state = env.start
        initial_state = self.to_state(initial_state[0],initial_state[1])
        self.gList[initial_state] = 0
        self.open[initial_state] = (self.heuristic(env, initial_state, 0, h_weight),
                                    env.start)  # append initial state and 'distance' to goal
        while self.open:
            (current_state, priority) = self.open.popitem()
            self.close[current_state] = (priority[0], current_state)
            if self.is_final_state(current_state):
                return self.list_path(self.parent, current_state, env)
            else:
                self.counter += 1
                self.expanded_nodes.append(self.to_row_col(current_state))
                for neighbour_state in self.state_neighbors(current_state):
                    if neighbour_state is not None:
                        new_g = self.gList[current_state] + self.cost_to_go(env, current_state, neighbour_state)
                        new_f = self.heuristic(env, neighbour_state, new_g,
                                               h_weight)  # generate possible better heuristic for neighbourstate
                    if (neighbour_state not in self.open) and (neighbour_state not in self.close) and (
                            neighbour_state is not None):
                        self.parent[neighbour_state] = current_state
                        self.gList[neighbour_state] = new_g
                        self.open[neighbour_state] = (self.heuristic(env, neighbour_state, new_g, h_weight), neighbour_state)
                    elif (neighbour_state in self.open) and (neighbour_state is not None):
                        curr_heuristic,some_state = self.open[neighbour_state]
                        if self.heuristic(env, neighbour_state, new_g, h_weight) < curr_heuristic:
                            self.parent[neighbour_state] = current_state
                            self.open.pop(neighbour_state)
                            self.gList[neighbour_state] = new_g
                            self.open[neighbour_state] = (self.heuristic(env, neighbour_state, new_g, h_weight), neighbour_state)
                    else:
                        curr_heuristic,some_state = self.close[neighbour_state]
                        if (neighbour_state is not None) and (self.heuristic(env, neighbour_state, new_g, h_weight) <  curr_heuristic):
                            self.parent[neighbour_state] = current_state
                            self.close.pop(neighbour_state)
                            self.close[neighbour_state] = (self.heuristic(env, neighbour_state, new_g,h_weight),neighbour_state)
                            self.gList[neighbour_state] = new_g
        return np.array(plan)

    def get_expanded_nodes(self):
        '''
        Return list of expanded nodes without duplicates.
        '''

        # used for visualizing the expanded nodes
        return self.expanded_nodes

    def cost_to_go(self, env, state,neighbor_state):
        return self.planning_env.compute_distance(self.to_row_col(state),self.to_row_col(neighbor_state))

    def cost_to_come(self, env, state):
        goal = env.goal
        return env.compute_distance(self.to_row_col(state),self.to_row_col(goal)[1])

    def heuristic(self, env, state, g_cost, h_weight):
        h_cost = self.cost_to_come(env, state)
        cost = g_cost + h_weight*h_cost
        return cost

    def cost(self, env, state):
        current_state = state
        cost = 0
        while self.parent[current_state] is not None:
            cost += self.cost_to_go(env, self.parent[current_state], current_state)
            current_state = self.parent[current_state]
        return cost

    def list_path(self, parent, end_state, env):
        # create path with positional values
        path = []
        current_state = end_state
        path.append(self.to_row_col(current_state))
        while parent[current_state] is not None:
            current_state = parent[current_state]
            current_state_row_col = self.to_row_col(current_state)
            path.append(current_state_row_col)
        path.reverse()
        new_path = np.array(path)
        return new_path

    def state_neighbors(self, current_state):
        x,y = self.to_row_col(current_state)
        current_state_array = np.array([x,y])
        d = [-1, 0, 1]
        neighbors = []
        neighbors_states = []
        for dx in d:
            for dy in d:
                if dx == 0 and dy == 0:
                    continue
                neighbor_x = x + dx
                neighbor_y = y + dy
                neighbors.append(np.array([neighbor_x,neighbor_y]))
        i=0
        bad_neighbors = []
        for neighbor in neighbors:
            if ((self.planning_env.state_validity_checker(neighbor) == False) or (self.planning_env.edge_validity_checker(current_state_array,neighbor)==False)):
                bad_neighbors.append(i)
            i+=1
        i=0
        for neighbor in neighbors:
            if(i not in bad_neighbors):
                neighbors_states.append(self.to_state(neighbor[0],neighbor[1]))
            i+=1
        return neighbors_states

    def to_state(self, row: int, col: int) -> int:
        #Converts between location on the board and state.
        return row * (self.planning_env.ylimit[1]+1) + col

    def to_row_col(self, state: int):
        #Converts between state and location on the board.
        return np.array([state // ((self.planning_env.ylimit[1])+1), state % ((self.planning_env.ylimit[1])+1)])

    def is_final_state(self,state):
        if state == self.to_state(self.planning_env.goal[0],self.planning_env.goal[1]):
            return True
