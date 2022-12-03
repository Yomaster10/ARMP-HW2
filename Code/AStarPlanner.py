import numpy as np

class AStarPlanner(object):    
    def __init__(self, planning_env):
        self.planning_env = planning_env

        # used for visualizing the expanded nodes
        # make sure that this structure will contain a list of positions (states, numpy arrays) without duplicates
        self.expanded_nodes = [] 

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''

        # initialize an empty plan.
        plan = []

        # TODO: Task 4.3

        return np.array(plan)

    def get_expanded_nodes(self):
        '''
        Return list of expanded nodes without duplicates.
        '''

        # used for visualizing the expanded nodes
        return self.expanded_nodes
