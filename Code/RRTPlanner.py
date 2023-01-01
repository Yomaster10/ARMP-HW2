import numpy as np
from RRTTree import RRTTree
import time

class RRTPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob

        self.step_size = 0.2

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 4.4
        env = self.planning_env
        self.tree.add_vertex(env.start)
        #self.tree.add_vertex(env.goal)
        
        #N_samples = 100
        #for _ in range(N_samples):
        goal_added = False; num_iter = 0
        while not goal_added:
            num_iter += 1
            goal = False; 

            # Here we manually add goal biasing
            p = np.random.uniform()
            if p < self.goal_prob:
                s = env.goal
                goal = True
            else:
                x = np.random.uniform(env.xlimit[0], env.xlimit[1])
                y = np.random.uniform(env.ylimit[0], env.ylimit[1])
                s = [x,y]

            # Partial extensions, if enabled
            if self.ext_mode == 'E2':
                nearest_vert = self.tree.get_nearest_state(s)
                nearest_vert_idx = nearest_vert[0]
                s, goal_added = self.extend(nearest_vert[1], s)
            
            # Is the sample in the free space?
            if env.state_validity_checker(s):
                nearest_vert = self.tree.get_nearest_state(s)
                nearest_vert_idx = nearest_vert[0]
                
                # Does the edge between the sample and its nearest tree node collide with any obstacles?
                if env.edge_validity_checker(s, nearest_vert[1]):
                    s_idx = self.tree.add_vertex(s, nearest_vert)
                    cost = env.compute_distance(s, nearest_vert[1])
                    self.tree.add_edge(nearest_vert_idx,s_idx,cost)
                    if goal == True and self.ext_mode == 'E1':
                        goal_added = True

        if goal_added:
            plan.append(s)
            child_idx = s_idx
            parent_state = nearest_vert[1]
            while self.tree.edges[child_idx]:
                plan.append(parent_state)
                child_idx = self.tree.get_idx_for_state(parent_state)
                # new parent
                parent_idx = self.tree.edges[child_idx] 
                parent_state = self.tree.vertices[parent_idx].state
            plan.append(parent_state)
        plan = plan[::-1]

        #print(self.tree.vertices)
        #print(self.tree.edges)
        print(f"Total number of iterations needed to reach goal: {num_iter}")

        # print total path cost and time
        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f} seconds'.format(time.time()-start_time))

        return np.array(plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 4.4
        cost = self.tree.get_vertex_for_state(plan[-1]).cost
        return cost

    def extend(self, near_state, rand_state):
        '''
        Compute and return a new position for the sampled one.
        @param near_state The nearest position to the sampled position.
        @param rand_state The sampled position.
        '''
        # TODO: Task 4.4
        goal = False
        goal_state = self.planning_env.goal
        if (rand_state[0]==goal_state[0] and rand_state[1]==goal_state[1]):
            goal = True

        vec = [rand_state[i]-near_state[i] for i in range(2)]
        vec_mag = np.sqrt(sum(j**2 for j in vec))
        unit_vec = vec / vec_mag
        new_vec = self.step_size * unit_vec
        new_state = near_state + new_vec

        # check if this intersects the goal or not
        goal_added = False
        if goal:
            if vec_mag < self.step_size:
                new_state = rand_state
                goal_added = True
        return new_state, goal_added