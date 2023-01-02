import numpy as np
from RRTTree import RRTTree
import time

from TableCreator import update_table

class RRTStarPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob, k):
        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.k = k

        # set step size for extensions
        if planning_env.ylimit[1] < 100:
            self.step_size = 0.2
        else:
            self.step_size = 10

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''
        start_time = time.time()
        
        env = self.planning_env
        self.tree.add_vertex(env.start)

        log = False
        if self.k == 0: # log mode
            log = True

        goal_added = False; num_iter = 0; num_rewires = 0; plan = []
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

            # Is the sample in the free space?
            if env.state_validity_checker(s):
                nearest_vert = self.tree.get_nearest_state(s)
                nearest_vert_idx = nearest_vert[0]

                # Partial extensions, if enabled
                if self.ext_mode == 'E2':
                    s, goal_added = self.extend(nearest_vert[1], s) # s = x_new
                    if not env.state_validity_checker(s):
                        continue
                
                # Does the edge between the sample and its nearest tree node collide with any obstacles?
                if env.edge_validity_checker(s, nearest_vert[1]):
                    s_idx = self.tree.add_vertex(s, nearest_vert)
                    cost = env.compute_distance(s, nearest_vert[1])
                    self.tree.add_edge(nearest_vert_idx,s_idx,cost)
                    if goal == True and self.ext_mode == 'E1':
                        goal_added = True
                    if log == True:
                        self.k = int(2*np.log10(len(self.tree.vertices)))
                    # rewiring phase
                    if len(self.tree.vertices) > self.k:
                        knn_idxs, knn_states = self.tree.get_k_nearest_neighbors(s, self.k)
                        for i in range(len(knn_states)):
                            if knn_idxs[i] == s_idx:
                                continue
                            if env.edge_validity_checker(knn_states[i],s):
                                old_cost = self.tree.vertices[s_idx].cost
                                # calculating the potential new cost for the sample
                                c = env.compute_distance(knn_states[i],s)
                                potential_parent_cost = self.tree.vertices[knn_idxs[i]].cost
                                potential_new_cost = potential_parent_cost + c
                                # checking for improvement
                                if potential_new_cost < old_cost:
                                    self.tree.vertices[s_idx].cost = potential_new_cost
                                    self.tree.edges[s_idx] = knn_idxs[i]
                                    num_rewires += 1
                        for i in range(len(knn_states)):
                            if knn_idxs[i] == s_idx:
                                continue
                            if env.edge_validity_checker(s,knn_states[i]):
                                old_cost = self.tree.vertices[knn_idxs[i]].cost
                                # calculating the potential new cost for the neighbors
                                c = env.compute_distance(s,knn_states[i])
                                potential_parent_cost = self.tree.vertices[s_idx].cost
                                potential_new_cost = potential_parent_cost + c
                                # checking for improvement
                                if potential_new_cost < old_cost:
                                    self.tree.vertices[knn_idxs[i]].cost = potential_new_cost
                                    self.tree.edges[knn_idxs[i]] = s_idx
                                    num_rewires += 1
                else:
                    goal_added = False
                    
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

        print(f"Total number of iterations needed to reach goal: {num_iter}")
        print(f"Total number of rewirings conducted: {num_rewires}")

        # print total path cost and time
        total_time = time.time()-start_time
        total_cost = self.compute_cost(plan)
        print('Total cost of path: {:.3f}'.format(total_cost))
        print('Total time: {:.3f} seconds'.format(total_time))
        
        UpdateTable = False
        if UpdateTable:
            if env.start[0] == 10:
                map = 'M1'
            elif env.start[0] == 250:
                map = 'M2'
            else:
                map = 'Unknown'

            if log==True:
                self.k='floor[2*log(n)]'

            update_table(planner='rrtstar', map=map, ext_mode=self.ext_mode, goal_bias=self.goal_prob, step_size=self.step_size,
                num_iter=num_iter, time=total_time, cost=total_cost, k=self.k, num_rewires=num_rewires)
                
        return np.array(plan)

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps.
        @param plan A given plan for the robot.
        '''
        return self.tree.get_vertex_for_state(plan[-1]).cost

    def extend(self, near_state, rand_state):
        '''
        Compute and return a new position for the sampled one.
        @param near_state The nearest position to the sampled position.
        @param rand_state The sampled position.
        '''
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