import operator
import numpy as np

class RRTTree(object):
    
    def __init__(self, planning_env, task="mp"):
        
        self.planning_env = planning_env
        self.task = task
        self.vertices = {}
        self.edges = {}

    def get_root_id(self):
        '''
        Returns the ID of the root in the tree.
        '''
        return 0

    def add_vertex(self, state, inspected_points=None):
        '''
        Add a state to the tree.
        @param state state to add to the tree.
        '''
        vid = len(self.vertices)
        self.vertices[vid] = RRTVertex(state=state, inspected_points=inspected_points)

        return vid

    def add_edge(self, sid, eid, edge_cost):
        '''
        Adds an edge in the tree.
        @param sid start state ID
        @param eid end state ID
        '''
        self.edges[eid] = sid
        self.vertices[eid].set_cost(cost=self.vertices[sid].cost + edge_cost)

    def is_goal_exists(self, state):
        '''
        Check if goal exists.
        @param state state to check if exists.
        '''
        goal_idx = self.get_idx_for_state(state=state)
        if goal_idx is not None:
            return True
        return False

    def get_vertex_for_state(self, state):
        '''
        Search for the vertex with the given state and return it if exists
        @param state state to check if exists.
        '''
        v_idx = self.get_idx_for_state(state=state)
        if v_idx is not None:
            return self.vertices[v_idx]
        return None

    def get_idx_for_state(self, state):
        '''
        Search for the vertex with the given state and return the index if exists
        @param state state to check if exists.
        '''
        valid_idxs = [v_idx for v_idx, v in self.vertices.items() if (v.state == state).all()]
        if len(valid_idxs) > 0:
            return valid_idxs[0]
        return None

    def get_nearest_state(self, state):
        '''
        Find the nearest vertex for the given state and returns its state index and state
        @param state Sampled state.
        '''
        # compute distances from all vertices
        dists = []
        for _, vertex in self.vertices.items():
            dists.append(self.planning_env.compute_distance(state, vertex.state))

        # retrieve the id of the nearest vertex
        vid, _ = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid].state

    def get_k_nearest_neighbors(self, state, k):
        '''
        Return k-nearest neighbors
        @param state Sampled state.
        @param k Number of nearest neighbors to retrieve.
        '''
        dists = []
        for _, vertex in self.vertices.items():
            dists.append(self.planning_env.compute_distance(state, vertex.state))

        dists = np.array(dists)
        knn_ids = np.argpartition(dists, k)[:k]
        #knn_dists = [dists[i] for i in knn_ids]

        return knn_ids.tolist(), [self.vertices[vid].state for vid in knn_ids]

    def get_edges_as_states(self):
        '''
        Return the edges in the tree as a list of pairs of states (positions)
        '''

        return [[self.vertices[val].state,self.vertices[key].state] for (key, val) in self.edges.items()]

class RRTVertex(object):

    def __init__(self, state, cost=0, inspected_points=None):

        self.state = state
        self.cost = cost
        self.inspected_points = inspected_points

    def set_cost(self, cost):
        '''
        Set the cost of the vertex.
        '''
        self.cost = cost