import os
import time
from datetime import datetime
import json
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
from matplotlib import patches as pat
from matplotlib import collections as coll
from numpy.core.fromnumeric import size
from shapely.geometry import Point, LineString, Polygon
import imageio

class MapEnvironment(object):
    
    def __init__(self, json_file):

        # check if json file exists and load
        json_path = os.path.join(os.getcwd(), json_file)
        if not os.path.isfile(json_path):
            raise ValueError('Json file does not exist!');
        with open(json_path) as f:
            json_dict = json.load(f)

        # obtain boundary limits, start and inspection points
        self.xlimit = [0, json_dict['WIDTH']-1]
        self.ylimit = [0, json_dict['HEIGHT']-1]
        self.start = np.array(json_dict['START'])
        self.goal = np.array(json_dict['GOAL'])
        self.load_obstacles(obstacles=json_dict['OBSTACLES'])

        # check that the start location is within limits and collision free
        if not self.state_validity_checker(state=self.start):
            raise ValueError('Start state must be within the map limits');

        # check that the goal location is within limits and collision free
        if not self.state_validity_checker(state=self.goal):
            raise ValueError('Goal state must be within the map limits');

        # if you want to - you can display starting map here
        #self.visualize_map()

    def load_obstacles(self, obstacles):
        '''
        A function to load and verify scene obstacles.
        @param obstacles A list of lists of obstacles points.
        '''
        # iterate over all obstacles
        self.obstacles, self.obstacles_edges = [], []
        for obstacle in obstacles:
            non_applicable_vertices = [x[0] < self.xlimit[0] or x[0] > self.xlimit[1] or x[1] < self.ylimit[0] or x[1] > self.ylimit[1] for x in obstacle]
            if any(non_applicable_vertices):
                raise ValueError('An obstacle coincides with the maps boundaries!');
            
            # make sure that the obstacle is a closed form
            if obstacle[0] != obstacle[-1]:
                obstacle.append(obstacle[0])
                self.obstacles_edges.append([LineString([Point(x[0],x[1]),Point(y[0],y[1])]) for (x,y) in zip(obstacle[:-1], obstacle[1:])])
            self.obstacles.append(Polygon(obstacle))

    def compute_distance(self, start_state, end_state):
        '''
        Return the Euclidean distance between two states.
        @param start_state The starting state (position) of the robot.
        @param end_state The target state (position) of the robot.
        '''
        return np.linalg.norm(np.array(end_state) - np.array(start_state))

    def state_validity_checker(self, state):
        '''
        Verify that the state is in the world boundaries, and is not inside an obstacle.
        Return false if the state is not applicable, and true otherwise.
        @param state The given position of the robot.
        '''

        # make sure robot state is a numpy array
        if not isinstance(state, np.ndarray):
            state = np.array(state)

        # verify that the robot position is between world boundaries
        if state[0] < self.xlimit[0] or state[1] < self.ylimit[0] or state[0] > self.xlimit[1] or state[1] > self.ylimit[1]:
            return False

        # verify that the robot is not positioned inside an obstacle
        for obstacle in self.obstacles:
            if obstacle.intersects(Point(state[0], state[1])):
                return False

        return True

    def edge_validity_checker(self, state1, state2):
        '''
        A function to check if the edge between two states is free from collisions. The function will return False if the edge intersects another obstacle.
        @param state1 The source state of the robot.
        @param state2 The destination state of the robot.
        '''

        # define undirected edge
        given_edge = LineString([state1, state2])

        # verify that the robot does not crossing any obstacle
        for obstacle in self.obstacles:
            if given_edge.intersects(obstacle):
                return False

        return True

    def compute_heuristic(self, state):
        '''
        Return the heuristic function for the A* algorithm.
        @param state The state (position) of the robot.
        '''

        # TODO: Task 4.3

        pass

    # ------------------------#
    # Visualization Functions
    # ------------------------#

    def visualize_map(self, show_map=False, plan=None, tree_edges=None, expanded_nodes=None):
        '''
        Visualize map with current state of robot and obstacles in the map.
        @param show_map If to show the map or save it.
        @param plan A given plan to draw for the robot.
        @param tree_edges A set of tree edges to draw.
        @param expanded_nodes A set of expanded nodes to draw.
        '''
        # create empty background
        plt = self.create_map_visualization()

        # add obstacles
        plt = self.visualize_obstacles(plt=plt)

        # add plan if given
        if plan is not None:
            plt = self.visualize_plan(plt=plt, plan=plan, color='navy')

        # add tree edges if given
        if tree_edges is not None:
            plt = self.visualize_tree_edges(plt=plt, tree_edges=tree_edges, color='lightgrey')

        # add expanded nodes if given
        if expanded_nodes is not None:
            plt = self.visualize_expanded_nodes(plt=plt, expanded_nodes=expanded_nodes, color='lightgrey')

        # add start
        plt = self.visualize_point_location(plt=plt, state=self.start, color='r')

        # add goal or inspection points
        plt = self.visualize_point_location(plt=plt, state=self.goal, color='g')

        # show map
        if show_map:
            plt.show()
        else:
            plt.savefig('map.png')

        return plt

    def create_map_visualization(self):
        '''
        Prepare the plot of the scene for visualization.
        '''
        # create figure and add background
        plt.figure()
        back_img = np.zeros((self.ylimit[1]+1, self.xlimit[1]+1))
        plt.imshow(back_img, origin='lower', zorder=0)

        return plt

    def visualize_obstacles(self, plt):
        '''
        Draw the scene's obstacles on top of the given frame.
        @param plt Plot of a frame of the environment.
        '''
        # plot obstacles
        for obstacle in self.obstacles:
            obstacle_xs, obstacle_ys = zip(*list(obstacle.exterior.coords))
            plt.fill(obstacle_xs, obstacle_ys, "y", zorder=5)

        return plt

    def visualize_plan(self, plt, plan, color):
        '''
        Draw a given plan on top of the given frame.
        @param plt Plot of a frame of the environment.
        @param plan The requested sequence of steps.
        @param color The requested color for the plan.
        '''
        # add plan edges to the plt
        for i in range(0, len(plan)-1):
            plt.plot([plan[i,0],plan[i+1,0]], [plan[i,1],plan[i+1,1]], color=color, linewidth=1, zorder=20)

        return plt 

    def visualize_tree_edges(self, plt, tree_edges, color):
        '''
        Draw the set of the given tree edges on top of the given frame.
        @param plt Plot of a frame of the environment.
        @param tree_edges The requested set of edges.
        @param color The requested color for the plan.
        '''
        # add plan edges to the plt
        for tree_edge in tree_edges:
            plt.plot([tree_edge[0][0],tree_edge[1][0]], [tree_edge[0][1],tree_edge[1][1]], color=color, zorder=10)

        return plt

    def visualize_expanded_nodes(self, plt, expanded_nodes, color):
        '''
        Draw the set of the given expanded nodes on top of the given frame.
        @param plt Plot of a frame of the environment.
        @param expanded_nodes The requested set of expanded states.
        @param color The requested color for the plan.
        '''
        # add plan edges to the plt
        point_radius = 0.5
        for expanded_node in expanded_nodes:
            point_circ = plt.Circle(expanded_node, radius=point_radius, color=color, zorder=10)
            plt.gca().add_patch(point_circ)

        return plt

    def visualize_point_location(self, plt, state, color):
        '''
        Draw a point of start/goal on top of the given frame.
        @param plt Plot of a frame of the environment.
        @param state The requested state.
        @param color The requested color for the point.
        '''

        # draw the circle
        point_radius = 0.5
        point_circ = plt.Circle(state, radius=point_radius, color=color, zorder=30)
        plt.gca().add_patch(point_circ)
    
        return plt