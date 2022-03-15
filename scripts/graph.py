from itertools import permutations
import math
import numpy as np

example_nodes = [(0,0),(3,0),(0,3),(3,3),(6,0), (0,6)]

class Graph:
        def __init__(self, nodes):
                # this makes an undirected graph, we could do a directed one too?
                # take in a list of nodes stored as tuples
                self.edges = {} # list of edges to be used in bfs
                self.n = 3  # grid size
             
                for node in nodes:
                        # for movement purposes, we want edges[node] = [left_node, right_node, up_node, down_node]
                        # edges[node][i] = -1 if movement in that direction is impossible

                        new = [-1]*4

                        for point in nodes:
                                if node[0] in point or node[1] in point:
                                        n = self.n # size of grid square
                                        if point[0] == node[0]-n and point[1] == node[1]:
                                                new[0] = point
                                        if point[0] == node[0] + n and point[1] == node[1]:
                                                new[1] = point
                                        if point[1] == node[1]+n and point[0] == node[0]:
                                                new[2] = point
                                        if point[1] == node[1]-n and point[0] == node[0]:
                                                new[3] = point
                        self.edges[node] = new

        def remove_edge(self, src, goal):
                # method to remove edge that is no longer navigable(i.e traffic cone blocking, etc)
                # remove src -> goal edge:
                direction = self.edges[src].index(goal)
                self.edges[src][direction] = -1

                direction = self.edges[goal].index(src)
                self.edges[goal][direction] = -1

        def is_equal(self,node1,node2):
                # check if two nodes are equal
                return node1[0] == node2[0] and node1[1] == node2[1] 

        def find_nearest_node_with_goal(self, src, goal):
            # for fractional source points, find the nearest intersection from node from pos to goal
            low = (self.n * math.floor(src[0]/3.0), self.n * math.floor(src[1]/3.0))
            high = (self.n * math.ceil(src[0]/3.0), self.n * math.ceil(src[1]/3.0))
            bounds = [low,high]

            if low not in self.edges:
                return high
            if high not in self.edges:
                return low

            dist_to_pos = [math.dist(low, src), math.dist(high,src)]
            dist_to_goal = [math.dist(low, goal), math.dist(high, goal)]
            dists = [dist_to_goal[0] + dist_to_pos[0], dist_to_goal[1] +dist_to_pos[1]]
            
            min_dist = min(dists)
            min_i = dists.index(min_dist)

            return bounds[min_i]

        def find_nearest_goal(self, src, goal):
        # for fractional goal points, find the closest intersection to navigate to first
            low = (self.n * math.floor(goal[0]/3.0), self.n * math.floor(goal[1]/3.0))
            high = (self.n * math.ceil(goal[0]/3.0), self.n * math.ceil(goal[1]/3.0))
            goals = [low,high]
         
            src = self.find_nearest_node_with_goal(src,goal)
            distances_goals = [math.dist(low, goal), math.dist(high,goal)]
            paths = [self.plan_path_in_edges(src,low), self.plan_path_in_edges(src,high)]

            if paths[0] == -1 or paths[1] == -1:
                return -1

            distances_paths = [self.n * len(paths[0]), self.n * len(paths[1])]

            distances = [distances_goals[0] + distances_paths[0], distances_goals[1] + distances_paths[1]]

            min_i = distances.index(min(distances))
            min_goal = goals[min_i]
            return min_goal

        def plan_path_in_edges(self, src, goal):
        # bfs search to find fastest path on unweighted, undirected graph
        # this function only works on src and goal in the set of edges
                explored = [] # array of explored nodes
                queue = [[src]] # array of paths

                if self.is_equal(src, goal):
                    return []
                
                while queue:
                        path = queue.pop(0) # queue stores the paths of each branch
                        node = path[-1] # last item of path

                        if node not in explored:
                                adjacent = [i for i in self.edges[node] if i != -1] # find neighbors
                                for nbr in adjacent:
                                        new_path = list(path)
                                        new_path.append(nbr)
                                        queue.append(new_path)

                                        if nbr == goal:
                                                return new_path
                                explored.append(node)
                return -1

        def plan_path_single(self, src, goal):
            # plan_path, but works with src and goal not in edges, i.e. betwen nodes
            whole_src = self.find_nearest_node_with_goal(src,goal)
            whole_goal = self.find_nearest_goal(src, goal)
            if whole_goal == -1 or whole_src == -1:
                return -1

            if math.dist(whole_src, goal) > math.dist(src, goal) or math.dist(src, whole_goal) > math.dist(src, goal):
                # check if the src and goal are between the same intersections
                return [goal]
            path = self.plan_path_in_edges(whole_src, whole_goal)
            if path == -1:
                return path

            if not self.is_equal(whole_goal, goal):
                if not path:
                    path += [whole_goal]
                else:
                    path[-1] = whole_goal
                path += [goal]
               
            return path
        
        def plan_mult_paths(self, src, goals):
            # goals is a list of goal points
            perms = permutations(goals)
            path = []
            length = math.inf 

            for perm in perms:
                cur_path = []
                prev = src
                for goal in perm:
                    if len(cur_path):    
                        cur_path.pop()
                    cur_path += self.plan_path_single(prev,goal)
                    prev = goal
                if len(cur_path) < length:
                    path = cur_path
                    length = len(cur_path)
            return path
        
        def plan_path(self, src, goal):
            if type(goal) is list:
                return self.plan_mult_paths(src, goal)
            else:
                return self.plan_path_single(src, goal)


