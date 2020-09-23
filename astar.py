"""Module to implement the A* algorithm using the abstract classes and generic graph search of search.py
Author: Clinton Walker"""

from search import *
import math
import heapq

class RoutingGraph(Graph):
    def __init__(self, map_str):
        
        # convert a map string to a 2d matrix of symbols
        self.map = [list(line) for line in map_str.strip().splitlines()]

        # initializes and populates lists for starting nodes and goal nodes   
        self.start = []
        self.goal = []
        self.read_map()
        

    def read_map(self):
        """iterates over a map adding start, goal and fuel nodes"""
        for row in range(len(self.map)):
            for column in range(len(self.map[row])):
                position = self.map[row][column]                
                if position == 'S':
                    self.start.append((row, column, math.inf))
                elif position.isdigit():
                    self.start.append((row, column, int(position)))
                elif position == 'G':
                    self.goal.append((row, column))
                
    def is_goal(self, node):
        """returns true if the given node is a goal node"""
        return any([goal == node[:2] for goal in self.goal])

    def starting_nodes(self):
        """Returns the starting node"""
        return self.start
    
    def outgoing_arcs(self, tail_node):
        """returns avalible actions for the given state (node)"""
        directions = [('N' , -1, 0),
                      ('E' ,  0, 1),
                      ('S' ,  1, 0),
                      ('W' ,  0, -1),]
        arcs = []
        row, column, fuel = tail_node

        # if out of fuel, either fuel up or halt.
        if fuel == 0:
            if self.map[row][column] != 'F':
                return arcs
            else:
                return [Arc(tail_node, (row, column, 9), "Fuel up", 15)]
            
        # Calculate row/coulumn/fuel for each direction and add it as an arc.
        for direction in directions:
            new_row, new_column, new_fuel = tail_node
            new_row += direction[1]
            new_column += direction[2]
            new_fuel -= 1
            if self.map[new_row][new_column] not in ['X', '|', '-']:
                arcs.append(Arc(tail_node, (new_row, new_column, new_fuel), direction[0], 5))
        
        # Add a fuel up arc if current node is a fuel up node.
        if self.map[row][column] == 'F' and fuel < 9:
            arcs.append(Arc(tail_node, (row, column, 9), "Fuel up", 15))

        return arcs
    
    def estimated_cost_to_goal(self, node):
        """Heuristic used for algorithm, uses manhattan distance * cost"""

        return 5 * min([manhattan_d(node, goal) for goal in self.goal])

class AStarFrontier(Frontier):

    def __init__ (self, graph):
        self.graph = graph
        self.container = []
        self.expanded = set()
        self.entry_count = 0

    def add(self, path):
        # Pruning (dont expand paths that have been expanded)
        if path[-1] in self.expanded:
            return
        
        cost = 0
        for arc in path:
            cost += arc.cost
        cost += self.graph.estimated_cost_to_goal(path[-1][1])

        # entry_count keeps the heap stable. Since it can only increment, paths with the same cost will work in a LIFO fashion.
        heapq.heappush(self.container, (cost, self.entry_count, path))
        self.entry_count += 1

    def __next__(self):
        if len(self.container) > 0:
            # Select a path that has not had its tail node expanded
            path = heapq.heappop(self.container)[2]
            while path[-1] in self.expanded:
                if len(self.container) > 0:
                    path = heapq.heappop(self.container)[2]
                else:
                    raise StopIteration
            # Mark the node about to be processed as expanded
            self.expanded.add(path[-1])
            return path
        else:
            raise StopIteration

        
def print_map(graph, frontier, solution):
    printed_map =graph.map
    
    # Mark solution on map
    if solution is not None:
        for arc in solution:
            row, column = arc.head[0], arc.head[1]
            if printed_map[row][column] == ' ':
                printed_map[row][column] = '*'
    
    # Mark expanded nodes on map
    for arc in frontier.expanded:
        row, column = arc.head[0], arc.head[1]
        if printed_map[row][column] == ' ':
            printed_map[row][column] = '.'
    
    # print each row of the map array as a string
    for row in printed_map:
        print("".join(row))



def manhattan_d(node1, node2):
    """calculates the manhattan disctance between two nodes"""
    return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])
    

