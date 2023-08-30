"""
Author: Thomas Pardy
Date Modified: <2023-08-21 Mon>
Module containing graph related classes for mapping of arena.
"""
import math
import heapq


class Node:
    def __init__(self, name):
        self.name = name
        self.visited = False
        self.distance = float('inf')
        self.neighbours = []
        self.prev_node = None
        self.xy = [float('inf'), float('inf')]
        self.is_obstacle = False


    def add_neighbour(self, neighbour, weight):
        self.neighbours.append((neighbour, weight))


    def __lt__(self, other_node):
        return self.distance < other_node.distance





class Graph:
    """
    Contains graph module, with shortest path methods.
    """
    def __init__(self):
        self.nodes = {}


    def add_node(self, node: Node):
        self.nodes[node.name] = node

    def __getitem__(self, pos) -> Node:
        """get_node: Returns node given name of node"""
        i, j = pos
        return self.nodes[f'({i},{j})']

    def distance(self, pos1, pos2) -> float:
        """distance: Helper function, returns distance between two positions."""
        x1, y1 = pos1
        x2, y2 = pos2
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def get_nearest_node(self, pos) -> Node:
        """get_nearest_node: Takes in a (x,y) position, returns nearest node on map"""
        min_dist = float('inf')
        min_node = None
        x1, y1 = pos
        for node_name in self.nodes:
            node = self.nodes[node_name]
            dist = self.distance((x1,y1), node.xy)
            if dist < min_dist and node.is_obstacle == False:
                min_dist = dist
                min_node = node

        return min_node

    def adjacent_nodes(self, node, radius) -> list:
        """adjacent_nodes:  returns surrounding nodes within a radius"""

        def recursive_nodes(node, radius, pos, memo) -> None:
            for neighbour,_ in node.neighbours:
                memo.append(node)
                if self.distance(pos, neighbour.xy) < radius and neighbour not in memo:
                    recursive_nodes(neighbour, radius, pos, memo)
                else:
                    return

        memo = []
        recursive_nodes(node, radius, node.xy, memo)
        return memo

    def set_obstacle(self, node) -> None:
        """set_obstacle: Given a node sets it as an obstacle in the graph"""
        node.is_obstacle = True
        for neighbour, _ in node.neighbours:
            neighbour.neighbours = [x for x in neighbour.neighbours if x[0] != node]
        node.neighbours = []

    

    def get_adjacent_nodes(self, xy_position: tuple, radius: float) -> list:
        """get_adjacent_nodes: returns nodes around a given position within radius."""
        pass

    def remove_nodes(self, node_list: list) -> None:
        """remove_nodes: given list of nodes, removes all nodes from graph"""
        pass




    def djikstras(self, start_node, target_node):
        start_node.distance = 0
        heap = [(0, start_node)]


        while heap:
            current_distance, current_node = heapq.heappop(heap)

            if current_node == target_node:
                break

            if current_node.visited:
                continue

            current_node.visited = True

            for neighbour,  weight in current_node.neighbours:
                if not neighbour.visited:
                    new_distance = current_distance + weight
                    if new_distance < neighbour.distance:
                        neighbour.distance = new_distance
                        neighbour.prev_node = current_node
                        heapq.heappush(heap, (new_distance, neighbour))



    def get_shortest_distance(self, target: Node):
        path = []
        current_node = target

        while current_node.prev_node is not None:
            path.insert(0,current_node.name)
            current_node = current_node.prev_node
        path.insert(0,current_node.name)
        return target.distance, path



if __name__ == '__main__':
    nodes = [[Node(f"({row},{col})") for col in range(3)] for row in range(3)]

    for row in range(3):
        for col in range(3):
            if row > 0:
                nodes[row][col].add_neighbour(nodes[row - 1][col], 1) # Up

            if row < 2:
                nodes[row][col].add_neighbour(nodes[row + 1][col], 1) # Down

            if col > 0:
                nodes[row][col].add_neighbour(nodes[row][col - 1], 1) # Left

            if col < 2:
                nodes[row][col].add_neighbour(nodes[row][col + 1], 1) # Right


    G = Graph()
    for row in nodes:
        for node_i in row:
            G.add_node(node_i)

    start_node = nodes[1][1]
    target_node = nodes[2][2]
    #G.djikstras(start_node, target_node)
    #path_test, dist = G.get_shortest_distance(target_node)
    #print(path_test, dist)
    print(G[0, 0])
