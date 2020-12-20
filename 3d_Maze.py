from collections import deque
from queue import PriorityQueue
import math
import time


def encode_instructions(location, code):
    dest = list(location)
    if code == 1:
        dest[0] += 1
    elif code == 2:
        dest[0] -= 1
    elif code == 3:
        dest[1] += 1
    elif code == 4:
        dest[1] -= 1
    elif code == 5:
        dest[2] += 1
    elif code == 6:
        dest[2] -= 1
    elif code == 7:
        dest[0] += 1
        dest[1] += 1
    elif code == 8:
        dest[0] += 1
        dest[1] -= 1
    elif code == 9:
        dest[0] -= 1
        dest[1] += 1
    elif code == 10:
        dest[0] -= 1
        dest[1] -= 1
    elif code == 11:
        dest[0] += 1
        dest[2] += 1
    elif code == 12:
        dest[0] += 1
        dest[2] -= 1
    elif code == 13:
        dest[0] -= 1
        dest[2] += 1
    elif code == 14:
        dest[0] -= 1
        dest[2] -= 1
    elif code == 15:
        dest[1] += 1
        dest[2] += 1
    elif code == 16:
        dest[1] += 1
        dest[2] -= 1
    elif code == 17:
        dest[1] -= 1
        dest[2] += 1
    elif code == 18:
        dest[1] -= 1
        dest[2] -= 1

    return tuple(dest)


def check_out_of_grid(location, maze_size):
    if location < maze_size:
        return True
    else:
        return False


def check_if_goal_exist(graph, goal):
    if graph.if_node_exist(goal):
        return True
    else:
        f = open("output.txt", "w")
        f.write("FAIL")
        return False


def bfs_search(bfs_graph, start_location, goal_location):
    if start_location == goal_location:
        bfs_output([], start_location, start_location)
        return
    frontier = deque([start_location])
    visited = {start_location: start_location}
    while frontier:
        currentNode = frontier.popleft()
        for neighbor in bfs_graph.get_node(currentNode).get_adjacents():
            if neighbor == goal_location:
                visited[neighbor] = currentNode
                bfs_output(visited, start_location, neighbor)
                return
            if neighbor not in visited:
                visited[neighbor] = currentNode
                frontier.append(neighbor)
    f = open("output.txt", "w")
    f.write("FAIL")


def bfs_output(visited, start_location, cursor):
    f = open("output.txt", "w")
    value_of_steps = 0
    num_of_steps = 1
    cost = 0
    path_list = list()
    path_list.append(cursor)
    while cursor != start_location:
        cursor = visited[cursor]
        value_of_steps += 1
        num_of_steps += 1
        path_list.append(cursor)
    f.write(str(value_of_steps) + '\n')
    f.write(str(num_of_steps) + '\n')
    while path_list:
        outcome = path_list.pop()
        if outcome != start_location:
            cost = 1
        fixed_outcome = "{x} {y} {z} {cost}".format(x=outcome[0], y=outcome[1], z=outcome[2], cost=cost)
        f.write(fixed_outcome + '\n')


def ucs_search(ucs_graph, start_location, goal_location):
    current_location = start_location
    frontier = {start_location: 0}
    visited = {start_location: start_location}
    while frontier:
        cost = frontier.pop(current_location)
        if current_location == goal_location:
            ucs_output(visited, start_location, current_location, cost, ucs_graph)
            return
        else:
            for neighbor in ucs_graph.get_node(current_location).get_adjacents():
                total_cost = cost + ucs_graph.get_node(current_location).get_weight(neighbor)
                if neighbor not in visited:
                    visited[neighbor] = current_location
                    frontier[neighbor] = total_cost
                else:
                    previous_cost = ucs_graph.get_node(visited[neighbor]).get_weight(neighbor)
                    if total_cost < previous_cost:
                        visited[neighbor] = current_location
                        frontier[neighbor] = total_cost
        if frontier:
            min_cost_in_frontier = min(frontier.values())
            for location, frontier_cost in frontier.items():
                if frontier_cost == min_cost_in_frontier:
                    current_location = location
                    break
        else:
            break

    f = open("output.txt", "w")
    f.write("FAIL")


def ucs_output(parent, start_location, cursor, total_cost, ucs_graph):
    f = open("output.txt", "w")
    value_of_steps = total_cost
    num_of_steps = 1
    previous_location = start_location
    path_list = list()
    path_list.append(cursor)
    while cursor != start_location:
        cursor = parent[cursor]
        num_of_steps += 1
        path_list.append(cursor)
    f.write(str(value_of_steps) + '\n')
    f.write(str(num_of_steps) + '\n')
    while path_list:
        outcome = path_list.pop()
        if outcome != start_location:
            cost = ucs_graph.get_node(previous_location).get_weight(outcome)
        else:
            cost = 0
        previous_location = outcome
        fixed_outcome = "{x} {y} {z} {cost}".format(x=outcome[0], y=outcome[1], z=outcome[2], cost=cost)
        f.write(fixed_outcome + '\n')


def calc_heuristic_value(end_location, current_location):
    ex, ey, ez = end_location[0], end_location[1], end_location[2]
    cx, cy, cz = current_location[0], current_location[1], current_location[2]
    hv = 10 * math.sqrt(pow((ex - cx), 2) + pow((ey - cy), 2) + pow((ez - cz), 2))
    return int(hv)


def aStar_search(aStar_graph, start_location, goal_location):
    current_location = start_location
    start_heuristic_val = aStar_graph.get_node(start_location).get_heuristic_value()
    frontier = {start_location: start_heuristic_val}
    visited = {start_location: start_location}
    cost_menu = {start_location: 0}
    while frontier:
        frontier.pop(current_location)
        cost = cost_menu[current_location]
        if current_location == goal_location:
            aStar_output(visited, start_location, current_location, cost, aStar_graph)
            return
        else:
            for neighbor in aStar_graph.get_node(current_location).get_adjacents():
                total_cost = cost + aStar_graph.get_node(current_location).get_weight(neighbor) + aStar_graph.get_node(
                    neighbor).get_heuristic_value()
                neighbor_cost = cost + aStar_graph.get_node(current_location).get_weight(neighbor)
                if neighbor not in visited:
                    visited[neighbor] = current_location
                    frontier[neighbor] = total_cost
                    cost_menu[neighbor] = neighbor_cost
                else:
                    previous_cost_w_hv = aStar_graph.get_node(visited[neighbor]).get_weight(neighbor)\
                                         + aStar_graph.get_node(neighbor).get_heuristic_value()
                    if total_cost < previous_cost_w_hv:
                        visited[neighbor] = current_location
                        frontier[neighbor] = total_cost
                        cost_menu[neighbor] = neighbor_cost
        min_cost_in_frontier = min(frontier.values())
        for location, frontier_cost in frontier.items():
            if frontier_cost == min_cost_in_frontier:
                current_location = location
                break
    f = open("output.txt", "w")
    f.write("FAIL")


def aStar_output(parent, start_location, cursor, total_cost, aStar_graph):
    f = open("output.txt", "w")
    value_of_steps = total_cost
    num_of_steps = 1
    previous_location = start_location
    path_list = list()
    path_list.append(cursor)
    while cursor != start_location:
        cursor = parent[cursor]
        num_of_steps += 1
        path_list.append(cursor)
    f.write(str(value_of_steps) + '\n')
    f.write(str(num_of_steps) + '\n')
    while path_list:
        outcome = path_list.pop()
        if outcome != start_location:
            cost = aStar_graph.get_node(previous_location).get_weight(outcome)
        else:
            cost = 0
        previous_location = outcome
        fixed_outcome = "{x} {y} {z} {cost}".format(x=outcome[0], y=outcome[1], z=outcome[2], cost=cost)
        f.write(fixed_outcome + '\n')


class Node:
    def __init__(self, x, y, z, hv=0):
        self.x = x
        self.y = y
        self.z = z
        self.heuristic_value = hv
        self.location = (x, y, z)
        self.adjacents = {}

    def get_location(self):
        return self.location

    def add_adjacent(self, location, weight):
        self.adjacents[location] = weight

    def get_adjacents(self):
        return self.adjacents.keys()

    def get_weight(self, location):
        if location in self.adjacents.keys():
            return self.adjacents[location]
        else:
            return 0

    def get_heuristic_value(self):
        return self.heuristic_value

    def if_exist_path_with(self, location):
        return location in self.adjacents


class Graph:
    def __init__(self):
        self.nodes = {}

    def add_node(self, x, y, z, heuristic_value=0):
        node = Node(x, y, z, heuristic_value)
        self.nodes[(x, y, z)] = node

    def get_node(self, location):
        return self.nodes[location]

    def add_edge(self, current_location, next_location, weight=1):
        self.nodes[current_location].add_adjacent(next_location, weight)

    def if_node_exist(self, location):
        return location in self.nodes


def main():
    f = open("input.txt", "r")
    lines = f.readlines()
    method = lines[0].rstrip('\n')
    maze_size = (int(lines[1].split()[0]), int(lines[1].split()[1]), int(lines[1].split()[2]))
    start_location = (int(lines[2].split()[0]), int(lines[2].split()[1]), int(lines[2].split()[2]))
    end_location = (int(lines[3].split()[0]), int(lines[3].split()[1]), int(lines[3].split()[2]))
    num_of_nodes = int(lines[4])
    node_list = list()
    for line in lines[5:5 + num_of_nodes]:
        line = line.rstrip('\n')
        line = line.split()
        location = (int(line[0]), int(line[1]), int(line[2]))
        if check_out_of_grid(location, maze_size):
            path_list = list()
            for path in line[3:]:
                path_list.append(int(path))
            node_list.append((location, path_list))

    if method == "BFS":
        bfs_graph = Graph()
        for node in node_list:
            if not bfs_graph.if_node_exist(node[0]):
                bfs_graph.add_node(node[0][0], node[0][1], node[0][2])
            for code in node[1]:
                adjacent_node_location = encode_instructions(node[0], code)
                bfs_graph.add_edge(node[0], adjacent_node_location, 1)
                if not bfs_graph.if_node_exist(adjacent_node_location):
                    bfs_graph.add_node(adjacent_node_location[0], adjacent_node_location[1], adjacent_node_location[2])
        if check_if_goal_exist(bfs_graph, end_location):
            bfs_search(bfs_graph, start_location, end_location)
    elif method == "UCS":
        ucs_graph = Graph()
        for node in node_list:
            if not ucs_graph.if_node_exist(node[0]):
                ucs_graph.add_node(node[0][0], node[0][1], node[0][2])
            for code in node[1]:
                adjacent_node_location = encode_instructions(node[0], code)
                edge_value = 0
                if 0 < code <= 6:
                    edge_value = 10
                elif 7 <= code < 19:
                    edge_value = 14
                ucs_graph.add_edge(node[0], adjacent_node_location, edge_value)
                if not ucs_graph.if_node_exist(adjacent_node_location):
                    ucs_graph.add_node(adjacent_node_location[0], adjacent_node_location[1], adjacent_node_location[2])
        if check_if_goal_exist(ucs_graph, end_location):
            ucs_search(ucs_graph, start_location, end_location)
    elif method == "A*":
        aStar_graph = Graph()
        for node in node_list:
            if not aStar_graph.if_node_exist(node[0]):
                heuristic_value = calc_heuristic_value(end_location, node[0])
                aStar_graph.add_node(node[0][0], node[0][1], node[0][2], heuristic_value)
            for code in node[1]:
                adjacent_node_location = encode_instructions(node[0], code)
                edge_value = 0
                if 0 < code <= 6:
                    edge_value = 10
                elif 7 <= code < 19:
                    edge_value = 14
                aStar_graph.add_edge(node[0], adjacent_node_location, edge_value)
                if not aStar_graph.if_node_exist(adjacent_node_location):
                    heuristic_value = calc_heuristic_value(end_location, adjacent_node_location)
                    aStar_graph.add_node(adjacent_node_location[0], adjacent_node_location[1],
                                         adjacent_node_location[2], heuristic_value)
        if check_if_goal_exist(aStar_graph, end_location):
            aStar_search(aStar_graph, start_location, end_location)


if __name__ == '__main__':
    start_time = time.time()
    main()
    print("--- %s seconds ---" % (time.time() - start_time))
