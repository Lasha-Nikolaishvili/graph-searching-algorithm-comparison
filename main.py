from collections import deque
import heapq
import time
import math
from data import graph, weighted_graph, coordinates
from utils import get_nodes, print_stats


def bfs(graph, start, goal):
    if start == goal:
        return [start], 1

    queue = deque([(start, [start])])
    visited = set()

    while queue:
        (node, path) = queue.popleft()
        if node in visited:
            continue

        visited.add(node)

        for neighbor in graph[node]:
            if neighbor == goal:
                return path + [neighbor], len(visited)
            else:
                queue.append((neighbor, path + [neighbor]))

    return None


def run_bfs():
    start_node, goal_node = get_nodes(graph)

    start_time = time.perf_counter()
    path, visited_count = bfs(graph, start_node, goal_node)
    end_time = time.perf_counter()

    print_stats(
        start_node, goal_node, path,
        start_time=start_time, end_time=end_time, visited_count=visited_count, time_comp='V + E', space_comp='V'
    )


def dfs_with_depth_limit(graph, node, goal, depth, path, visited, counter):
    counter[0] += 1

    if depth == 0 and node == goal:
        return path

    if depth > 0:
        for neighbor in graph[node]:
            if neighbor not in visited:
                visited.add(neighbor)
                result = dfs_with_depth_limit(graph, neighbor, goal, depth - 1, path + [neighbor], visited, counter)

                if result is not None:
                    return result

                visited.remove(neighbor)

    return None


def iddfs(graph, start, goal, max_depth):
    counter = [0]
    for depth in range(max_depth + 1):
        visited = {start}
        path = dfs_with_depth_limit(graph, start, goal, depth, [start], visited, counter)

        if path is not None:
            return path, counter[0]

    return None, counter[0]


def run_iddfs():
    start_node, goal_node = get_nodes(graph)
    max_depth = int(input('Max Depth (ex. 5): '))

    start_time = time.perf_counter()
    path, visited_count = iddfs(graph, start_node, goal_node, max_depth)
    end_time = time.perf_counter()

    print_stats(
        start_node, goal_node, path,
        start_time=start_time, end_time=end_time, visited_count=visited_count, time_comp='B ^ D', space_comp='B * D'
    )


def haversine(coord1, coord2):
    earth_radius = 6371
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    delta_lat = math.radians(lat2 - lat1)
    delta_lon = math.radians(lon2 - lon1)
    a = (
        math.sin(delta_lat / 2) * math.sin(delta_lat / 2) + math.cos(math.radians(lat1)) *
        math.cos(math.radians(lat2)) * math.sin(delta_lon / 2) * math.sin(delta_lon / 2)
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = earth_radius * c

    return distance


def a_star(weighted_graph, start, goal, coordinates):
    def heuristic(node1, node2):
        return haversine(coordinates[node1], coordinates[node2])

    open_list = []
    heapq.heappush(open_list, (0 + heuristic(start, goal), 0, start, [start]))
    closed_set = set()
    visited_count = 0

    while open_list:
        _, cost, current_node, path = heapq.heappop(open_list)

        if current_node in closed_set:
            continue

        visited_count += 1

        if current_node == goal:
            return path, cost, visited_count

        closed_set.add(current_node)

        for neighbor, weight in weighted_graph[current_node].items():
            if neighbor in closed_set:
                continue

            new_cost = cost + weight
            heapq.heappush(open_list, (new_cost + heuristic(neighbor, goal), new_cost, neighbor, path + [neighbor]))

    return None, float('inf'), visited_count


def run_a_star():
    start_node, goal_node = get_nodes(weighted_graph)

    start_time = time.perf_counter()
    path, cost, visited_count = a_star(weighted_graph, start_node, goal_node, coordinates)
    end_time = time.perf_counter()

    print_stats(
        start_node, goal_node, path,
        start_time=start_time, end_time=end_time, visited_count=visited_count, time_comp='B ^ D', space_comp='B ^ D',
        cost=cost
    )


def main():
    print('Choose an algorithm to run:')
    choice = input("BFS - 1, IDDFS - 2, A* - 3, '-1' to quit: ")

    while choice in ['1', '2', '3']:
        if choice == '1':
            run_bfs()
        elif choice == '2':
            run_iddfs()
        elif choice == '3':
            run_a_star()

        choice = input("BFS - 1, IDDFS - 2, A* - 3, any other character to quit: ")


if __name__ == '__main__':
    main()
