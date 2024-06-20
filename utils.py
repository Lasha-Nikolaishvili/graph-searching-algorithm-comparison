def get_nodes(graph):
    start_node = input('Start Node (ex. Arad): ')
    goal_node = input('Goal Node (ex. Bucharest): ')

    while start_node not in graph or goal_node not in graph:
        if start_node not in graph:
            print('Start node is not in the graph. Try again!')
            start_node = input('Start Node (ex. Arad): ')
        if goal_node not in graph:
            print('Goal node is not in the graph. Try again!')
            goal_node = input('Goal Node (ex. Bucharest): ')

    return start_node, goal_node


def print_stats(start_node, goal_node, path, *args, **kwargs):
    print(f"The shortest path from {start_node} to {goal_node} is: {path}")
    print(f"Finished in {kwargs['end_time'] - kwargs['start_time']} second(s).")
    print(f"Visited {kwargs['visited_count']} nodes in total.")
    print(f"Time complexity is O({kwargs['time_comp']})")
    print(f"Space complexity is O({kwargs['space_comp']})")
    if 'cost' in kwargs:
        print(f"Cost of the path is {kwargs['cost']}km.")
