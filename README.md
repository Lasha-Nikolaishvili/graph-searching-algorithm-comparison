# Romania Map Pathfinding Algorithms

This project implements various pathfinding algorithms on a graph representing cities and connections in Romania. The algorithms included are Breadth-First Search (BFS), Iterative Deepening Depth First Search (IDDFS), and A* search algorithm. Each algorithm is used to find the shortest path between two given cities.

## Table of Contents
1. [Project Structure](#project-structure)
2. [Dependencies](#dependencies)
3. [Usage](#usage)
4. [Algorithms](#algorithms)
5. [Data](#data)
6. [Utilities](#utilities)

## Project Structure
```
graph-searching-algorithm-comparison/
├── main.py
├── data.py
└── utils.py
```

- `main.py`: Contains the main logic for running the different algorithms.
- `data.py`: Contains the graph representations and city coordinates.
- `utils.py`: Contains utility functions for node input and printing statistics.

## Dependencies
- Python 3.x
- No additional libraries are required.

## Usage
Run the `main.py` script to start the program:
```sh
python main.py
```
You will be prompted to choose an algorithm to run:
- `1` for Breadth-First Search (BFS)
- `2` for Iterative Deepening Depth First Search (IDDFS)
- `3` for A* Search
- `Any other character to quit` to quit

Follow the on-screen prompts to input the start and goal cities, and in the case of IDDFS, the maximum depth.

## Algorithms

### Breadth-First Search (BFS)
BFS explores all nodes at the present depth level before moving on to nodes at the next depth level.

**Time Complexity**: O(V + E)

**Space Complexity**: O(V)

### Iterative Deepening Depth First Search (IDDFS)
IDDFS combines the depth-first search's space efficiency and breadth-first search's completeness. It iteratively deepens the search depth limit until the goal is found.

**Time Complexity**: O(b^d)

**Space Complexity**: O(b * d)

### A* Search Algorithm
A* uses both actual cost from the start node and a heuristic estimate to the goal node to determine the order in which nodes are explored.

**Time Complexity**: O(b^d)

**Space Complexity**: O(b^d)


## Data
### Graph Representations
- `graph`: An unweighted graph for BFS and IDDFS.
- `weighted_graph`: A weighted graph for A* search.

**Example:**
```python
graph = {
    'Arad': ['Zerind', 'Timisoara', 'Sibiu'],
    # ... (other cities and connections)
}

weighted_graph = {
    'Arad': {'Zerind': 75, 'Timisoara': 118, 'Sibiu': 140},
    # ... (other cities and weights)
}

coordinates = {
    'Arad': (46.175101, 21.319541),
    # ... (other cities and coordinates)
}
```

## Utilities
### Node Input
`get_nodes(graph)`: Prompts the user for start and goal nodes, ensuring they are valid.

### Statistics Printing
`print_stats(start_node, goal_node, path, **kwargs)`: Prints the path, time taken, nodes visited, and complexities.


---

By running this project, you can explore different pathfinding algorithms and observe their performance on a graph representing Romania's cities.