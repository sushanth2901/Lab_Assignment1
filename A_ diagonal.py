import heapq
import math

# Define the grid and obstacles.
grid = [[0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0, 0]]

# Define the start and end points.
start = (0, 0)
end = (4, 5)

# Define the heuristic function.
def heuristic(a, b):
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

# Define A* algorithm.
def astar(start, end, grid):
    pq = []
    heapq.heappush(pq, (0, start))
    visited = {start: (0, None)}
    while pq:
        current_cost, current = heapq.heappop(pq)
        if current == end:
            path = []
            while current != start:
                path.append(current)
                current = visited[current][1]
            path.append(start)
            return list(reversed(path))
        for next in get_neighbors(current, grid):
            new_cost = current_cost + 1
            if next not in visited or new_cost < visited[next][0]:
                heapq.heappush(pq, (new_cost + heuristic(end, next), next))
                visited[next] = (new_cost, current)
    return []

# Define the helper function to get the neighbors of a node.
def get_neighbors(node, grid):
    neighbors = [(node[0]-1, node[1]-1),
                 (node[0]-1, node[1]),
                 (node[0]-1, node[1]+1),
                 (node[0], node[1]-1),
                 (node[0], node[1]+1),
                 (node[0]+1, node[1]-1),
                 (node[0]+1, node[1]),
                 (node[0]+1, node[1]+1)]
    valid_neighbors = []
    for neighbor in neighbors:
        i, j = neighbor
        if 0 <= i < len(grid) and 0 <= j < len(grid[0]) and not grid[i][j]:
            valid_neighbors.append(neighbor)
    return valid_neighbors

# Find the shortest path using A* algorithm.
path = astar(start, end, grid)

# Print the resulting shortest path.
print("Shortest path:", path)
