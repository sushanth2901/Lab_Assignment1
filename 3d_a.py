import numpy as np
from queue import PriorityQueue

# Define a 3D numpy array to represent the grid
grid = np.array([
    [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
    [[0, 0, 0], [1, 1, 0], [0, 0, 0]],
    [[0, 0, 0], [1, 1, 0], [0, 0, 0]],
    [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
])

# Define the starting and goal points
start = (0, 0, 0)
goal = (3, 2, 0)

# Define a function to calculate the Euclidean distance between two points
def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2)

# Define the A* algorithm
def astar(grid, start, goal):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: distance(start, goal)}

    while not open_set.empty():
        current = open_set.get()[1]
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for neighbor in [(current[0]-1, current[1], current[2]), (current[0]+1, current[1], current[2]), (current[0], current[1]-1, current[2]), (current[0], current[1]+1, current[2]), (current[0], current[1], current[2]-1), (current[0], current[1], current[2]+1)]:
            if neighbor[0] < 0 or neighbor[0] >= grid.shape[0] or neighbor[1] < 0 or neighbor[1] >= grid.shape[1] or neighbor[2] < 0 or neighbor[2] >= grid.shape[2]:
                continue
            if grid[neighbor] == 1:
                continue
            tentative_g_score = g_score[current] + distance(current, neighbor)
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + distance(neighbor, goal)
                open_set.put((f_score[neighbor], neighbor))

    return None

# Find the shortest path using A*
path = astar(grid, start, goal)
print(path)
