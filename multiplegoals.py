from heapq import heappop, heappush
import numpy as np
def a_star_multi(start, goals, grid):
    # Define heuristic function (Euclidean distance)
    def h(node):
        return min([((node[0] - goal[0])**2 + (node[1] - goal[1])**2)**0.5 for goal in goals])
 
    # Define movement costs
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    costs = [1.0, 1.0, 1.0, 1.0, 1.4, 1.4, 1.4, 1.4]
 
    # Initialize data structures
    frontier = []
    came_from = {}
    g_score = {start: 0.0}
    f_score = {start: h(start)}
    heappush(frontier, (f_score[start], start))
    # Main loop
    while frontier:
        current = heappop(frontier)[1]
        # Check if current node is a goal node
        if current in goals:
            return current, came_from
        # Calculate cost of moving to neighbors
        for i, neighbor in enumerate(neighbors):
            next_node = (current[0]+neighbor[0], current[1]+neighbor[1])
            if next_node[0] < 0 or next_node[0] >= len(grid) or next_node[1] < 0 or next_node[1] >= len(grid[0]):
                continue
            if grid[next_node[0]][next_node[1]] == 1:
                continue
            new_g_score = g_score[current] + costs[i]
            if next_node not in g_score or new_g_score < g_score[next_node]:
                g_score[next_node] = new_g_score
                f_score[next_node] = new_g_score + h(next_node)
                came_from[next_node] = current
                heappush(frontier, (f_score[next_node], next_node))
    # If there is no path to any goal node
    return None, None
 
 
# Define the grid with obstacles
grid = np.zeros((5, 5))
grid[1, 1:4] = 1
grid[2, 1:3] = 1
grid[3, 3] = 1
 
# Define the start and goal positions
start = (0, 0)
goals = [(4, 4), (4, 0), (2, 2)]
 
# Find the shortest path using A* to any of the goals
goal, path = a_star_multi(start, goals, grid)
 
# Print the result
if path is None:
    print("No path found to any goal")
else:
    print("Shortest path to goal", goal, ":", path)