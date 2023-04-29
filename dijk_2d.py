import heapq

def dijkstra(grid, start, end):
    rows, cols = len(grid), len(grid[0])
    dist = [[float('inf') for _ in range(cols)] for _ in range(rows)]
    dist[start[0]][start[1]] = 0
    heap = [(0, start)]
    visited = set()
    prev = {}

    while heap:
        (d, node) = heapq.heappop(heap)
        if node == end:
            return dist[end[0]][end[1]], reconstruct_path(start, end, prev)
        if node in visited:
            continue
        visited.add(node)

        for neighbor in get_neighbors(grid, node):
            new_cost = dist[node[0]][node[1]] + get_weight(node, neighbor)
            if new_cost < dist[neighbor[0]][neighbor[1]]:
                dist[neighbor[0]][neighbor[1]] = new_cost
                prev[neighbor] = node
                heapq.heappush(heap, (new_cost, neighbor))

    return -1, []

def get_neighbors(grid, node):
    i, j = node
    neighbors = []
    for di in [-1, 0, 1]:
        for dj in [-1, 0, 1]:
            if di == 0 and dj == 0:
                continue
            ni, nj = i + di, j + dj
            if 0 <= ni < len(grid) and 0 <= nj < len(grid[0]) and grid[ni][nj] != 1:
                if di == 0 or dj == 0:
                    # straight move
                    neighbors.append((ni, nj))
                else:
                    # diagonal move
                    if grid[i][nj] != 1 and grid[ni][j] != 1:
                        neighbors.append((ni, nj))
    return neighbors

def get_weight(node1, node2):
    i1, j1 = node1
    i2, j2 = node2
    if i1 != i2 and j1 != j2:
        return 1.4  # diagonal move
    else:
        return 1.0  # straight move

def reconstruct_path(start, end, prev):
    path = [end]
    while path[-1] != start:
        path.append(prev[path[-1]])
    return path[::-1]

# Example usage:
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 1, 1, 0],
    [0, 0, 0, 0, 0]
]
start = (0, 0)
end = (4, 4)
distance, path = dijkstra(grid, start, end)
print("Shortest distance:", distance)
print("Shortest path:", path)
