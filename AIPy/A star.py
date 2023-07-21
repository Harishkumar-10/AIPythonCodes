import networkx as nx
import matplotlib.pyplot as plt
import heapq

def heuristic(node, goal):
    # Calculate the heuristic value (e.g., Manhattan distance)
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

def astar(graph, start, goal):
    queue = []
    heapq.heappush(queue, (0, start, [start]))
    visited = set(start)
    
    while queue:
        _, node, path = heapq.heappop(queue)
        
        if node == goal:
            return path
        
        for neighbor in graph[node]:
            if neighbor not in visited:
                priority = len(path) + heuristic(neighbor, goal)
                heapq.heappush(queue, (priority, neighbor, path + [neighbor]))
                visited.add(neighbor)
    
    return None

# Example usage
graph = {
    (0, 0): [(0, 1), (1, 0)],
    (0, 1): [(0, 0), (0, 2), (1, 1)],
    (0, 2): [(0, 1), (1, 2)],
    (1, 0): [(0, 0), (1, 1), (2, 0)],
    (1, 1): [(0, 1), (1, 0), (1, 2), (2, 1)],
    (1, 2): [(0, 2), (1, 1), (2, 2)],
    (2, 0): [(1, 0), (2, 1)],
    (2, 1): [(1, 1), (2, 0), (2, 2)],
    (2, 2): [(1, 2), (2, 1)]
}

start_node = (0, 0)
goal_node = (2, 2)

path = astar(graph, start_node, goal_node)
print("Path:", path)

# Create a graph visualization
G = nx.Graph(graph)
pos = nx.spring_layout(G)

# Draw nodes
nx.draw_networkx_nodes(G, pos, node_color='lightblue', node_size=500)

# Draw edges
nx.draw_networkx_edges(G, pos, edge_color='gray')

# Highlight the path
path_edges = [(path[i], path[i+1]) for i in range(len(path)-1)]
nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color='red', width=2)

# Draw labels
nx.draw_networkx_labels(G, pos, font_color='black', font_size=12)

# Set plot options
plt.title('A* Path')
plt.axis('off')

# Show the graph
plt.show()
