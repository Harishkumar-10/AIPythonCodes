import networkx as nx
import matplotlib.pyplot as plt
from collections import deque

def bfs(graph, start, goal):
    queue = deque([(start, [start])])
    
    while queue:
        node, path = queue.popleft()
        
        if node == goal:
            return path
        
        for neighbor in graph[node]:
            if neighbor not in path:
                queue.append((neighbor, path + [neighbor]))
    
    return None

# Example usage
graph = {
    'A': ['B', 'C'],
    'B': ['A', 'D', 'E'],
    'C': ['A', 'F'],
    'D': ['B'],
    'E': ['B', 'F'],
    'F': ['C', 'E']
}

start_node = 'A'
goal_node = 'F'

path = bfs(graph, start_node, goal_node)
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
plt.title('BFS Path')
plt.axis('off')

# Show the graph
plt.show()
