import networkx as nx
import matplotlib.pyplot as plt

def dfs(graph, node, goal, visited=None):
    if visited is None:
        visited = set()
    
    visited.add(node)
    
    if node == goal:
        return [node]
    
    for neighbor in graph[node]:
        if neighbor not in visited:
            path = dfs(graph, neighbor, goal, visited)
            if path:
                return [node] + path
    
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

path = dfs(graph, start_node, goal_node)
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
plt.title('DFS Path')
plt.axis('off')

# Show the graph
plt.show()
