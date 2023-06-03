import heapq
import networkx as nx
import matplotlib.pyplot as plt

# Define the graph as a dictionary of nodes and their neighbors with associated costs
graph = {
    'A': [('B', 5), ('C', 3)],
    'B': [('D', 2), ('E', 4)],
    'C': [('F', 6)],
    'D': [('G', 8)],
    'E': [('F', 4), ('H', 3)],
    'F': [('G', 2), ('H', 6)],
    'G': [('H', 1)],
    'H': []
}

def heuristic(node, goal):
    # Heuristic function that estimates the cost from the current node to the goal
    h_values = {
        'A': 10,
        'B': 8,
        'C': 7,
        'D': 6,
        'E': 4,
        'F': 3,
        'G': 2,
        'H': 0
    }
    return h_values[node]

def astar_search(graph, start, goal):
    # A* search algorithm
    open_list = [(0, start)]  # Priority queue of nodes to visit, initially containing the start node
    came_from = {}  # Dictionary to store the path
    g_scores = {node: float('inf') for node in graph}  # Cost from the start node to each node
    g_scores[start] = 0

    while open_list:
        current_cost, current_node = heapq.heappop(open_list)  # Get the node with the lowest total cost
        if current_node == goal:
            break

        for neighbor, cost in graph[current_node]:
            new_cost = g_scores[current_node] + cost
            if new_cost < g_scores[neighbor]:
                g_scores[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(open_list, (priority, neighbor))
                came_from[neighbor] = current_node

    # Reconstruct the path from start to goal
    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()

    # Calculate the cost of the path
    cost = sum(graph[node][graph[node].index((path[i], path[i+1]))][1] for i, node in enumerate(path[:-1]))

    return path, cost

# Visualize the graph
G = nx.Graph()
for node in graph:
    G.add_node(node)
    for neighbor, cost in graph[node]:
        G.add_edge(node, neighbor, weight=cost)

pos = nx.spring_layout(G)
nx.draw_networkx(G, pos, with_labels=True, node_size=500, font_size=12, node_color='lightblue')
labels = nx.get_edge_attributes(G, 'weight')
nx.draw_networkx_edge_labels(G, pos, edge_labels=labels)
plt.title("Graph Visualization")
plt.axis('off')

# Save the graph visualization as an image
plt.savefig('graph_visualization.png')

# Test the A* search algorithm
start_node = 'A'
goal_node = 'H'
path, cost = astar_search(graph, start_node, goal_node)
print(f"Shortest path from {start_node} to {goal_node}: {' -> '.join(path)}")
print(f"Cost of the path: {cost}")
