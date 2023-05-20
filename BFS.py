from collections import defaultdict
import pydot

class Graph:
    def __init__(self):
        self.graph=defaultdict(list)
    
    def addEdge(self,u,v):
        self.graph[u].append(v)
    
    def BFS(self,gs):
        visited=[False]*(max(self.graph)+1)
        queue=[]
        queue.append(gs)
        visited[gs]=True
        while queue:
            gs=queue.pop(0)
            print(gs,end="")
            
            for i in self.graph[gs]:
                if visited[i]==False:
                    queue.append(i)
                    visited[i]=True
                    
vis_Graph=pydot.Dot(graph_type='digraph')
node0=pydot.Node("0")
vis_Graph.add_node(node0)
node1=pydot.Node("1")
vis_Graph.add_node(node1)
node2=pydot.Node("2")
vis_Graph.add_node(node2)
node3=pydot.Node("3")
vis_Graph.add_node(node3)
                    
g = Graph()
            
g.addEdge(0,1)
vis_Graph.add_edge(pydot.Edge(node0,node1))
g.addEdge(0,3)
vis_Graph.add_edge(pydot.Edge(node0,node3))
g.addEdge(1,2)
            
vis_Graph.add_edge(pydot.Edge(node1,node2))
g.addEdge(1,3)
            
vis_Graph.add_edge(pydot.Edge(node1,node3))
g.addEdge(2,1)
            
vis_Graph.add_edge(pydot.Edge(node2,node1))
g.addEdge(2,3)
vis_Graph.add_edge(pydot.Edge(node2,node3))
g.addEdge(3,3)
vis_Graph.add_edge(pydot.Edge(node3,node3))
            
vis_Graph.write_png(r'C:\Users\USER\Desktop\MCA\graph.png')
print("Breathfirsttraversalis:""(starting from vertex 0)")

g.BFS(0)    
