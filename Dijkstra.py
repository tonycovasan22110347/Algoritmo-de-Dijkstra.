import heapq
import time
import matplotlib.pyplot as plt
import networkx as nx
import os

def draw_graph(graph, pos, shortest_paths, current_node=None):
    G = nx.DiGraph()
    
    for node, edges in graph.items():
        for neighbor, weight in edges.items():
            G.add_edge(node, neighbor, weight=weight)
    
    plt.clf()
    
    nx.draw(G, pos, with_labels=True, node_size=700, node_color='lightblue')
    
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    
    for node, dist in shortest_paths.items():
        if dist < float('inf'):
            nx.draw_networkx_nodes(G, pos, nodelist=[node], node_color='yellow')

    if current_node:
        nx.draw_networkx_nodes(G, pos, nodelist=[current_node], node_color='red')
    
    plt.pause(0.5)

def dijkstra_simulation(graph, start):
    distances = {node: float('inf') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    visited = set()
    
    pos = nx.spring_layout(nx.DiGraph(graph))
    
    step = 1
    while priority_queue:
        print(f"\n--- Paso {step} ---")
        current_distance, current_node = heapq.heappop(priority_queue)
        
        if current_node in visited:
            print(f"Nodo '{current_node}' ya visitado. Saltando.")
            continue
        
        print(f"Procesando nodo '{current_node}' con distancia acumulada {current_distance}.")
        
        draw_graph(graph, pos, distances, current_node)
        
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
                print(f"Actualizando distancia de '{neighbor}' a {distance}.")
        
        visited.add(current_node)
        print(f"Nodo '{current_node}' marcado como visitado.")
        print(f"Distancias actuales: {distances}")
        step += 1
        time.sleep(1)
    
    print("\nDistancias finales desde el nodo de inicio:")
    print(distances)
    
    draw_graph(graph, pos, distances)
    unique_filename = get_unique_filename()
    plt.savefig(unique_filename)
    print(f"Gráfica final guardada como: {unique_filename}")
    plt.show()

def get_unique_filename(base_filename="dijkstra_graph", extension=".jpg"):
    counter = 1
    filename = f"{base_filename}{extension}"
    while os.path.exists(filename):
        filename = f"{base_filename}_{counter}{extension}"
        counter += 1
    return filename

graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 6},
    'C': {'A': 4, 'B': 2, 'D': 3},
    'D': {'B': 6, 'C': 3}
}

start_node = 'A'

plt.ion()  
dijkstra_simulation(graph, start_node)
