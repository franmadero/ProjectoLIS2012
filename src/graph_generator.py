import random
import math

class Graph:
    """Representa un grafo ponderado como una lista de adyacencia."""
    def __init__(self, num_nodes):
        self.num_nodes = num_nodes
        # {nodo_origen: [(nodo_destino, peso), ...]}
        self.adj = {i: [] for i in range(num_nodes)}
        # {nodo: (x, y)} para la heurística
        self.coordinates = {i: (random.random() * 100, random.random() * 100) for i in range(num_nodes)}
    
    def add_edge(self, u, v, weight):
        # El grafo es no dirigido para simplificar la generación
        self.adj[u].append((v, weight))
        self.adj[v].append((u, weight))

    def get_coords(self, node):
        return self.coordinates.get(node)
        
    def generate_random_graph(self, avg_degree=4):
        """Genera un grafo disperso donde el peso es la distancia euclidiana."""
        for u in range(self.num_nodes):
            coords_u = self.get_coords(u)
            
            # Conectar cada nodo a un promedio de 'avg_degree' vecinos cercanos
            potential_neighbors = [v for v in range(self.num_nodes) if v != u]
            
            # Ordenar por distancia euclidiana para asegurar conectividad local
            potential_neighbors.sort(key=lambda v: self.euclidean_distance(coords_u, self.get_coords(v)))
            
            # Seleccionar los 'avg_degree' vecinos más cercanos
            neighbors_to_connect = potential_neighbors[:avg_degree]
            
            for v in neighbors_to_connect:
                if v not in [n[0] for n in self.adj[u]]: # Evitar duplicados si ya está conectado por v
                    coords_v = self.get_coords(v)
                    weight = self.euclidean_distance(coords_u, coords_v)
                    self.add_edge(u, v, weight)

    @staticmethod
    def euclidean_distance(coord1, coord2):
        """Calcula la distancia euclidiana entre dos coordenadas (x, y)."""
        return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

# --- Generación de Test Cases ---

def generate_test_cases(graph, num_cases=10):
    """Genera 10 pares origen-destino aleatorios."""
    nodes = list(graph.adj.keys())
    test_cases = []
    for _ in range(num_cases):
        # Aseguramos que origen y destino no sean el mismo nodo
        origin = random.choice(nodes)
        destination = random.choice([n for n in nodes if n != origin])
        test_cases.append((origin, destination))
    return test_cases

if __name__ == '__main__':
    # Ejemplo de uso
    G_medium = Graph(500)
    G_medium.generate_random_graph()
    print(f"Grafo de {G_medium.num_nodes} nodos generado.")
    print(f"Número de aristas del nodo 0: {len(G_medium.adj[0])}")
    
    cases = generate_test_cases(G_medium)
    print(f"Ejemplo de caso de prueba: {cases[0]}")