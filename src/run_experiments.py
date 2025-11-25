
import csv
import time
import statistics
import os
from graph_generator import Graph, generate_test_cases
from dijkstra import dijkstra, a_star, d_star_lite

# --- Configuración del Experimento ---
GRAPH_SIZES = [100, 500, 1000] # Nodos V
NUM_RUNS = 3 # Repeticiones por caso
OUTPUT_FILE = "experiments_results.csv"
ALGORITHMS = {
    "Dijkstra": dijkstra,
    "A*": a_star,
    "D* Lite": d_star_lite
}

def setup_environment():
    """Garantiza que la carpeta de resultados exista."""
    if not os.path.exists('results'):
        os.makedirs('results')

def run_single_case(graph, algorithm_func, start, goal):
    """Ejecuta un algoritmo y retorna las métricas (costo, expansiones, tiempo)."""
    # La función ya retorna (costo, expansiones, tiempo_cpu)
    cost, expansions, runtime = algorithm_func(graph, start, goal)
    return {'cost': cost, 'expansions': expansions, 'runtime': runtime}

def main():
    setup_environment()
    
    # Encabezados del CSV
    fieldnames = ['V_Size', 'Pair_ID', 'Algorithm', 'Cost', 
                  'Mean_Time', 'StdDev_Time', 'Mean_Expansions', 'StdDev_Expansions']
    
    print("Iniciando experimentos...")
    
    with open(f'results/{OUTPUT_FILE}', 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for V_size in GRAPH_SIZES:
            print(f"\n--- Probando con Grafo de V={V_size} nodos ---")
            
            # 1. Generación del Grafo
            G = Graph(V_size)
            G.generate_random_graph()
            test_cases = generate_test_cases(G, num_cases=10) # 10 pares origen-destino

            for algo_name, algo_func in ALGORITHMS.items():
                
                for pair_id, (start_node, goal_node) in enumerate(test_cases):
                    
                    # 2. Corridas Múltiples
                    results_list = []
                    for _ in range(NUM_RUNS):
                        metrics = run_single_case(G, algo_func, start_node, goal_node)
                        results_list.append(metrics)
                    
                    # 3. Cálculo de Media y Desviación Estándar
                    runtime_values = [res['runtime'] for res in results_list]
                    expansion_values = [res['expansions'] for res in results_list]
                    
                    mean_runtime = statistics.mean(runtime_values)
                    std_dev_runtime = statistics.stdev(runtime_values) if len(runtime_values) > 1 else 0
                    
                    mean_expansions = statistics.mean(expansion_values)
                    std_dev_expansions = statistics.stdev(expansion_values) if len(expansion_values) > 1 else 0

                    # 4. Validación de Correctitud
                    final_cost = results_list[0]['cost'] 
                    # Se asume que el costo es constante en todas las corridas de un mismo caso
                    if final_cost == float('inf'):
                        print(f"  [Error] No se encontró ruta para {algo_name} ({start_node} -> {goal_node})")
                    
                    # 5. Exportación
                    writer.writerow({
                        'V_Size': V_size,
                        'Pair_ID': pair_id,
                        'Algorithm': algo_name,
                        'Cost': final_cost,
                        'Mean_Time': f"{mean_runtime:.6f}",
                        'StdDev_Time': f"{std_dev_runtime:.6f}",
                        'Mean_Expansions': f"{mean_expansions:.1f}",
                        'StdDev_Expansions': f"{std_dev_expansions:.1f}"
                    })
                    print(f"  > Caso V={V_size}, {algo_name}, Par {pair_id} completado. Tiempo medio: {mean_runtime:.4f}s")


    print(f"\n✅ Todos los experimentos han finalizado. Resultados guardados en results/{OUTPUT_FILE}")

if __name__ == '__main__':
    main()