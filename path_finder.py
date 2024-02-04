import heapq

#Questo metodo prende in input il vertice di partenza e una lista di possibili destinazioni
# per prima cosa trova la destinazione più vicina ed infine restituisce il percorso più veloce per raggiungerla
def find_path_to_nearest_block(adjacency_list, vertices_positions, source, destinations):

    def shortest_path(adjacency_list, vertices_positions, source_vertex, destination_vertices):
        def calculate_distance(v1, v2):
            return ((v1.x - v2.x) ** 2 + (v1.y - v2.y) ** 2) ** 0.5

        num_vertices = len(adjacency_list)
        distances = [float('inf')] * num_vertices
        visited = [False] * num_vertices
        previous_vertices = [-1] * num_vertices

        distances[source_vertex] = 0
        priority_queue = [(0, source_vertex)]

        while priority_queue:
            current_distance, current_vertex = heapq.heappop(priority_queue)

            if visited[current_vertex]:
                continue

            visited[current_vertex] = True

            for neighbor in adjacency_list[current_vertex]:
                distance_to_neighbor = calculate_distance(vertices_positions[current_vertex],
                                                        vertices_positions[neighbor])

                if distances[current_vertex] + distance_to_neighbor < distances[neighbor]:
                    distances[neighbor] = distances[current_vertex] + distance_to_neighbor
                    previous_vertices[neighbor] = current_vertex
                    heapq.heappush(priority_queue, (distances[neighbor], neighbor))

        min_cost = float('inf')
        min_cost_destination = None

        for destination in destination_vertices:
            if distances[destination] < min_cost:
                min_cost = distances[destination]
                min_cost_destination = destination

        path = []
        current_vertex = min_cost_destination
        while current_vertex != -1:
            path.insert(0, current_vertex)
            current_vertex = previous_vertices[current_vertex]

        return path, min_cost_destination

    path, destination = shortest_path(adjacency_list, vertices_positions, source, destinations)
    return path, destination