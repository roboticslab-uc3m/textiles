
def dfs(graph, source, target):
    """
    Looks for the path from a source to a target node
    """
    visited = set()
    to_visit = [source]
    current_node = None
    origins = {}

    # Find target node
    while to_visit and current_node != target:
        next_node = to_visit.pop()

        if next_node not in visited:
            visited.add(next_node)
            next_nodes = graph.get(next_node, [])
            for node in next_nodes:
                if node not in origins:
                    origins[node] = next_node
            to_visit += next_nodes
            current_node = next_node

    if current_node != target:
        return None

    # Reconstruct path to target node
    path = []
    node = target

    while node != source:
        path.append(node)
        node = origins[node]
    path.append(source)

    return path


if __name__ == '__main__':
    graph = {1: [2, 3, 4],
             2: [5, 6],
             3: [7, 8],
             4: [9],
             7: [10, 11],
             9: [12, 13]}
    source = 3
    target = 11

    path = dfs(graph, source, target)
    print(path)