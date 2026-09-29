"""
A* shortest path algorithm for road networks.
"""

import heapq
import math


def astar_search(road_graph, xy_nodes, start_node, goal_node):
    """
    Computes shortest path between start_node and goal_node on road_graph.
    
    Args:
        road_graph: dict of u -> {v: distance_m}
        xy_nodes: dict of node_id -> (x, y)
        start_node: start node ID
        goal_node: target goal node ID
        
    Returns:
        path: list of node IDs forming shortest path, or None if no path exists
        total_cost: total path length in meters
    """
    if start_node not in road_graph or goal_node not in road_graph:
        return None, float('inf')

    if start_node == goal_node:
        return [start_node], 0.0

    goal_xy = xy_nodes[goal_node]

    # Priority queue stores tuples: (f_score, g_score, current_node, path)
    # f = g + h
    h_start = math.hypot(xy_nodes[start_node][0] - goal_xy[0], xy_nodes[start_node][1] - goal_xy[1])
    pq = [(h_start, 0.0, start_node, [start_node])]
    visited = {}

    while pq:
        f_score, g_score, curr, path = heapq.heappop(pq)

        if curr in visited and visited[curr] <= g_score:
            continue
        visited[curr] = g_score

        if curr == goal_node:
            return path, g_score

        for nbr, edge_dist in road_graph[curr].items():
            new_g = g_score + edge_dist
            if nbr not in visited or visited[nbr] > new_g:
                h = math.hypot(xy_nodes[nbr][0] - goal_xy[0], xy_nodes[nbr][1] - goal_xy[1])
                heapq.heappush(pq, (new_g + h, new_g, nbr, path + [nbr]))

    return None, float('inf')
