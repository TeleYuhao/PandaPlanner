'''
@Project :高精度地图
@Author : YuhaoDu
@Date : 2024/10/13 
'''
import heapq


def dijkstra(graph, start, end):
    # 初始化距离字典，所有节点的距离设为无穷大，除了起始节点设为0
    distances = {vertex: float('infinity') for vertex in graph}
    distances[start] = 0

    # 初始化优先队列，存储(距离, 节点)元组
    priority_queue = [(0, start)]

    # 记录路径的字典
    previous_nodes = {vertex: None for vertex in graph}

    while priority_queue:
        # 弹出具有最小距离的节点
        current_distance, current_vertex = heapq.heappop(priority_queue)

        # 节点可能被重新加入优先队列，所以检查是否跳过
        if current_distance > distances[current_vertex]:
            continue

        # 遍历当前节点的所有邻居节点
        for neighbor, weight in graph[current_vertex].items():
            distance = current_distance + weight

            # 只有当找到更短的路径时才更新距离并添加到优先队列
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_vertex
                heapq.heappush(priority_queue, (distance, neighbor))

    # 重建路径
    path, current_vertex = [], end

    while previous_nodes[current_vertex] is not None:
        path.insert(0, current_vertex)
        current_vertex = previous_nodes[current_vertex]


    # 将起始节点添加到路径的开头
    path.insert(0, start)

    return path#, distances[end]

if __name__ == '__main__':

    # 示例图，使用字典的字典来表示带权重的图
    graph = {
        'A': {'B': 1, 'C': 4},
        'B': {'A': 1, 'D': 2, 'E': 5},
        'C': {'A': 4, 'F': 3},
        'D': {'B': 2},
        'E': {'B': 5, 'F': 2},
        'F': {'C': 3, 'E': 2}
    }

    # 执行Dijkstra算法
    start_node = 'A'
    end_node = 'F'
    path, min_distance = dijkstra(graph, start_node, end_node)

    if path:
        print(f"The shortest path from {start_node} to {end_node} is: {path}")
        print(f"The shortest distance is: {min_distance}")
    else:
        print(f"No path found from {start_node} to {end_node}")