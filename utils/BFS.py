'''
@Project :高精度地图
@Author : YuhaoDu
@Date : 2024/10/13 
'''
from collections import deque


def breadth_first_search(graph, start_node, end_node=None):
    # 创建一个队列用于BFS
    queue = deque([start_node])
    # 记录访问过的节点
    visited = set([start_node])
    # 记录路径
    path = {start_node: []}  # 使用字典记录每个节点的路径

    # 开始广度优先搜索
    while queue:
        # 从队列中取出一个节点
        current_node = queue.popleft()

        # 如果找到了目标节点，返回路径
        if current_node == end_node:
            return path[current_node]

        # 遍历当前节点的所有邻居节点
        for neighbor in graph[current_node]:
            # 如果邻居节点未被访问过
            if neighbor not in visited:
                # 标记为已访问
                visited.add(neighbor)
                # 将邻居节点加入队列
                queue.append(neighbor)
                # 记录路径
                path[neighbor] = path[current_node] + [neighbor]

    # 如果没有找到目标节点，返回None
    return None


# 示例图
graph = {
    'A': ['B', 'C'],
    'B': ['D', 'E'],
    'C': ['F'],
    'D': [],
    'E': ['F'],
    'F': []
}
if __name__ == '__main__':

    # 执行BFS
    start_node = 'A'
    end_node = 'F'
    path = breadth_first_search(graph, start_node, end_node)

    if path:
        print(f"Path from {start_node} to {end_node}: {path}")
    else:
        print(f"No path found from {start_node} to {end_node}")