import heapq
import time

def is_valid_neighbor(neighbor, grid, boundary_start=(0,0), boundary_end=(float('inf'),float('inf')), buffer=1):
    """
    判断邻居节点是否为有效可通行节点
    
    Args:
        neighbor: 待检查的点坐标 (x, y)
        grid: 网格障碍物地图
        boundary_start: 边界左上角点 (x, y)
        boundary_end: 边界右下角点 (x, y)
        buffer: 安全缓冲区大小
        
    Returns:
        bool: 是否可通行
    """
    x, y = neighbor
    
    # 检查是否在边界范围内
    if not (boundary_start[0] <= x <= boundary_end[0] and 
            boundary_start[1] <= y <= boundary_end[1]):
        return False
        
    # 检查是否在网格范围内
    if not (0 <= x < len(grid) and 0 <= y < len(grid[0])):
        return False
        
    # 检查周围buffer半径范围内是否有障碍物
    for dx in range(-buffer, buffer + 1):
        for dy in range(-buffer, buffer + 1):
            nx, ny = x + dx, y + dy
            if (0 <= nx < len(grid) and 
                0 <= ny < len(grid[0]) and 
                grid[nx][ny] == 1):
                return False
                
    return True

def reconstruct_path(came_from, current, start):
    """
    从came_from字典反向重建路径
    
    Args:
        came_from: 记录每个节点的前一个节点
        current: 终点
        start: 起点
        
    Returns:
        list: 从起点到终点的路径
    """
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(start)
    return path[::-1]  # 反转路径，从起点到终点

def dijkstra(start, goal, grid, boundary_start=(0,0), boundary_end=(float('inf'),float('inf'))):
    """
    Dijkstra最短路径算法
    
    Args:
        start: 起点坐标 (x, y)
        goal: 终点坐标 (x, y)
        grid: 网格障碍物地图
        boundary_start: 边界左上角点 (x, y)
        boundary_end: 边界右下角点 (x, y)
        
    Returns:
        tuple: (路径列表, 计算耗时)
    """
    start_time = time.time()
    
    # 初始化数据结构
    open_set = [(0, start)]  # 优先队列，(距离, 节点)
    came_from = {}  # 记录路径
    g_score = {start: 0}  # 从起点到当前点的实际距离
    
    # 四个方向的移动（上、右、下、左）
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    
    while open_set:
        # 取出当前最短距离的节点
        current_g, current = heapq.heappop(open_set)
        
        # 达到目标
        if current == goal:
            path = reconstruct_path(came_from, current, start)
            return path, time.time() - start_time
            
        # 探索邻居节点
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            
            if is_valid_neighbor(neighbor, grid, boundary_start, boundary_end):
                # 计算邻居的新距离
                tentative_g_score = g_score[current] + 1
                
                # 如果找到更短路径，则更新
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    heapq.heappush(open_set, (tentative_g_score, neighbor))
                    
    # 没找到路径
    return None, time.time() - start_time

def heuristic(a, b):
    """曼哈顿距离启发式函数"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, grid, boundary_start=(0,0), boundary_end=(float('inf'),float('inf'))):
    """
    A*最短路径算法
    
    Args:
        start: 起点坐标 (x, y)
        goal: 终点坐标 (x, y)
        grid: 网格障碍物地图
        boundary_start: 边界左上角点 (x, y)
        boundary_end: 边界右下角点 (x, y)
        
    Returns:
        tuple: (路径列表, 计算耗时)
    """
    start_time = time.time()
    
    # 初始化数据结构
    open_set = [(0, start)]  # 优先队列，(f_score, 节点)
    came_from = {}  # 记录路径
    g_score = {start: 0}  # 从起点到当前点的实际距离
    f_score = {start: heuristic(start, goal)}  # 估计总距离
    
    # 四个方向的移动（上、右、下、左）
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    
    while open_set:
        # 取出当前f值最小的节点
        _, current = heapq.heappop(open_set)
        
        # 达到目标
        if current == goal:
            path = reconstruct_path(came_from, current, start)
            return path, time.time() - start_time
            
        # 探索邻居节点
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            
            if is_valid_neighbor(neighbor, grid, boundary_start, boundary_end):
                # 计算邻居的新g值
                tentative_g_score = g_score[current] + 1
                
                # 如果找到更短路径，则更新
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    
    # 没找到路径
    return None, time.time() - start_time