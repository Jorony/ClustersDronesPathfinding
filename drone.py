import pygame
import math
import random
import time
from pathfinding_algorithms import dijkstra, a_star
from config import *

class Drone:
    """
    无人机对象，支持单体与集群、动态/静态避障、通信延迟等
    
    主要功能:
    - 路径规划和跟踪
    - 动态和静态障碍物避障
    - 集群形成和协调移动
    - 模拟通信延迟和传感器误差
    """
    def __init__(self, start, goal, grid, algorithm=DEFAULT_ALGORITHM, color=BLUE, 
                 boundary_start=BOUNDARY_START, boundary_end=BOUNDARY_END):
        # 位置和路径属性
        self.pos = tuple(start)
        self.goal = tuple(goal)
        self.algorithm = algorithm
        self.color = color
        self.boundary_start = boundary_start
        self.boundary_end = boundary_end
        self.path, self.calc_time = self.plan_path(grid)
        self.path_index = 0
        self.path_length = len(self.path) if self.path else 0
        self.trajectory = []
        self.actual_path_length = 0
        
        # 物理运动属性
        self.velocity = (0, 0)
        self.speed = DEFAULT_SPEED
        self.inertia = DEFAULT_INERTIA
        self.max_speed = DEFAULT_MAX_SPEED
        self.max_acceleration = DEFAULT_MAX_ACCEL
        
        # 避障属性
        self.base_influence_radius = DEFAULT_BASE_INFLUENCE_RADIUS
        self.base_speed_threshold = DEFAULT_BASE_SPEED_THRESHOLD
        self.obstacle_avoidance_weight = DEFAULT_OBS_AVOID_WEIGHT
        self.obstacle_detection_radius = DEFAULT_OBS_DETECT_RADIUS
        self.sensor_error_probability = DEFAULT_SENSOR_ERROR_PROB
        
        # 集群属性
        self.clustering = False
        self.in_cluster = False
        self.cluster_formed = False
        self.cluster_center = None
        self.min_cluster_distance = DEFAULT_MIN_CLUSTER_DIST
        self.max_cluster_distance = DEFAULT_MAX_CLUSTER_DIST
        self.cluster_attraction_weight = DEFAULT_CLUSTER_ATTRACT
        self.cluster_repulsion_weight = DEFAULT_CLUSTER_REPEL
        self.cluster_path = None
        self.cluster_path_index = 0
        self.is_leader = False
        
        # 通信属性
        self.received_positions = {}
        self.current_speed_factor = 1.0
        self.position_timeout = DEFAULT_POSITION_TIMEOUT
        self.last_broadcast_time = 0
        self.broadcast_interval = DEFAULT_BROADCAST_INTERVAL
        self.drone_id = id(self)
        self.current_delay = 0
        self.total_delay = 0
        self.delay_count = 0
        
        # 统计属性
        self.start_time = time.time()
        self.end_time = None
        self.avoidance_failures = 0
        self.avoidance_attempts = 0
        self.avoidance_successes = 0
        
        # 用于缓存上一次遇到的动态障碍物
        self._last_dynamic_obstacles = []

    def plan_path(self, grid):
        """使用指定算法(A*或Dijkstra)规划路径"""
        if self.algorithm == 'A*':
            return a_star(self.pos, self.goal, grid, 
                           boundary_start=self.boundary_start, 
                           boundary_end=self.boundary_end)
        elif self.algorithm == 'Dijkstra':
            return dijkstra(self.pos, self.goal, grid, 
                            boundary_start=self.boundary_start, 
                            boundary_end=self.boundary_end)
        else:
            raise ValueError(f"未知算法: {self.algorithm}")
            
    def get_total_time(self):
        """获取总耗时"""
        return self.end_time - self.start_time if self.start_time and self.end_time else None

    def get_avoidance_success_rate(self):
        """计算避障成功率"""
        return 1.0 if self.avoidance_attempts == 0 else (self.avoidance_attempts - self.avoidance_failures) / self.avoidance_attempts

    def get_average_delay(self):
        """获取平均通信延迟"""
        return self.total_delay / self.delay_count if self.delay_count > 0 else 0

    def update(self, dynamic_obstacles, grid, other_drones=None, cluster_mode=False):
        """
        无人机主更新函数，包含集群、避障、路径跟随等
        
        Args:
            dynamic_obstacles: 动态障碍物列表
            grid: 网格障碍物地图
            other_drones: 其他无人机列表，用于集群行为
            cluster_mode: 是否启用集群模式
            
        Returns:
            int: 避障计数
        """
        avoidance_count = 0
        self._last_dynamic_obstacles = dynamic_obstacles
        self.clustering = cluster_mode
        
        # 检查是否还有剩余路径
        if not (self.path and self.path_index < len(self.path)):
            return avoidance_count
            
        # 计算下一个目标点的方向与距离
        next_pos = self.path[self.path_index]
        dx, dy = next_pos[0] - self.pos[0], next_pos[1] - self.pos[1]
        dist = math.hypot(dx, dy)
        
        if dist > 0:
            # 根据不同状态更新速度
            if self.clustering and other_drones and not self.cluster_formed:
                # 集群未形成，执行集群形成行为
                self._apply_clustering_behavior(other_drones, grid)
            elif self.clustering and self.cluster_formed and not self._is_at_goal():
                # 集群已形成，沿集群路径移动
                self._follow_cluster(other_drones, grid)
            else:
                # 普通路径跟随
                self._update_velocity(dx, dy, dist, dynamic_obstacles, grid)
                
            # 根据计算的速度更新位置
            self._update_position(grid)
            
        # 处理避障，更新计数
        avoidance_count += self._handle_obstacle_avoidance(dynamic_obstacles, dist)
        
        # 检查是否到达当前路径点
        if dist < self.speed:
            self.path_index += 1
            self.velocity = (0, 0)
            
        # 检查是否到达终点
        if self.path_index >= len(self.path):
            self.end_time = time.time()
            
        # 记录轨迹
        self.trajectory.append(self.pos)
        return avoidance_count

    def _is_at_goal(self):
        """判断是否到达终点"""
        if not self.goal:
            return False
        return math.hypot(self.pos[0] - self.goal[0], self.pos[1] - self.goal[1]) < 1

    def _find_nearest_free(self, x, y, grid):
        """
        BFS算法寻找最近的非障碍物点
        
        Args:
            x, y: 起始点坐标
            grid: 网格障碍物地图
            
        Returns:
            tuple: 找到的最近可通行点坐标 (x, y)
        """
        from collections import deque
        visited = set()
        queue = deque([(int(round(x)), int(round(y)), 0)])
        
        while queue:
            cx, cy, _ = queue.popleft()
            if (0 <= cx < len(grid) and 0 <= cy < len(grid[0]) and (cx, cy) not in visited):
                visited.add((cx, cy))
                if grid[cx][cy] == 0:  # 找到无障碍点
                    return cx, cy
                # 四方向搜索
                for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                    queue.append((cx+dx, cy+dy, 0))
                    
        # 未找到时返回原点
        return x, y

    def _apply_clustering_behavior(self, other_drones, grid):
        """
        集群形成前的聚合行为
        
        Args:
            other_drones: 其他无人机列表
            grid: 网格障碍物地图
        """
        # 计算所有无人机的中心位置
        total_pos = [self.pos[0], self.pos[1]]
        drone_count = 1
        for drone in other_drones:
            if drone is not self:
                total_pos[0] += drone.pos[0]
                total_pos[1] += drone.pos[1]
                drone_count += 1
        center_x, center_y = total_pos[0] / drone_count, total_pos[1] / drone_count
        
        # 检查集群中心位置是否有障碍物，如果有则寻找替代位置
        grid_cx, grid_cy = int(round(center_x)), int(round(center_y))
        if (0 <= grid_cx < len(grid) and 0 <= grid_cy < len(grid[0]) and 
            grid[grid_cx][grid_cy] == 1):
            # 在周围寻找最近的可通行点
            found_valid_center = False
            search_radius = 1
            while not found_valid_center and search_radius < 10:  # 限制搜索范围
                for dx in range(-search_radius, search_radius + 1):
                    for dy in range(-search_radius, search_radius + 1):
                        test_x = grid_cx + dx
                        test_y = grid_cy + dy
                        if (0 <= test_x < len(grid) and 0 <= test_y < len(grid[0]) and 
                            grid[test_x][test_y] == 0):
                            center_x, center_y = test_x, test_y
                            found_valid_center = True
                            break
                    if found_valid_center:
                        break
                search_radius += 1
            
            # 如果找不到合适的替代位置，使用当前无人机位置作为集群中心
            if not found_valid_center:
                center_x, center_y = self.pos[0], self.pos[1]
        
        self.cluster_center = (center_x, center_y)
        
        # 计算到集群中心的方向和距离
        dx, dy = center_x - self.pos[0], center_y - self.pos[1]
        dist_to_center = math.hypot(dx, dy)
        
        # 判断所有无人机是否都在集群范围内
        all_drones_in_position = all(
            math.hypot(drone.pos[0] - center_x, drone.pos[1] - center_y) <= self.max_cluster_distance
            for drone in other_drones if drone is not self
        )
        
        # 如果所有无人机都接近集群中心，更新集群状态
        if all_drones_in_position and dist_to_center <= self.max_cluster_distance:
            self.in_cluster = True
            if all(drone.in_cluster for drone in other_drones if drone is not self):
                if not any(drone.cluster_formed for drone in other_drones):
                    # 首次形成集群，当前无人机成为leader
                    for drone in other_drones:
                        drone.cluster_formed = True
                        drone.is_leader = False
                    self.cluster_formed = True
                    self.is_leader = True
                    self._plan_cluster_path(grid, other_drones)
                else:
                    # 其他无人机已经形成集群，加入
                    self.cluster_formed = True
        
        # 计算集群行为的目标速度
        target_velocity = [0, 0]
        
        # 添加集群中心的吸引力
        if dist_to_center > 0:
            target_velocity[0] += (dx / dist_to_center) * self.speed * self.cluster_attraction_weight
            target_velocity[1] += (dy / dist_to_center) * self.speed * self.cluster_attraction_weight
        
        # 添加无人机间的相互作用力
        for drone in other_drones:
            if drone is not self:
                drone_dx = drone.pos[0] - self.pos[0]
                drone_dy = drone.pos[1] - self.pos[1]
                drone_dist = math.hypot(drone_dx, drone_dy)
                
                if drone_dist > 0:
                    # 太近时产生排斥力
                    if drone_dist < self.min_cluster_distance:
                        repulsion = (self.min_cluster_distance - drone_dist) / self.min_cluster_distance
                        target_velocity[0] -= (drone_dx / drone_dist) * repulsion * self.cluster_repulsion_weight
                        target_velocity[1] -= (drone_dy / drone_dist) * repulsion * self.cluster_repulsion_weight
                    # 太远时产生吸引力
                    elif drone_dist > self.max_cluster_distance:
                        attraction = (drone_dist - self.max_cluster_distance) / self.max_cluster_distance
                        target_velocity[0] += (drone_dx / drone_dist) * attraction * self.cluster_attraction_weight
                        target_velocity[1] += (drone_dy / drone_dist) * attraction * self.cluster_attraction_weight
        
        # 静态障碍物避障
        if self._avoid_static_obstacles(target_velocity, grid):
            # 若附近有障碍物，减小速度以提高安全性
            target_velocity[0] *= 0.5
            target_velocity[1] *= 0.5
            
        # 应用计算得到的速度
        self._apply_velocity(target_velocity)

    def _plan_cluster_path(self, grid, other_drones):
        """为集群规划从当前位置到目标的路径"""
        center = self.cluster_center
        algo = a_star if self.algorithm == 'A*' else dijkstra
        
        # 从集群中心到目标规划路径
        path, _ = algo((int(center[0]), int(center[1])), self.goal, grid,
                       boundary_start=self.boundary_start, boundary_end=self.boundary_end)
                       
        # 更新所有无人机的集群路径
        for drone in other_drones:
            drone.cluster_path = path
            drone.cluster_path_index = 0

    def _follow_cluster(self, other_drones, grid):
        """集群已形成后，沿集群路径移动的行为"""
        # 检查是否到达目标
        my_dist_to_goal = math.hypot(self.goal[0] - self.pos[0], self.goal[1] - self.pos[1])
        goal_radius = DEFAULT_GOAL_RADIUS
        
        # 如果已到达目标或路径已完成，停止移动
        if my_dist_to_goal <= goal_radius or not self.cluster_path or self.cluster_path_index >= len(self.cluster_path):
            self.velocity = (0, 0)
            return
            
        # 获取当前目标路径点
        target = self.cluster_path[self.cluster_path_index]
        dx, dy = target[0] - self.pos[0], target[1] - self.pos[1]
        dist = math.hypot(dx, dy)
        
        # 如果接近当前路径点，前进到下一点
        if dist < self.speed:
            self.cluster_path_index = min(self.cluster_path_index + 1, len(self.cluster_path) - 1)
            
        # 计算目标速度
        target_velocity = [(dx / dist) * self.speed, (dy / dist) * self.speed] if dist > 0 else [0, 0]
        
        # 应用动态障碍物势场
        self._apply_potential_field_to_velocity(target_velocity)
        
        # 更新无人机速度
        self._apply_velocity(target_velocity)

    def _apply_potential_field_to_velocity(self, velocity):
        """
        将动态障碍物势场直接应用到给定的速度向量
        
        Args:
            velocity: 要修改的速度向量 [vx, vy]
        """
        dynamic_obstacles = getattr(self, "_last_dynamic_obstacles", [])
        repulsive_force = [0, 0]
        
        # 计算所有障碍物的合力
        for obstacle in dynamic_obstacles:
            influence, direction = obstacle.get_influence(self.pos)
            dynamic_radius = self.base_influence_radius * (1 + influence / 15)
            
            # 检查障碍物是否在影响范围内
            if influence > 0 and math.hypot(*direction) < dynamic_radius:
                repulsion = min(5.0, influence * (1 - math.exp(-influence / 2)))
                repulsive_force[0] -= repulsion * direction[0]
                repulsive_force[1] -= repulsion * direction[1]
        
        # 应用排斥力到速度
        velocity[0] += repulsive_force[0]
        velocity[1] += repulsive_force[1]

    def _apply_velocity(self, velocity):
        """应用惯性和速度限制"""
        # 计算速度大小
        speed = math.hypot(velocity[0], velocity[1])
        
        # 如果速度不为零，应用速度限制
        if speed > 0:
            # 限制速度不超过最大值
            velocity[0] = (velocity[0] / speed) * min(self.max_speed, speed)
            velocity[1] = (velocity[1] / speed) * min(self.max_speed, speed)
            
            # 考虑惯性，平滑速度变化
            self.velocity = (
                self.velocity[0] * self.inertia + velocity[0] * (1 - self.inertia),
                self.velocity[1] * self.inertia + velocity[1] * (1 - self.inertia)
            )

    def _avoid_static_obstacles(self, velocity, grid):
        """
        静态障碍物人工势场避障
        
        Args:
            velocity: 当前速度向量 [vx, vy]
            grid: 网格障碍物地图
            
        Returns:
            bool: 是否检测到障碍物
        """
        grid_x, grid_y = int(self.pos[0]), int(self.pos[1])
        has_nearby_obstacle = False
        
        # 扫描周围区域检测障碍物
        for dx in range(-self.obstacle_detection_radius, self.obstacle_detection_radius + 1):
            for dy in range(-self.obstacle_detection_radius, self.obstacle_detection_radius + 1):
                nx, ny = grid_x + dx, grid_y + dy
                
                # 检查位置有效性和是否有障碍物
                if (0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] == 1):
                    has_nearby_obstacle = True
                    
                    # 计算障碍物相对位置和距离
                    obstacle_dx = nx - self.pos[0]
                    obstacle_dy = ny - self.pos[1]
                    dist = math.hypot(obstacle_dx, obstacle_dy)
                    
                    # 根据距离计算排斥力，距离越近排斥力越大
                    if dist > 0:
                        repulsion = (self.obstacle_detection_radius - dist) ** 2
                        velocity[0] -= (obstacle_dx / dist) * repulsion * self.obstacle_avoidance_weight
                        velocity[1] -= (obstacle_dy / dist) * repulsion * self.obstacle_avoidance_weight
                        
        return has_nearby_obstacle

    def broadcast_position(self, comm_manager, current_time):
        """广播自身位置信息，考虑广播间隔"""
        if current_time - self.last_broadcast_time >= self.broadcast_interval:
            self.last_broadcast_time = current_time
            return {
                'type': 'position',
                'position': self.pos,
                'velocity': self.velocity,
                'timestamp': current_time,
                'cluster_status': {
                    'in_cluster': self.in_cluster,
                    'cluster_formed': self.cluster_formed
                }
            }
        return None

    def handle_received_messages(self, messages, current_time, comm_manager):
        """处理接收到的通信消息，更新延迟与速度因子"""
        # 处理新收到的消息
        for msg in messages:
            sender_id = msg['sender_id']
            if msg['message']['type'] == 'position':
                message_delay = current_time - msg['message']['timestamp']
                self.received_positions[sender_id] = {
                    'position': msg['message']['position'],
                    'delay': message_delay,
                    'timestamp': current_time
                }
        
        # 清理超时的位置信息
        for drone_id in list(self.received_positions.keys()):
            if current_time - self.received_positions[drone_id]['timestamp'] > self.position_timeout:
                del self.received_positions[drone_id]
        
        # 计算平均延迟和速度因子
        if self.received_positions:
            avg_delay = sum(info['delay'] for info in self.received_positions.values()) / len(self.received_positions)
            self.current_delay = avg_delay
            self.total_delay += avg_delay
            self.delay_count += 1
            self.current_speed_factor = comm_manager.get_speed_factor(avg_delay)
        else:
            self.current_speed_factor = 1.0

    def _handle_obstacle_avoidance(self, dynamic_obstacles, dist):
        """处理动态障碍物避障，更新避障统计"""
        avoidance_count = 0
        
        for obstacle in dynamic_obstacles:
            # 计算障碍物影响
            influence, _ = obstacle.get_influence(self.pos)
            
            # 根据距离调整动态影响半径和阈值
            dynamic_radius = self.base_influence_radius * (1 + dist / 15)
            dynamic_threshold = self.base_speed_threshold * (1 + dist / 25)
            
            # 如果影响超过阈值，则需要避障
            if influence > dynamic_threshold:
                self.avoidance_attempts += 1
                
                # 模拟传感器误差，成功率与障碍物大小相关
                if random.random() < (self.sensor_error_probability / obstacle.size):
                    self.avoidance_failures += 1
                    self.trajectory.append(("failure", self.pos))
                else:
                    self.avoidance_successes += 1
                    avoidance_count += 1
                    self.trajectory.append(("avoidance", self.pos))
                    
        return avoidance_count

    def _apply_potential_field(self, dynamic_obstacles, grid):
        """
        对速度应用动态障碍物人工势场
        
        Args:
            dynamic_obstacles: 动态障碍物列表
            grid: 网格障碍物地图
        """
        # 计算所有动态障碍物产生的合力
        repulsive_force = [0, 0]
        for obstacle in dynamic_obstacles:
            influence, direction = obstacle.get_influence(self.pos)
            dynamic_radius = self.base_influence_radius * (1 + influence / 15)
            
            # 如果障碍物在影响范围内
            if influence > 0 and math.hypot(*direction) < dynamic_radius:
                # 非线性映射影响力，避免过度反应
                repulsion = min(5.0, influence * (1 - math.exp(-influence / 2)))
                repulsive_force[0] -= repulsion * direction[0]
                repulsive_force[1] -= repulsion * direction[1]
        
        # 计算加速度向量
        prev_velocity = self.velocity
        acceleration = (repulsive_force[0], repulsive_force[1])
        
        # 限制最大加速度
        acceleration_magnitude = math.hypot(*acceleration)
        if acceleration_magnitude > self.max_acceleration:
            scaling_factor = self.max_acceleration / acceleration_magnitude
            acceleration = (acceleration[0] * scaling_factor, acceleration[1] * scaling_factor)
        
        # 更新速度，考虑加速度和平滑因子
        self.velocity = (
            self.velocity[0] + acceleration[0],
            self.velocity[1] + acceleration[1]
        )
        
        # 平滑速度变化
        self.velocity = (
            prev_velocity[0] * 0.3 + self.velocity[0] * 0.7,
            prev_velocity[1] * 0.3 + self.velocity[1] * 0.7
        )

    def _limit_speed(self, max_speed=None):
        """
        限制速度不超过给定的最大值
        
        Args:
            max_speed: 最大速度限制，若为None则使用默认值
        """
        if max_speed is None:
            max_speed = self.max_speed
            
        # 计算当前速度大小
        speed = math.hypot(self.velocity[0], self.velocity[1])
        
        # 如果超过最大速度，按比例缩放
        if speed > max_speed:
            self.velocity = (
                self.velocity[0] / speed * max_speed,
                self.velocity[1] / speed * max_speed
            )

    def _update_velocity(self, dx, dy, dist, dynamic_obstacles, grid):
        """根据目标方向和动态障碍物更新速度"""
        # 计算目标速度（朝向下一个路径点）
        target_velocity = ((dx / dist) * self.speed, (dy / dist) * self.speed)
        
        # 考虑惯性，平滑速度变化
        self.velocity = (
            self.velocity[0] * self.inertia + target_velocity[0] * (1 - self.inertia),
            self.velocity[1] * self.inertia + target_velocity[1] * (1 - self.inertia)
        )
        
        # 应用动态障碍物的人工势场
        self._apply_potential_field(dynamic_obstacles, grid)
        
        # 限制速度不超过最大值
        self._limit_speed()

    def _update_position(self, grid):
        """
        根据速度更新位置，考虑边界和障碍物
        
        Args:
            grid: 网格障碍物地图
        """
        # 检查是否已到达目标
        goal_radius = DEFAULT_GOAL_RADIUS
        dist_to_goal = math.hypot(self.goal[0] - self.pos[0], self.goal[1] - self.pos[1])
        if dist_to_goal <= goal_radius:
            self.velocity = (0, 0)
            return
        
        # 计算新位置
        new_pos = (self.pos[0] + self.velocity[0], self.pos[1] + self.velocity[1])
        
        # 边界检查和处理
        if new_pos[0] < self.boundary_start[0]:
            new_pos = (self.boundary_start[0], new_pos[1])
            self.velocity = (0, self.velocity[1])
        elif new_pos[0] > self.boundary_end[0]:
            new_pos = (self.boundary_end[0], new_pos[1])
            self.velocity = (0, self.velocity[1])
            
        if new_pos[1] < self.boundary_start[1]:
            new_pos = (new_pos[0], self.boundary_start[1])
            self.velocity = (self.velocity[0], 0)
        elif new_pos[1] > self.boundary_end[1]:
            new_pos = (new_pos[0], self.boundary_end[1])
            self.velocity = (self.velocity[0], 0)
        
        # 检查新位置是否有障碍物
        grid_x, grid_y = int(new_pos[0]), int(new_pos[1])
        if (0 <= grid_x < len(grid) and 0 <= grid_y < len(grid[0]) and grid[grid_x][grid_y] == 0):
            # 计算实际移动距离
            self.actual_path_length += math.hypot(new_pos[0] - self.pos[0], new_pos[1] - self.pos[1])
            # 更新位置
            self.pos = new_pos

    def draw(self, screen):
        """在屏幕上绘制无人机及其轨迹"""
        # 绘制轨迹
        for point in self.trajectory:
            # 特殊标记点(避障成功或失败)
            if isinstance(point, tuple) and len(point) == 2 and isinstance(point[0], str):
                color = GREEN if point[0] == "avoidance" else RED
                pygame.draw.circle(screen, color,
                                  (int(point[1][1] * CELL_SIZE), int(point[1][0] * CELL_SIZE)), 6)
            else:
                # 普通轨迹点
                traj_color = (self.color[0]//2, self.color[1]//2, self.color[2]//2)
                pygame.draw.circle(screen, traj_color,
                                  (int(point[1] * CELL_SIZE), int(point[0] * CELL_SIZE)), 2)
        
        # 绘制无人机本体
        pygame.draw.circle(screen, self.color,
                          (int(self.pos[1] * CELL_SIZE), int(self.pos[0] * CELL_SIZE)), 3)
        
        # 如果在集群中，添加白色指示环
        if self.in_cluster:
            pygame.draw.circle(screen, WHITE,
                              (int(self.pos[1] * CELL_SIZE), int(self.pos[0] * CELL_SIZE)), 5, 1)