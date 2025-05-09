import pygame
import asyncio
import platform
import random
import math
import time
from drone import Drone
from button import Button
from dynamic_obstacle import DynamicObstacle
from communication import CommunicationManager
from config import *

class SimulationApp:
    """无人机路径规划与集群避障仿真应用"""
    def __init__(self):
        # 初始化 Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((GRID_SIZE * CELL_SIZE, GRID_SIZE * CELL_SIZE + 50))
        pygame.display.set_caption("无人机集群避障仿真")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(pygame.font.match_font('simhei'), 13)
        
        # 环境设置
        self.grid = [[0] * GRID_SIZE for _ in range(GRID_SIZE)]
        self.goal = GOAL_POS
        self.current_algorithm = DEFAULT_ALGORITHM
        self.brush_size = BRUSH_SIZE
        self.mode = DEFAULT_MODE
        self.placing = False
        self.drones = []  # 多个无人机列表
        self.avoidance_count = 0
        self.dynamic_obstacles_enabled = DYNAMIC_OBSTACLES_ENABLED
        self.dynamic_obstacles = [DynamicObstacle(GRID_SIZE, CELL_SIZE) for _ in range(DYNAMIC_OBSTACLE_COUNT)]
        self.cluster_mode = CLUSTER_MODE_ENABLED
        self.drone_count = 3  # 当前无人机数量（1或3）
        
        # 初始化无人机
        self.initialize_drones()
        
        # 创建UI按钮
        self.create_buttons()
        
        # 通信管理器
        self.comm_manager = CommunicationManager(
            latency_range=COMMUNICATION_LATENCY_RANGE,
            packet_loss_prob=PACKET_LOSS_PROBABILITY
        )

    def create_buttons(self):
        """创建界面按钮"""
        self.algorithm_button = Button(10, GRID_SIZE * CELL_SIZE + 10, 80, 30, "切换算法", self.switch_algorithm)
        self.mode_button = Button(120, GRID_SIZE * CELL_SIZE + 10, 80, 30, "模式: 放置", self.switch_mode)
        self.obstacle_button = Button(230, GRID_SIZE * CELL_SIZE + 10, 120, 30, "动态障碍: 开启", self.toggle_dynamic_obstacles)
        self.cluster_button = Button(370, GRID_SIZE * CELL_SIZE + 10, 120, 30, "集群模式: 开启", self.toggle_cluster_mode)
        self.drone_count_button = Button(510, GRID_SIZE * CELL_SIZE + 10, 120, 30, f"无人机数量: {self.drone_count}", self.switch_drone_count)
        
        # 所有按钮列表，方便批量处理
        self.buttons = [
            self.algorithm_button,
            self.mode_button,
            self.obstacle_button,
            self.cluster_button,
            self.drone_count_button
        ]

    def switch_algorithm(self):
        """切换路径规划算法"""
        self.current_algorithm = 'Dijkstra' if self.current_algorithm == 'A*' else 'A*'
        # 更新所有无人机的算法
        self.initialize_drones()

    def switch_mode(self):
        """切换放置/移除模式"""
        self.mode = 'remove' if self.mode == 'place' else 'place'
        self.mode_button.text = f"模式: {'移除' if self.mode == 'remove' else '放置'}"

    def toggle_dynamic_obstacles(self):
        """切换动态障碍物启用状态"""
        self.dynamic_obstacles_enabled = not self.dynamic_obstacles_enabled
        self.obstacle_button.text = f"动态障碍: {'开启' if self.dynamic_obstacles_enabled else '关闭'}"

    def toggle_cluster_mode(self):
        """切换集群模式启用状态"""
        self.cluster_mode = not self.cluster_mode
        self.cluster_button.text = f"集群模式: {'开启' if self.cluster_mode else '关闭'}"
        # 重新初始化无人机
        self.initialize_drones()

    def switch_drone_count(self):
        """切换无人机数量"""
        self.drone_count = 1 if self.drone_count == 3 else 3
        self.drone_count_button.text = f"单体/集群: {'单体' if self.drone_count == 1 else '集群'}"
        self.initialize_drones()

    def initialize_drones(self):
        """根据无人机数量初始化无人机"""
        if self.drone_count == 1:
            start_points = [DRONE_START_POSITIONS[0]]
            colors = [DRONE_COLORS[0]]
        else:
            start_points = DRONE_START_POSITIONS
            colors = DRONE_COLORS
            
        self.drones = [
            Drone(start_points[i], self.goal, self.grid, algorithm=self.current_algorithm, color=colors[i],
                 boundary_start=BOUNDARY_START, boundary_end=BOUNDARY_END) 
            for i in range(self.drone_count)
        ]

    def handle_mouse_event(self, event):
        """处理鼠标事件，用于放置/移除障碍物"""
        # 检查按钮点击
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            for button in self.buttons:
                if button.rect.collidepoint(event.pos):
                    button.handle_event(event)
                    return True
        
        # 只处理鼠标类型事件，这些事件才有pos属性
        if event.type in (pygame.MOUSEBUTTONDOWN, pygame.MOUSEBUTTONUP, pygame.MOUSEMOTION):
            # 鼠标在网格范围内处理
            if event.pos[1] < GRID_SIZE * CELL_SIZE:
                # 鼠标按下事件
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    self.placing = True
                    self.modify_grid(event.pos)
                    return True
                # 鼠标松开事件
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    self.placing = False
                    return True
                # 鼠标移动事件
                elif event.type == pygame.MOUSEMOTION and self.placing:
                    self.modify_grid(event.pos)
                    return True
                
        return False

    def modify_grid(self, pos):
        """修改网格上的障碍物"""
        # 计算网格坐标
        grid_x = pos[1] // CELL_SIZE
        grid_y = pos[0] // CELL_SIZE
        
        # 使用刷子修改网格
        for i in range(-self.brush_size // 2 + 1, self.brush_size // 2 + 1):
            for j in range(-self.brush_size // 2 + 1, self.brush_size // 2 + 1):
                new_x = grid_x + i
                new_y = grid_y + j
                
                # 确保在网格范围内
                if 0 <= new_x < GRID_SIZE and 0 <= new_y < GRID_SIZE:
                    self.grid[new_x][new_y] = 1 if self.mode == 'place' else 0
        
        # 重新初始化所有无人机
        self.initialize_drones()

    def draw_grid_and_obstacles(self):
        """绘制网格和静态障碍物"""
        # 只绘制实际有障碍物的格子，提高性能
        for i in range(GRID_SIZE):
            for j in range(GRID_SIZE):
                if self.grid[i][j] == 1:  # 障碍物
                    pygame.draw.rect(self.screen, BLACK, 
                                  (j * CELL_SIZE, i * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    def draw_start_goal(self):
        """绘制起点和终点"""
        # 绘制每个无人机的起点
        for drone in self.drones:
            if drone.path and len(drone.path) > 0:
                start_pos = drone.path[0]
                pygame.draw.rect(self.screen, drone.color, 
                              (start_pos[1] * CELL_SIZE, start_pos[0] * CELL_SIZE, CELL_SIZE, CELL_SIZE))
        
        # 绘制终点区域
        goal_radius = GOAL_RADIUS
        pygame.draw.circle(self.screen, YELLOW, 
                           (self.goal[1] * CELL_SIZE, self.goal[0] * CELL_SIZE), 
                           goal_radius * CELL_SIZE, 0)

    def draw_info_panel(self):
        """绘制信息面板"""
        info_y = INFO_START_Y
        # 绘制面板背景
        pygame.draw.rect(self.screen, GRAY, (INFO_START_X-10, INFO_START_Y + 10, INFO_WIDTH, 195), 0)
        
        # 绘制算法和集群信息
        algo_text = self.font.render(f"路径算法: {self.current_algorithm}  集群模式: {'开启' if self.cluster_mode else '关闭'}", True, BLACK)
        self.screen.blit(algo_text, (INFO_START_X + 10, info_y + 15))
        info_y += INFO_LINE_HEIGHT
        
        # 绘制无人机数量信息
        drone_count_text = self.font.render(f"无人机数量: {self.drone_count}", True, BLACK)
        self.screen.blit(drone_count_text, (INFO_START_X + 230, info_y - 5))
        
        # 如果有无人机，绘制每个无人机的信息
        if self.drones:
            self._draw_drone_info(info_y)

    def _draw_drone_info(self, info_y):
        """绘制每个无人机的详细信息"""
        column_width = INFO_WIDTH // 3
        
        for i, drone in enumerate(self.drones):
            column = i
            column_x = INFO_START_X + 20 + (column * column_width)
            column_y = info_y + 20
            
            # 绘制无人机指示器和标题
            drone_indicator = pygame.Rect(column_x, column_y, 10, 10)
            pygame.draw.rect(self.screen, drone.color, drone_indicator)
            drone_text = self.font.render(f"无人机 {i+1}:", True, BLACK)
            self.screen.blit(drone_text, (column_x + 15, column_y))
            column_y += INFO_LINE_HEIGHT
            
            # 基本信息行
            texts = [
                f"路径长度: {drone.path_length}",
                f"实际长度: {drone.actual_path_length:.2f}",
                f"避障率: {drone.get_avoidance_success_rate() * 100:.2f}%"
            ]
            
            # 仅多无人机时显示延迟、速度因子、集群状态
            if self.drone_count > 1:
                texts += [
                    f"当前延迟: {drone.current_delay:.3f}s",
                    f"平均延迟: {drone.get_average_delay():.3f}s",
                    f"速度因子: {drone.current_speed_factor:.2f}"
                ]
                
            # 绘制各行信息
            for text in texts:
                text_surf = self.font.render(text, True, BLACK)
                self.screen.blit(text_surf, (column_x + 10, column_y))
                column_y += INFO_LINE_HEIGHT
                
            # 绘制总耗时
            if drone.get_total_time() is not None:
                total_time_text = self.font.render(f"耗时: {drone.get_total_time():.4f}s", True, BLACK)
                self.screen.blit(total_time_text, (column_x + 10, column_y))
                column_y += INFO_LINE_HEIGHT
                
            # 仅多无人机时显示集群状态
            if self.cluster_mode and self.drone_count > 1:
                cluster_status = "已形成集群" if drone.cluster_formed else "集群中" if drone.in_cluster else "移动中"
                cluster_text = self.font.render(f"集群状态: {cluster_status}", True, BLACK)
                self.screen.blit(cluster_text, (column_x + 10, column_y))
                
            # 仅多无人机时绘制速度因子条
            if self.drone_count > 1:
                self._draw_speed_indicator(drone, column_x, column_y)

    def _draw_speed_indicator(self, drone, x, y):
        """绘制无人机速度因子指示条"""
        speed_factor = drone.current_speed_factor
        indicator_x = x + 10
        indicator_y = y + INFO_LINE_HEIGHT
        
        # 绘制背景条
        pygame.draw.rect(self.screen, GRAY, 
                       (indicator_x, indicator_y, 
                        SPEED_INDICATOR_WIDTH, SPEED_INDICATOR_HEIGHT))
                        
        # 根据速度因子计算颜色并绘制前景条
        red = min(255, max(0, int(255 * (1 - speed_factor))))
        green = min(255, max(0, int(255 * speed_factor)))
        color = (red, green, 0)
        pygame.draw.rect(self.screen, color,
                       (indicator_x, indicator_y,
                        int(SPEED_INDICATOR_WIDTH * speed_factor), SPEED_INDICATOR_HEIGHT))

    def draw_boundary(self):
        """绘制黑色边界线"""
        # 计算像素坐标
        start_x = BOUNDARY_START[1] * CELL_SIZE
        start_y = BOUNDARY_START[0] * CELL_SIZE
        end_x = BOUNDARY_END[1] * CELL_SIZE
        end_y = BOUNDARY_END[0] * CELL_SIZE
        width = end_x - start_x
        height = end_y - start_y
        
        # 绘制边界线框
        pygame.draw.rect(self.screen, BLACK, (start_x, start_y, width, height), BOUNDARY_LINE_THICKNESS)

    def update_drones(self, current_time):
        """更新所有无人机状态，包括通信和位置"""
        for drone in self.drones:
            # 广播位置
            message = drone.broadcast_position(self.comm_manager, current_time)
            if message:
                for other_drone in self.drones:
                    if other_drone != drone:
                        self.comm_manager.send_message(drone.drone_id, other_drone.drone_id, message, current_time)
            
            # 接收消息并更新
            messages = self.comm_manager.receive_messages(drone.drone_id, current_time)
            drone.handle_received_messages(messages, current_time, self.comm_manager)
            
            # 更新无人机状态，记录避障情况
            self.avoidance_count += drone.update(
                self.dynamic_obstacles if self.dynamic_obstacles_enabled else [], 
                self.grid,
                other_drones=self.drones if self.cluster_mode else None,
                cluster_mode=self.cluster_mode
            )
            
            # 绘制无人机
            drone.draw(self.screen)

    async def main_loop(self):
        """主游戏循环"""
        start_time = time.time()
        running = True
        
        while running:
            current_time = time.time() - start_time
            
            # 处理事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                else:
                    # 将事件传递给handle_mouse_event处理
                    self.handle_mouse_event(event)
            
            # 清空屏幕
            self.screen.fill(WHITE)
            
            # 绘制环境
            self.draw_grid_and_obstacles()
            self.draw_start_goal()
            self.draw_boundary()
            
            # 更新和绘制动态障碍物
            if self.dynamic_obstacles_enabled:
                for obstacle in self.dynamic_obstacles:
                    obstacle.update()
                    obstacle.draw(self.screen)
            
            # 更新和绘制所有无人机
            self.update_drones(current_time)
            
            # 绘制UI
            for button in self.buttons:
                button.draw(self.screen, self.font)
            
            # 绘制信息面板
            self.draw_info_panel()
            
            # 更新显示
            pygame.display.flip()
            self.clock.tick(FPS)
            
            # 异步延迟
            await asyncio.sleep(0)

# 主程序入口
async def main():
    app = SimulationApp()
    await app.main_loop()

# 针对 Windows 平台修复 asyncio
if platform.system() == 'Windows':
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

# 运行主循环
if __name__ == "__main__":
    asyncio.run(main())