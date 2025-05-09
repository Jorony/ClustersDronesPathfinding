import pygame
import random
import math
from config import RED, BOUNDARY_START, BOUNDARY_END

class DynamicObstacle:
    """动态障碍物类，模拟随机移动障碍物并计算对无人机的影响"""
    def __init__(self, grid_size, cell_size):
        # 基本属性
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.pos = [random.randint(0, grid_size - 1), random.randint(0, grid_size - 1)]
        self.size = random.randint(1, 3)  # 障碍物大小，影响显示和避障难度
        
        # 势场和影响属性
        self.influence_radius = 5  # 基础影响半径
        self.repulsion_strength = 4  # 排斥力强度
        
        # 运动属性
        self.direction = [random.choice([-1, 1]), random.choice([-1, 1])]
        self.speed = random.uniform(0.5, 1)
        self.stop_probability = 0.02  # 每帧随机停止的概率
        self.change_direction_probability = 0.05  # 每帧随机改变方向的概率
        
        # 动态影响因子
        self.dynamic_influence_factor = 1.0  # 当前影响因子
        self.prev_influence_factor = 1.0  # 上一帧影响因子
        self.max_influence_change = 0.1  # 每帧最大影响因子变化

    def update(self):
        """更新障碍物位置和影响因子"""
        # 随机停止
        if random.random() < self.stop_probability:
            return
            
        # 随机改变方向
        if random.random() < self.change_direction_probability:
            self.direction = [random.choice([-1, 0, 1]), random.choice([-1, 0, 1])]
            # 确保至少朝一个方向移动
            if self.direction[0] == 0 and self.direction[1] == 0:
                self.direction[random.randint(0, 1)] = random.choice([-1, 1])
            self.speed = random.uniform(0.5, 1)
            
        # 更新位置
        self.pos[0] += self.direction[0] * self.speed
        self.pos[1] += self.direction[1] * self.speed
        
        # 处理边界碰撞
        self._handle_boundary_collision()
        
        # 更新影响因子
        target_influence = random.uniform(0.9, 1.3)
        change = target_influence - self.prev_influence_factor
        # 限制每帧影响因子变化量
        if abs(change) > self.max_influence_change:
            change = math.copysign(self.max_influence_change, change)
        self.dynamic_influence_factor = self.prev_influence_factor + change
        self.prev_influence_factor = self.dynamic_influence_factor

    def _handle_boundary_collision(self):
        """处理边界碰撞，确保障碍物在边界内，并在碰撞时反弹"""
        min_x, min_y = BOUNDARY_START
        max_x, max_y = BOUNDARY_END
        
        # X轴边界检查与反弹
        if self.pos[0] < min_x:
            self.pos[0] = min_x
            self.direction[0] *= -1
        elif self.pos[0] > max_x:
            self.pos[0] = max_x
            self.direction[0] *= -1
            
        # Y轴边界检查与反弹
        if self.pos[1] < min_y:
            self.pos[1] = min_y
            self.direction[1] *= -1
        elif self.pos[1] > max_y:
            self.pos[1] = max_y
            self.direction[1] *= -1

    def draw(self, screen):
        """在屏幕上绘制障碍物"""
        pygame.draw.circle(screen, RED,
                          (int(self.pos[1] * self.cell_size), int(self.pos[0] * self.cell_size)),
                          self.size * self.cell_size // 2)

    def get_influence(self, drone_pos):
        """计算障碍物对给定无人机位置的影响力和方向"""
        dx, dy = self.pos[0] - drone_pos[0], self.pos[1] - drone_pos[1]
        dist = math.hypot(dx, dy)
        
        # 计算有效影响半径(考虑动态因子)
        effective_radius = self.influence_radius * self.dynamic_influence_factor
        
        if dist < effective_radius:
            # 计算影响系数(离障碍物越近，影响越大)
            influence_factor = 1 - (dist / effective_radius)
            influence = self.repulsion_strength * (influence_factor ** 2)
            
            # 计算影响方向(远离障碍物)
            if dist > 0:
                return influence, (dx / dist, dy / dist)
            
            # 如果距离为0，随机一个方向
            angle = random.uniform(0, 2 * math.pi)
            return influence, (math.cos(angle), math.sin(angle))
            
        # 超出影响范围，无影响
        return 0, (0, 0)
