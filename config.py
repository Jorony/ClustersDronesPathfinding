# 全局配置文件

# =========================
# 颜色定义
# =========================
BLUE   = (0, 0, 255)       # 蓝色（无人机1/路径/起点等）
PURPLE = (128, 0, 128)     # 紫色（备用/可用于特殊状态）
RED    = (255, 0, 0)       # 红色（动态障碍物/失败轨迹）
GREEN  = (0, 255, 0)       # 绿色（避障成功轨迹）
WHITE  = (255, 255, 255)   # 白色（背景/集群指示器）
BLACK  = (0, 0, 0)         # 黑色（边界/障碍物/文字）
YELLOW = (255, 255, 0)     # 黄色（终点区域）
GRAY   = (200, 200, 200)   # 灰色（信息面板/按钮/速度条底色）
CYAN   = (0, 255, 255)     # 青色（无人机3）
ORANGE = (255, 165, 0)     # 橙色（无人机2）

# =========================
# 界面与网格设置
# =========================
GRID_SIZE = 150            # 网格边长（格子数）
CELL_SIZE = 5              # 单个格子的像素大小
FPS = 60                   # 帧率
INFO_LINE_HEIGHT = 20      # 信息面板每行高度
INFO_START_Y = GRID_SIZE * CELL_SIZE - 200  # 信息面板起始Y坐标
INFO_START_X = 10          # 信息面板起始X坐标
INFO_WIDTH = 650           # 信息面板宽度
SPEED_INDICATOR_WIDTH = 50  # 速度条宽度（像素）
SPEED_INDICATOR_HEIGHT = 5  # 速度条高度（像素）

# =========================
# 边界设置
# =========================
BOUNDARY_START = (2, 2)    # 可行区域左上角（含）
BOUNDARY_END = (105, 125)  # 可行区域右下角（含）
BOUNDARY_LINE_THICKNESS = 2 # 边界线宽度

# =========================
# 无人机设置
# =========================
DRONE_COLORS = [BLUE, ORANGE, CYAN]  # 多无人机颜色
DRONE_START_POSITIONS = [            # 无人机起始点（可修改）
    (10, 30),
    (25, 25),
    (50, 45)
]
DEFAULT_ALGORITHM = 'A*'             # 默认路径规划算法

# 无人机物理特性
DEFAULT_SPEED = 1.5                  # 无人机基础速度
DEFAULT_INERTIA = 0.9                # 速度惯性系数（越大越平滑）
DEFAULT_MAX_SPEED = 2                # 最大速度
DEFAULT_MAX_ACCEL = 0.5              # 最大加速度

# 避障设置
DEFAULT_OBS_AVOID_WEIGHT = 2         # 静态障碍物避障权重
DEFAULT_OBS_DETECT_RADIUS = 2        # 静态障碍物检测半径
DEFAULT_SENSOR_ERROR_PROB = 0.2      # 避障传感器误报概率
DEFAULT_BASE_INFLUENCE_RADIUS = 6    # 动态障碍物人工势场基础影响半径
DEFAULT_BASE_SPEED_THRESHOLD = 0.3   # 动态障碍物人工势场速度阈值

# 集群设置
DEFAULT_MIN_CLUSTER_DIST = 3         # 集群最小间距
DEFAULT_MAX_CLUSTER_DIST = 6         # 集群最大间距
DEFAULT_CLUSTER_ATTRACT = 0.8        # 集群吸引力权重
DEFAULT_CLUSTER_REPEL = 1.2          # 集群排斥力权重
CLUSTER_MODE_ENABLED = True          # 是否启用集群模式

# 通信设置
DEFAULT_BROADCAST_INTERVAL = 0.1     # 通信广播间隔（秒）
DEFAULT_POSITION_TIMEOUT = 0.5       # 通信位置信息超时时间（秒）
COMMUNICATION_LATENCY_RANGE = (0.1, 0.5)  # 通信延迟范围（秒，最小值, 最大值）
PACKET_LOSS_PROBABILITY = 0.1             # 通信丢包概率（0~1）

# =========================
# 动态障碍物设置
# =========================
DYNAMIC_OBSTACLE_COUNT = 10          # 动态障碍物数量
DYNAMIC_OBSTACLES_ENABLED = True     # 是否启用动态障碍物

# =========================
# 终点设置
# =========================
GOAL_POS = (100, 120)                # 终点坐标
GOAL_RADIUS = 5                      # 终点判定半径（格子）
DEFAULT_GOAL_RADIUS = GOAL_RADIUS    # 兼容drone.py

# =========================
# 交互设置
# =========================
BRUSH_SIZE = 5                       # 障碍物刷子大小
DEFAULT_MODE = 'place'               # 默认模式（'place'放置障碍，'remove'移除障碍）
