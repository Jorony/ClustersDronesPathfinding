from collections import deque
import random

class CommunicationManager:
    """模拟通信延迟与丢包的管理器，处理无人机间的消息传递"""
    def __init__(self, latency_range=(0.1, 0.3), packet_loss_prob=0.1):
        self.latency_range = latency_range
        self.packet_loss_prob = packet_loss_prob
        self.speed_reduction_factor = 0.7
        self.max_latency = latency_range[1]
        self.message_queue = {}  # 接收者ID -> 消息队列

    def get_latency(self):
        """获取随机通信延迟"""
        return random.uniform(*self.latency_range)

    def is_packet_lost(self):
        """基于丢包概率判断消息是否丢失"""
        return random.random() < self.packet_loss_prob

    def get_speed_factor(self, latency):
        """基于延迟计算速度因子(0-1)，延迟越大速度越慢"""
        if latency <= 0:
            return 1.0
        # 线性映射延迟到速度因子
        return max(0, 1.0 - ((1.0 - self.speed_reduction_factor) * (latency / self.max_latency)))

    def send_message(self, sender_id, receiver_id, message, current_time):
        """发送消息到指定接收者，考虑丢包和延迟"""
        if self.is_packet_lost():
            return None
            
        latency = self.get_latency()
        arrival_time = current_time + latency
        
        # 初始化接收者的消息队列
        if receiver_id not in self.message_queue:
            self.message_queue[receiver_id] = deque()
            
        self.message_queue[receiver_id].append({
            'sender_id': sender_id,
            'message': message,
            'arrival_time': arrival_time
        })
        
        return arrival_time

    def receive_messages(self, receiver_id, current_time):
        """接收所有已到达的消息"""
        if receiver_id not in self.message_queue:
            return []
            
        messages = []
        queue = self.message_queue[receiver_id]
        
        # 提取所有已到达的消息
        while queue and queue[0]['arrival_time'] <= current_time:
            messages.append(queue.popleft())
            
        return messages