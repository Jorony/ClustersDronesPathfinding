import pygame
from config import GRAY, BLACK

class Button:
    """通用按钮控件，处理显示和点击事件"""
    def __init__(self, x, y, w, h, text, callback):
        self.rect = pygame.Rect(x, y, w, h)
        self.text = text
        self.callback = callback
        self.color = GRAY

    def draw(self, screen, font):
        """绘制按钮及其文本"""
        pygame.draw.rect(screen, self.color, self.rect)
        text_surf = font.render(self.text, True, BLACK)
        screen.blit(text_surf, text_surf.get_rect(center=self.rect.center))

    def handle_event(self, event):
        """处理鼠标点击事件，如果点击在按钮范围内则调用回调函数"""
        if (event.type == pygame.MOUSEBUTTONDOWN and 
            event.button == 1 and 
            self.rect.collidepoint(event.pos)):
            self.callback()