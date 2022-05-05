import pygame

COLOR_WHITE = pygame.Color(255, 255, 255)
COLOR_BLACK = pygame.Color(0, 0, 0)
COLOR_ACTIVE = pygame.Color('dodgerblue1')
COLOR_INACTIVE = pygame.Color('dodgerblue4')


class InputContrl(object):

    def __init__(self):
        self.mouse_pos = (0, 0)
        self.mouse_offset = [0.0, 0.0]
        self.wheel_offset = 1.0
        self.wheel_amount = 0.05

    def _parse_zoom(self, button):
        if button == 4:
            self.wheel_offset -= self.wheel_amount * 0.5
            if self.wheel_offset >= 1.0:
                self.wheel_offset = 1.0
        elif button == 5:
            self.wheel_offset += self.wheel_amount * 0.5
            if self.wheel_offset <= 0.1:
                self.wheel_offset = 0.1


class Button(object):
    def __init__(self, x, y, w, h, text, callback):
        self.text_surf = pygame.font.Font.render(pygame.font.SysFont("calibri", 20),
                                                 text, True, COLOR_WHITE)
        self.button_rect = pygame.Rect(x, y, w, h)
        self.text_rect = self.text_surf.get_rect(center=self.button_rect.center)

        self.color = COLOR_INACTIVE
        self.callback = callback

    def draw(self, screen):
        pygame.draw.rect(screen, self.color, self.button_rect)
        self.text_rect = self.text_surf.get_rect(center=self.button_rect.center)
        screen.blit(self.text_surf, self.text_rect)

    def reset_text(self, text):
        self.text_surf = pygame.font.Font.render(pygame.font.SysFont("calibri", 20),
                                                 text, True, COLOR_WHITE)

class Text(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=COLOR_WHITE):
        lines = text.split('\n')
        self.surface = pygame.Surface(self.dim)
        self.surface.fill(COLOR_BLACK)
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, color)
            self.surface.blit(text_texture, (22, (n + 0.5) * 22))
        self.surface.set_alpha(220)