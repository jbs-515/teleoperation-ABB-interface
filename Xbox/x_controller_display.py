import socket
import json
import pygame
import sys


# /////////////////////////////////////////////
BUTTON_MAP = {
    0x0001: 'pad_up', 0x0002: 'pad_down', 0x0004: 'pad_left', 0x0008: 'pad_right',
    0x0010: 'start', 0x0020: 'back', 0x0040: 'L3', 0x0080: 'R3',
    0x0100: 'LB', 0x0200: 'RB', 0x1000: 'A', 0x2000: 'B', 0x4000: 'X', 0x8000: 'Y'
}

BUTTON_POSITIONS = {
        0x0001: (200, 400), 0x0002: (200, 500), 0x0004: (150, 450), 0x0008: (250, 450),
        0x0010: (450, 200), 0x0020: (350, 200), 0x0040: (200, 300), 0x0080: (600, 500),
        0x0100: (200, 100), 0x0200: (600, 100), 0x1000: (600, 400), 0x2000: (650, 350),
        0x4000: (550, 350), 0x8000: (600, 300)
    }
LEFT_TRIGGER_POS = (50, 300)
RIGHT_TRIGGER_POS = (700, 300)
LEFT_JOYSTICK_POS = (200, 300)
RIGHT_JOYSTICK_POS = (600, 500)

STATE_POS_CAR = (750, 520)
STATE_POS_POL = (750, 550)

# ////////////////////////////////////////////


def draw_text(screen, text, position, font, color=(255, 255, 255)):
    text_surface = font.render(text, True, color)
    screen.blit(text_surface, position)


def run_display_client(address='localhost', port=5000):
    # pygame.init()
    # screen_width, screen_height = 800, 600
    # screen = pygame.display.set_mode((screen_width, screen_height))

    # Establecer la conexión
    client_socket_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket_.connect((address, port))
    print("conexión de servidor")
    return client_socket_


# //////////////////////////////////////////
def update_display(screen_, joystick_state):
    # Lógica para dibujar los botones y joysticks basada en el estado recibido
    screen.fill((0, 0, 0))
    for button, position in BUTTON_POSITIONS.items():
        # el grupo BUTTON_POSITIONS tiene dos elementos en un orden y en el for establezco que representan en ese orden
        color = (0, 255, 0) if joystick_state['buttons'] & button else (255, 255, 255)
        pygame.draw.circle(screen_, color, position, 20)
        draw_text(screen_, BUTTON_MAP[button], (position[0] + 25, position[1] - 10), font)

    draw_trigger(screen_, joystick_state['left_trigger'], LEFT_TRIGGER_POS)
    draw_trigger(screen_, joystick_state['right_trigger'], RIGHT_TRIGGER_POS)

    draw_joystick(screen_, joystick_state['l_thumb_x'], joystick_state['l_thumb_y'], LEFT_JOYSTICK_POS)
    draw_joystick(screen_, joystick_state['r_thumb_x'], joystick_state['r_thumb_y'], RIGHT_JOYSTICK_POS)

    draw_tog(screen_, joystick_state['cartesian_mode'], STATE_POS_CAR)
    draw_tog(screen_, not joystick_state['cartesian_mode'], STATE_POS_POL)

    pygame.display.update()

    # pass  # Aquí iría el código para dibujar basado en joystick_state


def draw_trigger(screen_, value, position):
    pygame.draw.rect(screen_, (255, 255, 255), (*position, 20, 100))
    pygame.draw.rect(screen_, (0, 255, 0), (*position, 20, int(100 * value)))
    if position == LEFT_TRIGGER_POS:
        draw_text(screen_, 'LT', (position[0], position[1] - 20), font)
    else:
        draw_text(screen_, 'RT', (position[0], position[1] - 20), font)


def draw_joystick(screen_, x_value, y_value, position):
    pygame.draw.circle(screen_, (255, 255, 255), position, 40, 1)
    pygame.draw.circle(screen_, (0, 255, 0),
                       (position[0] + int(40 * x_value), position[1] - int(40 * y_value)), 10)


def draw_tog(screen_, state_, position):
    color = (0, 255, 0) if state_ else (255, 255, 255)
    pygame.draw.rect(screen_, color, (*position, 20, 20))


if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    font = pygame.font.Font(None, 24)
    # run_display_client()
    client_socket = run_display_client()

    buffer = ""
    try:
        while True:
            data = client_socket.recv(4096).decode('utf-8')  # antes recv() era 1024
            if not data:
                break
            buffer += data
            while '\n' in buffer:
                message, buffer = buffer.split('\n', 1)
                state = json.loads(message)
                update_display(screen, state)
            # state = json.loads(data.decode('utf-8'))
            # update_display(screen, state)
    
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
    
            # pygame.time.delay(10)
    finally:
        client_socket.close()
