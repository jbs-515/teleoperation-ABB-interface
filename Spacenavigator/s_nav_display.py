import socket
import json
import pygame
import sys
import threading

# /////////////////////////////////////////////
BUTTON_MAP = {
    0x0001: 'B_1', 0x0002: 'B_2'
}

BUTTON_POSITIONS = {
    0x0001: (100, 20), 0x0002: (100, 280)
}
Z_TRIGGER_POS = (135, 200)
YAW_TRIGGER_POS = (385, 175)
LEFT_JOYSTICK_POS = (100, 500)
RIGHT_JOYSTICK_POS = (300, 500)

# ////////////////////////////////////////////


def run_display_client(address='localhost', port=65432):
    # pygame.init()
    # screen_width, screen_height = 800, 600
    # screen = pygame.display.set_mode((screen_width, screen_height))
    # pygame.display.set_caption("Display de Controlador Xbox")

    # Establecer la conexión
    client_socket_ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket_.connect((address, port))
    print("conexión de servidor")
    return client_socket_


# //////////////////////////////////////////
def update_display(screen_, joystick_state):
    # Lógica para dibujar los botones y joysticks basada en el estado recibido
    screen.fill((0, 0, 0))

    # dibujar botones
    for i, button in enumerate(state['buttons']):
        # el grupo BUTTON_POSITIONS tiene dos elementos en un orden y en el for establezco que representan en ese orden
        color = (0, 0, 255) if button else (255, 255, 255)
        pygame.draw.circle(screen_, color, (250 * (1 + i), 100), 20)
        # draw_text(screen_, BUTTON_MAP[button], (250 * (1 + i) - 10, 100 - 10), font)

    # dibujo de barras
    draw_bar(screen_, joystick_state['z'], Z_TRIGGER_POS, "Z", horizontal=False)
    draw_bar(screen_, joystick_state['yaw'], YAW_TRIGGER_POS, "YAW", horizontal=True)

    # dibujo de etiquetas
    pygame.draw.line(screen_, (255, 255, 255), (265 - 40, 300), (265 + 40, 300))
    pygame.draw.line(screen_, (255, 255, 255), (265, 300 - 40), (265, 300 + 40))
    pygame.draw.line(screen_, (255, 255, 255), (485 - 40, 300), (485 + 40, 300))
    pygame.draw.line(screen_, (255, 255, 255), (485, 300 - 40), (485, 300 + 40))
    x_label = font.render("X", True, (255, 255, 255))
    y_label = font.render("Y", True, (255, 255, 255))
    screen_.blit(x_label, (265 + 50, 300))
    screen_.blit(y_label, (265, 300 - 50))
    pitch_label = font.render("ROLL", True, (255, 255, 255))
    roll_label = font.render("PITCH", True, (255, 255, 255))
    screen_.blit(pitch_label, (485 + 50, 300))
    screen_.blit(roll_label, (485, 300 - 50))

    #joysticks
    pygame.draw.circle(screen_, (255, 255, 255), (265, 300), 100, 1)
    pygame.draw.circle(screen_, (0, 0, 200), (int(265 + state['x'] * 80), int(300 - state['y'] * 80)), 10)

    pygame.draw.circle(screen_, (255, 255, 255), (485, 300), 100, 1)
    pygame.draw.circle(screen_, (0, 0, 200), (int(485 + state['roll'] * 80), int(300 - state['pitch'] * 80)), 10)

    pygame.display.update()


def draw_bar(screen_, value, position, label, horizontal=False):
    label_text = font.render(label, True, (255, 255, 255))
    if horizontal:
        pygame.draw.rect(screen_, (255, 255, 255), (*position, 200, 20), 1)
        if value >= 0:
            pygame.draw.rect(screen_, (0, 0, 255), (position[0] + 100, position[1], int(value * 100), 20))
        else:
            pygame.draw.rect(screen_, (255, 0, 255), (position[0] + 100 + int(value * 100), position[1], -int(value * 100), 20))
        screen_.blit(label_text, (position[0] + 200, position[1] + 25))
    else:
        pygame.draw.rect(screen_, (255, 255, 255), (*position, 20, 220), 1)
        if value >= 0:
            pygame.draw.rect(screen_, (0, 0, 255), (position[0], position[1] + 100 - int(value * 100), 20, int(value * 100)))
        else:
            pygame.draw.rect(screen_, (255, 0, 255), (position[0], position[1] + 100, 20, -int(value * 100)))
        screen_.blit(label_text, (position[0] - 20, position[1]))


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
