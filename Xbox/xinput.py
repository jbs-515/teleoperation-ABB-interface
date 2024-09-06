#!/usr/bin/env python

"""
Un modulo para obtener los datos de entrada de un controlador Xbox por medio de
la libreria XInput de Windows http://msdn.microsoft.com/en-gb/library/windows/desktop/ee417001%28v=vs.85%29.aspx


Adaptado de 'Jason R. Coombs' codigo aquí:
http://pydoc.net/Python/jaraco.input/1.0.1/jaraco.input.win32.xinput/
under the MIT licence terms
Y de 'r4dian' codigo aquí:
https://github.com/r4dian/Xbox-Controller-for-Python/tree/master

Implementado en Python 3
Modificado para añadir señales de seguimiento y disponer de conexión TCP
para su comunicación con otras aplicaciones
Solo requiere Pyglet 1.2alpha1 o superior:
pip install --upgrade http://pyglet.googlecode.com/archive/tip.zip
"""

import ctypes
import sys
import time
from operator import itemgetter, attrgetter
from itertools import count, starmap
from pyglet import event
import json
import socket
import threading


# biblioteca de eventos
class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ('buttons', ctypes.c_ushort),  # wButtons
        ('left_trigger', ctypes.c_ubyte),  # bLeftTrigger
        ('right_trigger', ctypes.c_ubyte),  # bLeftTrigger
        ('l_thumb_x', ctypes.c_short),  # sThumbLX
        ('l_thumb_y', ctypes.c_short),  # sThumbLY
        ('r_thumb_x', ctypes.c_short),  # sThumbRx
        ('r_thumb_y', ctypes.c_short),  # sThumbRy
    ]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [
        ('packet_number', ctypes.c_ulong),  # dwPacketNumber
        ('gamepad', XINPUT_GAMEPAD),  # Gamepad
    ]


class XINPUT_VIBRATION(ctypes.Structure):
    _fields_ = [("wLeftMotorSpeed", ctypes.c_ushort),
                ("wRightMotorSpeed", ctypes.c_ushort)]

class XINPUT_BATTERY_INFORMATION(ctypes.Structure):
    _fields_ = [("BatteryType", ctypes.c_ubyte),
                ("BatteryLevel", ctypes.c_ubyte)]


xinput = ctypes.windll.xinput1_4
# xinput = ctypes.windll.xinput9_1_0  # this is the Win 8 version ?
# xinput1_2, xinput1_1 (32-bit Vista SP1)
# xinput1_3 (64-bit Vista SP1)


def struct_dict(struct):
    """
    take a ctypes.Structure and return its field/value pairs
    as a dict.

    >>> 'buttons' in struct_dict(XINPUT_GAMEPAD)
    True
    >>> struct_dict(XINPUT_GAMEPAD)['buttons'].__class__.__name__
    'CField'
    """
    get_pair = lambda field_type: (
        field_type[0], getattr(struct, field_type[0]))
    return dict(list(map(get_pair, struct._fields_)))


def get_bit_values(number, size=32):
    """
    (Obtiene los valores de bit como una lista para un numero dado)
    Get bit values as a list for a given number

    >>> get_bit_values(1) == [0]*31 + [1]
    True

    >>> get_bit_values(0xDEADBEEF)
    [1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1]

    (Se puede sobrepasar el tamaño estandar de 32-bits para ajustarse a
    tu aplicación)
    You may override the default word size of 32-bits to match your actual
    application.
    >>> get_bit_values(0x3, 2)
    [1, 1]

    >>> get_bit_values(0x3, 4)
    [0, 0, 1, 1]
    """
    res = list(gen_bit_values(number))
    res.reverse()
    # 0-pad the MSB
    res = [0] * (size - len(res)) + res
    return res


def gen_bit_values(number):
    """
    Devuelve cero o uno por cada bit de un valor numerico hasta el MSB, empezando por el LSB.
    Return a zero or one for each bit of a numeric value up to the most
    significant 1 bit, beginning with the least significant bit.
    """
    number = int(number)
    while number:
        yield number & 0x1
        number >>= 1


ERROR_DEVICE_NOT_CONNECTED = 1167
ERROR_SUCCESS = 0


class XInputJoystick(event.EventDispatcher):

    """
    XInputJoystick

    Un gestor de estados, usa el modelo de eventos pyglet, conecta con un dispositivo
    y analiza los eventos cuando el estado cambia.
    .

    Example:
    controller_one = XInputJoystick(0)
    """
    max_devices = 4

    def __init__(self, device_number, normalize_axes=True):
        values = vars()
        del values['self']
        self.__dict__.update(values)

        super(XInputJoystick, self).__init__()

        self._last_state = self.get_state()
        self.received_packets = 0
        self.missed_packets = 0

        # Establece el método que se empleará para la normalización.
        choices = [self.translate_identity, self.translate_using_data_size]
        self.translate = choices[normalize_axes]
        # ///////////////////////
        self.cartesian_mode = True

    def translate_using_data_size(self, value, data_size):
        # normaliza los datos anaógicos en un rango de [0, 1] para los rangos estrictamente positivos
        #  y [-0.5,0.5] para los datos con simbolo
        data_bits = 8 * data_size
        return float(value) / (2 ** data_bits - 1)

    def translate_identity(self, value, data_size=None):
        return value

    def get_state(self):
        "Obtiene el estado del controlador representado por este objeto"
        state = XINPUT_STATE()
        res = xinput.XInputGetState(self.device_number, ctypes.byref(state))
        if res == ERROR_SUCCESS:
            return state
        if res != ERROR_DEVICE_NOT_CONNECTED:
            raise RuntimeError(
                "Unknown error %d attempting to get state of device %d" % (res, self.device_number))
        # else return None (no hay dispositivo conectado)

    def is_connected(self):
        return self._last_state is not None

    @staticmethod
    def enumerate_devices():
        "Devuelve el dato de # de dispositivos conectados"
        devices = list(
            map(XInputJoystick, list(range(XInputJoystick.max_devices))))
        return [d for d in devices if d.is_connected()]

    def set_vibration(self, left_motor, right_motor):
        "Controla la velocidad de  ambos motores independientemente"
        # Set up function argument types and return type
        XInputSetState = xinput.XInputSetState
        XInputSetState.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_VIBRATION)]
        XInputSetState.restype = ctypes.c_uint

        vibration = XINPUT_VIBRATION(
            int(left_motor * 65535), int(right_motor * 65535))
        XInputSetState(self.device_number, ctypes.byref(vibration))

    def get_battery_information(self):
        "Detecta el tipo de batería y el nivel de carga"
        BATTERY_DEVTYPE_GAMEPAD = 0x00
        BATTERY_DEVTYPE_HEADSET = 0x01
        XInputGetBatteryInformation = xinput.XInputGetBatteryInformation
        XInputGetBatteryInformation.argtypes = [ctypes.c_uint, ctypes.c_ubyte, ctypes.POINTER(XINPUT_BATTERY_INFORMATION)]
        XInputGetBatteryInformation.restype = ctypes.c_uint

        battery = XINPUT_BATTERY_INFORMATION(0,0)
        XInputGetBatteryInformation(self.device_number, BATTERY_DEVTYPE_GAMEPAD, ctypes.byref(battery))

        #define BATTERY_TYPE_DISCONNECTED       0x00
        #define BATTERY_TYPE_WIRED              0x01
        #define BATTERY_TYPE_ALKALINE           0x02
        #define BATTERY_TYPE_NIMH               0x03
        #define BATTERY_TYPE_UNKNOWN            0xFF
        #define BATTERY_LEVEL_EMPTY             0x00
        #define BATTERY_LEVEL_LOW               0x01
        #define BATTERY_LEVEL_MEDIUM            0x02
        #define BATTERY_LEVEL_FULL              0x03
        batt_type = "Unknown" if battery.BatteryType == 0xFF else ["Disconnected", "Wired", "Alkaline","Nimh"][battery.BatteryType]
        level = ["Empty", "Low", "Medium", "Full"][battery.BatteryLevel]
        return batt_type, level

    def dispatch_events(self):
        "Bucle principal de eventos de joystick"
        state = self.get_state()
        if not state:
            raise RuntimeError(
                "Joystick %d is not connected" % self.device_number)
        if state.packet_number != self._last_state.packet_number:
            # si el estado cambia lo maneja
            self.update_packet_count(state)
            self.handle_changed_state(state)
        self._last_state = state

    def update_packet_count(self, state):
        "Hace seguimiento de los paquetes recibidos y perdidos (modificación del rendimiento)"
        self.received_packets += 1
        missed_packets = state.packet_number - \
            self._last_state.packet_number - 1
        if missed_packets:
            self.dispatch_event('on_missed_packet', missed_packets)
        self.missed_packets += missed_packets

    def handle_changed_state(self, state):
        "Maneja varios eventos resultantes de cambios de estados"
        self.dispatch_event('on_state_changed', state)
        self.dispatch_axis_events(state)
        self.dispatch_button_events(state)

    def dispatch_axis_events(self, state):
        # axis fields se refieren a tod0 menos los botones
        axis_fields = dict(XINPUT_GAMEPAD._fields_)
        axis_fields.pop('buttons')
        for axis, type in list(axis_fields.items()):
            old_val = getattr(self._last_state.gamepad, axis)
            new_val = getattr(state.gamepad, axis)
            data_size = ctypes.sizeof(type)
            old_val = self.translate(old_val, data_size)
            new_val = self.translate(new_val, data_size)

            # establece zonas muertas: minimas
            # ultimos ajustes probados 18/08/24
            if ((old_val != new_val and (new_val > 0.08000000000000000 or new_val < -0.08000000000000000) and abs(old_val - new_val) > 0.00000000500000000) or
               (axis == 'right_trigger' or axis == 'left_trigger') and new_val == 0 and abs(old_val - new_val) > 0.00000000500000000):
                self.dispatch_event('on_axis', axis, new_val)

    def dispatch_button_events(self, state):
        changed = state.gamepad.buttons ^ self._last_state.gamepad.buttons
        changed = get_bit_values(changed, 16)
        buttons_state = get_bit_values(state.gamepad.buttons, 16)
        changed.reverse()
        buttons_state.reverse()
        button_numbers = count(1)
        changed_buttons = list(
            filter(itemgetter(0), list(zip(changed, button_numbers, buttons_state))))
        tuple(starmap(self.dispatch_button_event, changed_buttons))


    def dispatch_button_event(self, changed, number, pressed):
        self.dispatch_event('on_button', number, pressed)
        ''
        # /////////////////////// (este parece que funciona mejor 8/8/24)
        if number == 6 and pressed:
            self.cartesian_mode = not self.cartesian_mode
            print(f"Modo cambiado a {'cartesiano' if self.cartesian_mode else 'polar'}")
        ''

    # stub para manejar los eventos
    def on_state_changed(self, state):
        pass

    def on_axis(self, axis, value):
        pass

    def on_button(self, button, pressed):
        pass

    def on_missed_packet(self, number):
        pass


list(map(XInputJoystick.register_event_type, [
    'on_state_changed',
    'on_axis',
    'on_button',
    'on_missed_packet',
]))


def determine_optimal_sample_rate(joystick=None):
    """
    Acciona lentamente la palanca, monitoriza el flujo de paquetes para
    obsevar si hay perdida de paquetes, indicando que el ratio de
    muestreo es demasiado lento para los paquetes perdidos. Esto supone
    información perdida para actualizar el estado del joystick.
    Al detectar perdidas de paquetes, incrementa el ratio de muestreo
    hasta que se haya llegado al estado deseado.
    """
    # recomendado probar entre  200-2000Hz
    if joystick is None:
        joystick = XInputJoystick.enumerate_devices()[0]

    j = joystick

    print("Move the joystick or generate button events characteristic of your app")
    print("Hit Ctrl-C or press button 6 (<, Back) to quit.")


    # epezar a 1Hz y aumentar el ratio hasta que los menajes perdidos son eliminados
    j.probe_frequency = 1  # Hz
    j.quit = False
    j.target_reliability = .99  # perder 1 mensaje de 100 es aceptable

    @j.event
    def on_button(button, pressed):
        # flag the process to quit if the < button ('back') is pressed.
        j.quit = (button == 6 and pressed)

    @j.event
    def on_missed_packet(number):
        print('missed %(number)d packets' % vars())
        total = j.received_packets + j.missed_packets
        reliability = j.received_packets / float(total)
        if reliability < j.target_reliability:
            j.missed_packets = j.received_packets = 0
            j.probe_frequency *= 1.5

    while not j.quit:
        j.dispatch_events()
        time.sleep(1.0 / j.probe_frequency)
    print("final probe frequency was %s Hz" % j.probe_frequency)


def sample_first_joystick():
    """
    Utiliza el primer mando disponible, mostrando los cambios
    que se realicen en él en la consola. Los triggers LT y RT
    activan los motores de vibración
    """
    joysticks = XInputJoystick.enumerate_devices()
    device_numbers = list(map(attrgetter('device_number'), joysticks))

    print('found %d devices: %s' % (len(joysticks), device_numbers))

    if not joysticks:
        sys.exit(0)

    j = joysticks[0]
    print('using %d' % j.device_number)

    battery = j.get_battery_information()
    print(battery)

    @j.event
    def on_button(button, pressed):
        print('button', button, pressed)

    left_speed = 0
    right_speed = 0

    @j.event
    def on_axis(axis, value):
        left_speed = 0
        right_speed = 0

        print('axis', axis, value)
        if axis == "left_trigger":
            left_speed = value
        elif axis == "right_trigger":
            right_speed = value
        j.set_vibration(left_speed, right_speed)

    while True:
        j.dispatch_events()
        time.sleep(.01)


# ///////////////////////////////////////////////////
def run_xinput_server(address='localhost', port=5000):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((address, port))
    server_socket.listen(1)
    print("Esperando conexion")
    connection, client_address = server_socket.accept()
    print("Conexion de", client_address)

    joystick = XInputJoystick(0)

    try:
        while True:
            joystick.dispatch_events()  # para leer a tiempo real los eventos internos del joystick
            # (como el cartesian_mode)
            state = joystick.get_state()
            if state:
                data = {
                    'buttons': state.gamepad.buttons,
                    'left_trigger': state.gamepad.left_trigger / 255.0,
                    'right_trigger': state.gamepad.right_trigger / 255.0,
                    'l_thumb_x': state.gamepad.l_thumb_x / 32767.0,
                    'l_thumb_y': state.gamepad.l_thumb_y / 32767.0,
                    'r_thumb_x': state.gamepad.r_thumb_x / 32767.0,
                    'r_thumb_y': state.gamepad.r_thumb_y / 32767.0,
                    'cartesian_mode': joystick.cartesian_mode
                }
                message = json.dumps(data) + '\n'
                connection.sendall(message.encode('utf-8'))
                # # connection.sendall(json.dumps(data).encode('utf-8'))
                # connection.sendall(data.encode('utf-8'))
            time.sleep(0.01)  # limita frec de envio a 100Hz
    finally:
        connection.close()
        server_socket.close()


# ///////////////////////////////////////////////////
# Servidor/cliente EGM
def run_xinput_server_to_egm(address='localhost', port=5001):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    server_socket.bind((address, port))
    server_socket.listen(1)
    print("Esperando conexion con egm_prog")

    while True:
        connection, client_address = server_socket.accept()
        print("Conexion de", client_address)
        threading.Thread(target=handle_client, args=(connection,), daemon=True).start()


def handle_client(connection):
    joystick = XInputJoystick(0)

    try:
        while True:
            joystick.dispatch_events()  # para leer a tiempo real los eventos internos del joystick
            # (como el cartesian_mode)
            state = joystick.get_state()
            if state:
                data = {
                    'buttons': state.gamepad.buttons,
                    'left_trigger': state.gamepad.left_trigger / 255.0,
                    'right_trigger': state.gamepad.right_trigger / 255.0,
                    'l_thumb_x': state.gamepad.l_thumb_x / 32767.0,
                    'l_thumb_y': state.gamepad.l_thumb_y / 32767.0,
                    'r_thumb_x': state.gamepad.r_thumb_x / 32767.0,
                    'r_thumb_y': state.gamepad.r_thumb_y / 32767.0,
                    'cartesian_mode': joystick.cartesian_mode
                }
                message = json.dumps(data) + '\n'
                connection.sendall(message.encode('utf-8'))
                # # connection.sendall(json.dumps(data).encode('utf-8'))
                # connection.sendall(data.encode('utf-8'))
            time.sleep(0.01)  # limita frec de envio a 100Hz
    finally:
        connection.close()
        # server_socket.close()

# ////////////////////////////////////////////////////////

if __name__ == "__main__":

    server_thread = threading.Thread(target=run_xinput_server)
    ABB_thread = threading.Thread(target=run_xinput_server_to_egm, daemon=True)

    server_thread.start()
    ABB_thread.start()

    sample_first_joystick()

