from time import sleep
import pywinusb.hid as hid
from collections import namedtuple
import timeit
import copy
from pywinusb.hid import usage_pages, helpers, winapi
import socket
import json
import threading
import time

# current version number
__version__ = "0.2.3"

# reloj para medir tiempos
high_acc_clock = timeit.default_timer

GENERIC_PAGE = 0x1
BUTTON_PAGE = 0x9
LED_PAGE = 0x8
MULTI_AXIS_CONTROLLER_CAP = 0x8

HID_AXIS_MAP = {
    0x30: "x",
    0x31: "y",
    0x32: "z",
    0x33: "roll",
    0x34: "pitch",
    0x35: "yaw",
}

import pprint

# el mapeado de los ejes se especifica como:
# [channel, byte1, byte2, scale]; scale es por lo grneral -1 o 1 y multiplica el resultado por este valor
# (pero el escalado per-axis tabien puede ser establecido de porma manual)
# byte1 y byte2 son indices hacia el HID array indicando a los dos bytes que se leen a formar el valor para este eje
# Para el SpaceNavigator, these are consecutive bytes following the channel number.
AxisSpec = namedtuple("AxisSpec", ["channel", "byte1", "byte2", "scale"])

# los estados de los botones son especificados como:
# [channel, data byte,  bit of byte, index to write to]
# Si un mensaje es recibido en el canal especificado, el valordel data byte es establecido en el bit array del boton
ButtonSpec = namedtuple("ButtonSpec", ["channel", "byte", "bit"])


# convertir 2 bytes 8-bit a un int con signo de 16-bit
def to_int16(y1, y2):
    x = (y1) | (y2 << 8)
    if x >= 32768:
        x = -(65536 - x)
    return x


# tuppla para los resultados de los 6GDL
SpaceNavigator = namedtuple(
    "SpaceNavigator", ["t", "x", "y", "z", "roll", "pitch", "yaw", "buttons"]
)


class ButtonState(list):
    def __int__(self):
        return sum((b << i) for (i, b) in enumerate(reversed(self)))


class DeviceSpec(object):
    """Contiene las especificaciones de un solo dispositivo 3Dconnexion"""

    def __init__(
            self, name, hid_id, led_id, mappings, button_mapping, axis_scale=350.0
    ):
        self.name = name
        self.hid_id = hid_id
        self.led_id = led_id
        self.mappings = mappings
        self.button_mapping = button_mapping
        self.axis_scale = axis_scale

        self.led_usage = hid.get_full_usage_id(led_id[0], led_id[1])
        # se inicializa en un vector de reposo "0" para cada estado
        self.dict_state = {
            "t": -1,
            "x": 0,
            "y": 0,
            "z": 0,
            "roll": 0,
            "pitch": 0,
            "yaw": 0,
            "buttons": ButtonState([0] * len(self.button_mapping)),
        }
        self.tuple_state = SpaceNavigator(**self.dict_state)

        # start in disconnected state
        self.device = None
        self.callback = None
        self.button_callback = None

    def describe_connection(self):
        """Devuelve una cadena representativa del dispositivo, incluyendo
        el estado de conexión"""
        if self.device == None:
            return "%s [disconnected]" % (self.name)
        else:
            return "%s connected to %s %s version: %s [serial: %s]" % (
                self.name,
                self.vendor_name,
                self.product_name,
                self.version_number,
                self.serial_number,
            )

    @property
    def connected(self):
        """Si es True el dispositivo ha sido conectado"""
        return self.device is not None

    @property
    def state(self):
        """Devuelve el estado actual de read ()

        Devuelve: state: {t,x,y,z,pitch,yaw,roll,button} namedtuple
                O nada si el dispositivo no está conectado.
        """
        return self.read()

    def open(self):
        """Abre una conexion con el dispositivo, si es posible"""
        if self.device:
            self.device.open()
        # copy in product details
        self.product_name = self.device.product_name
        self.vendor_name = self.device.vendor_name
        self.version_number = self.device.version_number
        # no parece funcionar
        # el numero de serie es una cadena de bytes, las convertimos a hexa
        self.serial_number = "".join(
            ["%02X" % ord(char) for char in self.device.serial_number]
        )

    def set_led(self, state):
        """Establece el estado del LED"""
        if self.connected:
            reports = self.device.find_output_reports()
            for report in reports:
                if self.led_usage in report:
                    report[self.led_usage] = state
                    report.send()

    def close(self):
        """Cierra la conexión si está abierta"""
        if self.connected:
            self.device.close()
            self.device = None

    def read(self):
        """Devuelve el estado actual de este controlador.

        Devuelve:
            state: {t,x,y,z,pitch,yaw,roll,button} namedtuple
            O nada si no está abierto.
        """
        if self.connected:
            return self.tuple_state
        else:
            return None

    def process(self, data):
        """
        Actualiza el estado basado en los datos recibidos

        Esta funcion actualiza el estado del objeto, dan do valores por cada
        eje [x,y,z,roll,pitch,yaw] en un rango de [-1.0, 1.0] teoricamente-> no es cierto, parece que rangos max [-1.5, 1.5]
        El estado de la tupla es establecido solo cuando los 6GDL han sido leidos correctamente

        Si se proporciona un callback, es llamado con una copia del estado de la tupla actual
        Si button_callback, solo se le llama cuando existe un cambio en el estado del boton con los argumentos (state, button_state)

        Parametros:
            data    The data for this HID event, as returned by the HID callback

        """
        button_changed = False

        for name, (chan, b1, b2, flip) in self.mappings.items():
            if data[0] == chan:
                self.dict_state[name] = (
                        flip * to_int16(data[b1], data[b2]) / float(self.axis_scale)
                )

        for button_index, (chan, byte, bit) in enumerate(self.button_mapping):
            if data[0] == chan:
                button_changed = True
                # update the button vector
                mask = 1 << bit
                self.dict_state["buttons"][button_index] = (
                    1 if (data[byte] & mask) != 0 else 0
                )

        self.dict_state["t"] = high_acc_clock()

        # debe de recibir ambas partes del estado de los 6GDL antes de devolver el diccionario de estado
        if len(self.dict_state) == 8:
            self.tuple_state = SpaceNavigator(**self.dict_state)

        # llama cualquier llamada relacionada
        if self.callback:
            self.callback(self.tuple_state)

        # solo llama a la llamada de los botones si el estado de los botones ha cambiado
        if self.button_callback and button_changed:
            self.button_callback(self.tuple_state, self.tuple_state.buttons)


# ////////////////////////////////////////////////////////////////////////////////////
# Los identificadores de los dispositivos de 3Dconnection con los que tiene compatibilidad
# Cada identificador mapea un nombre a un objeto DeviceSpec
device_specs = {
    "SpaceNavigator": DeviceSpec(
        name="SpaceNavigator",
        # vendor ID and product ID
        hid_id=[0x46D, 0xC626],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),
            ButtonSpec(channel=3, byte=1, bit=1),
        ],
        axis_scale=350.0,
    ),
    "SpaceMouse Compact": DeviceSpec(
        name="SpaceMouse Compact",
        # vendor ID and product ID
        hid_id=[0x256F, 0xC635],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),
            ButtonSpec(channel=3, byte=1, bit=1),
        ],
        axis_scale=350.0,
    ),
    "SpaceMouse Pro Wireless": DeviceSpec(
        name="SpaceMouse Pro Wireless",
        # vendor ID and product ID
        hid_id=[0x256F, 0xC632],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
            "roll": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
            "yaw": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),  # MENU
            ButtonSpec(channel=3, byte=3, bit=7),  # ALT
            ButtonSpec(channel=3, byte=4, bit=1),  # CTRL
            ButtonSpec(channel=3, byte=4, bit=0),  # SHIFT
            ButtonSpec(channel=3, byte=3, bit=6),  # ESC
            ButtonSpec(channel=3, byte=2, bit=4),  # 1
            ButtonSpec(channel=3, byte=2, bit=5),  # 2
            ButtonSpec(channel=3, byte=2, bit=6),  # 3
            ButtonSpec(channel=3, byte=2, bit=7),  # 4
            ButtonSpec(channel=3, byte=2, bit=0),  # ROLL CLOCKWISE
            ButtonSpec(channel=3, byte=1, bit=2),  # TOP
            ButtonSpec(channel=3, byte=4, bit=2),  # ROTATION
            ButtonSpec(channel=3, byte=1, bit=5),  # FRONT
            ButtonSpec(channel=3, byte=1, bit=4),  # REAR
            ButtonSpec(channel=3, byte=1, bit=1),
        ],  # FIT
        axis_scale=350.0,
    ),
    # identico pero con el ID 0xc631
    "SpaceMouse Pro Wireless": DeviceSpec(
        name="SpaceMouse Pro Wireless",
        # vendor ID and product ID
        hid_id=[0x256F, 0xC631],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
            "roll": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
            "yaw": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),  # MENU
            ButtonSpec(channel=3, byte=3, bit=7),  # ALT
            ButtonSpec(channel=3, byte=4, bit=1),  # CTRL
            ButtonSpec(channel=3, byte=4, bit=0),  # SHIFT
            ButtonSpec(channel=3, byte=3, bit=6),  # ESC
            ButtonSpec(channel=3, byte=2, bit=4),  # 1
            ButtonSpec(channel=3, byte=2, bit=5),  # 2
            ButtonSpec(channel=3, byte=2, bit=6),  # 3
            ButtonSpec(channel=3, byte=2, bit=7),  # 4
            ButtonSpec(channel=3, byte=2, bit=0),  # ROLL CLOCKWISE
            ButtonSpec(channel=3, byte=1, bit=2),  # TOP
            ButtonSpec(channel=3, byte=4, bit=2),  # ROTATION
            ButtonSpec(channel=3, byte=1, bit=5),  # FRONT
            ButtonSpec(channel=3, byte=1, bit=4),  # REAR
            ButtonSpec(channel=3, byte=1, bit=1),
        ],  # FIT
        axis_scale=350.0,
    ),
    "SpaceMouse Pro": DeviceSpec(
        name="SpaceMouse Pro",
        # vendor ID and product ID
        hid_id=[0x46D, 0xC62b],
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),  # MENU
            ButtonSpec(channel=3, byte=3, bit=7),  # ALT
            ButtonSpec(channel=3, byte=4, bit=1),  # CTRL
            ButtonSpec(channel=3, byte=4, bit=0),  # SHIFT
            ButtonSpec(channel=3, byte=3, bit=6),  # ESC
            ButtonSpec(channel=3, byte=2, bit=4),  # 1
            ButtonSpec(channel=3, byte=2, bit=5),  # 2
            ButtonSpec(channel=3, byte=2, bit=6),  # 3
            ButtonSpec(channel=3, byte=2, bit=7),  # 4
            ButtonSpec(channel=3, byte=2, bit=0),  # ROLL CLOCKWISE
            ButtonSpec(channel=3, byte=1, bit=2),  # TOP
            ButtonSpec(channel=3, byte=4, bit=2),  # ROTATION
            ButtonSpec(channel=3, byte=1, bit=5),  # FRONT
            ButtonSpec(channel=3, byte=1, bit=4),  # REAR
            ButtonSpec(channel=3, byte=1, bit=1),  # FIT
        ],
        axis_scale=350.0,
    ),
    "SpaceMouse Wireless": DeviceSpec(
        name="SpaceMouse Wireless",
        # vendor ID and product ID
        hid_id=[0x256F, 0xC62E],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
            "roll": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
            "yaw": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),  # LEFT
            ButtonSpec(channel=3, byte=1, bit=1),  # RIGHT
        ],  # FIT
        axis_scale=350.0,
    ),
    "3Dconnexion Universal Receiver": DeviceSpec(
        name="3Dconnexion Universal Receiver",
        # vendor ID and product ID
        hid_id=[0x256F, 0xC652],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=1, byte1=7, byte2=8, scale=-1),
            "roll": AxisSpec(channel=1, byte1=9, byte2=10, scale=-1),
            "yaw": AxisSpec(channel=1, byte1=11, byte2=12, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=1, bit=0),  # MENU
            ButtonSpec(channel=3, byte=3, bit=7),  # ALT
            ButtonSpec(channel=3, byte=4, bit=1),  # CTRL
            ButtonSpec(channel=3, byte=4, bit=0),  # SHIFT
            ButtonSpec(channel=3, byte=3, bit=6),  # ESC
            ButtonSpec(channel=3, byte=2, bit=4),  # 1
            ButtonSpec(channel=3, byte=2, bit=5),  # 2
            ButtonSpec(channel=3, byte=2, bit=6),  # 3
            ButtonSpec(channel=3, byte=2, bit=7),  # 4
            ButtonSpec(channel=3, byte=2, bit=0),  # ROLL CLOCKWISE
            ButtonSpec(channel=3, byte=1, bit=2),  # TOP
            ButtonSpec(channel=3, byte=4, bit=2),  # ROTATION
            ButtonSpec(channel=3, byte=1, bit=5),  # FRONT
            ButtonSpec(channel=3, byte=1, bit=4),  # REAR
            ButtonSpec(channel=3, byte=1, bit=1),
        ],  # FIT
        axis_scale=350.0,
    ),
    "SpacePilot Pro": DeviceSpec(
        name="SpacePilot Pro",
        # vendor ID and product ID
        hid_id=[0x46D, 0xC629],
        # LED HID usage code pair
        led_id=[0x8, 0x4B],
        mappings={
            "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
            "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
            "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
            "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
            "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
            "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
        },
        button_mapping=[
            ButtonSpec(channel=3, byte=4, bit=0),  # SHIFT
            ButtonSpec(channel=3, byte=3, bit=6),  # ESC
            ButtonSpec(channel=3, byte=4, bit=1),  # CTRL
            ButtonSpec(channel=3, byte=3, bit=7),  # ALT
            ButtonSpec(channel=3, byte=3, bit=1),  # 1
            ButtonSpec(channel=3, byte=3, bit=2),  # 2
            ButtonSpec(channel=3, byte=2, bit=6),  # 3
            ButtonSpec(channel=3, byte=2, bit=7),  # 4
            ButtonSpec(channel=3, byte=3, bit=0),  # 5
            ButtonSpec(channel=3, byte=1, bit=0),  # MENU
            ButtonSpec(channel=3, byte=4, bit=6),  # -
            ButtonSpec(channel=3, byte=4, bit=5),  # +
            ButtonSpec(channel=3, byte=4, bit=4),  # DOMINANT
            ButtonSpec(channel=3, byte=4, bit=3),  # PAN/ZOOM
            ButtonSpec(channel=3, byte=4, bit=2),  # ROTATION
            ButtonSpec(channel=3, byte=2, bit=0),  # ROLL CLOCKWISE
            ButtonSpec(channel=3, byte=1, bit=2),  # TOP
            ButtonSpec(channel=3, byte=1, bit=5),  # FRONT
            ButtonSpec(channel=3, byte=1, bit=4),  # REAR
            ButtonSpec(channel=3, byte=2, bit=2),  # ISO
            ButtonSpec(channel=3, byte=1, bit=1),  # FIT
        ],
        axis_scale=350.0,
    ),
}

# //////////////////////////////////////////////////////////////////////
# [Para el SpaceNavigator]
# El formato del HID es
# [id, a, b, c, d, e, f]
# cada par (a,b), (c,d), (e,f) es un valor con signo de 16-bit representando el estado absoluto del dispositivo [from -350 to 350]

# si id==1, el mapeo es
# (a,b) = y translation
# (c,d) = x translation
# (e,f) = z translation

# si id==2 el mapeo es
# (a,b) = x tilting (roll)
# (c,d) = y tilting (pitch)
# (d,e) = z tilting (yaw)

# si id==3 el mapeo es
# a = button. Bit 1 = button 1, bit 2 = button 2

# Cada movimiento del dispositivo siempre causa dos eventos de HID
# uno con id 1 y otro con id 2, para generarse un tras otro.


supported_devices = list(device_specs.keys())
_active_device = None


def close():
    """Cierra el dispositivo activo"""
    if _active_device is not None:
        _active_device.close()


def read():
    """Devuelve el estado actual del spacenavigator conectado.

    Devuelve:
        state: {t,x,y,z,pitch,yaw,roll,button} namedtuple
        O nada si no está abierto.
    """
    if _active_device is not None:
        return _active_device.tuple_state
    else:
        return None


def list_devices():
    """Devuelve una lista de dispositivos compatibles conectados

    Devuelve:
        Una lista de los elementos con soporte que sean encontrados.
    """
    devices = []
    all_hids = hid.find_all_hid_devices()
    if all_hids:
        for index, device in enumerate(all_hids):
            for device_name, spec in device_specs.items():
                if (
                        device.vendor_id == spec.hid_id[0]
                        and device.product_id == spec.hid_id[1]
                ):
                    devices.append(device_name)
    return devices


def open(callback=None, button_callback=None, device=None, DeviceNumber=0):
    """
    Abre uno de los dispositivos spcadenavigator. Hace este dispositivo el dispositivo activo actual, lo cual habilita read() u close()
    Para multiples dispositivos usa el read() y close() de los respectivos objetos.

    Parametros:
        callback: Si se provee un callback, es llamado a cada actualizacion de HID con una copia del estado actual de namedtuple
        button_callback: Si button_callback se recibe, es llamado cada vez que se pulsa un boton, con los argumentos (state_tuple, button_state)
        device: nombre del dispositivo que se debe abrir. Deve de poseer los valores de alguno de los dispositivos de la lista.
                si no encuentra, elige el primer dispositivo compatible encontrado.
        DeviceNumber: usa el primer dispositivo (DeviceNumber=0) que encuentre.
    Devuelve:
        Un objeto del dispositivo si se ha abierto correctamente
        O nada si no esta abierto
    """
    # only used if the module-level functions are used
    global _active_device

    # if no device name specified, look for any matching device and choose the first
    if device == None:
        all_devices = list_devices()
        if len(all_devices) > 0:
            device = all_devices[0]
        else:
            return None

    found_devices = []
    all_hids = hid.find_all_hid_devices()
    if all_hids:
        for index, dev in enumerate(all_hids):
            spec = device_specs[device]
            if dev.vendor_id == spec.hid_id[0] and dev.product_id == spec.hid_id[1]:
                found_devices.append({"Spec": spec, "HIDDevice": dev})
                print("%s found" % device)

    else:
        print("No HID devices detected")
        return None

    if len(found_devices) == 0:
        print("No supported devices found")
        return None
    else:

        if len(found_devices) <= DeviceNumber:
            DeviceNumber = 0

        if len(found_devices) > DeviceNumber:
            # crea una copia de las especificaciones del dispositivo
            spec = found_devices[DeviceNumber]["Spec"]
            dev = found_devices[DeviceNumber]["HIDDevice"]
            new_device = copy.deepcopy(spec)
            new_device.device = dev

            # establece los callbacks
            new_device.callback = callback
            new_device.button_callback = button_callback
            # abre el dispositivo y establece el gestionador de datos
            new_device.open()
            dev.set_raw_data_handler(lambda x: new_device.process(x))
            _active_device = new_device
            return new_device

    print("Unknown error occured.")
    return None


def print_state(state):
    # llamada al print en consola
    if state:
        print(
            " ".join(
                [
                    "%4s %+.2f" % (k, getattr(state, k))
                    for k in ["x", "y", "z", "roll", "pitch", "yaw", "t"]
                ]
            )
        )


def toggle_led(state, buttons):
    print("".join(["buttons=", str(buttons)]))
    # Activa los leds si B_1 y los desactiva si B_2
    if buttons[0] == 1:
        set_led(1)
    if buttons[1] == 1:
        set_led(0)


def set_led(state):
    if _active_device:
        _active_device.set_led(state)


# ///////////////////////////////////////////////////
def run_sn_server(address='localhost', port=65432):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((address, port))
    server_socket.listen(1)
    print("Esperando conexion")
    connection, client_address = server_socket.accept()
    print("Conexion de", client_address)

    # joystick = XInputJoystick(0)
    Edmouse = DeviceSpec

    try:
        while True:
            # state = joystick.get_state()
            state = read()
            if state:
                data = {
                # data = json.dumps({
                    'x': state.x,
                    'y': state.y,
                    'z': state.z,
                    'roll': state.roll,
                    'pitch': state.pitch,
                    'yaw': state.yaw,
                    't': state.t,
                    'buttons': list(state.buttons)
                }
                # })
                message = json.dumps(data) + '\n'
                connection.sendall(message.encode('utf-8'))
                # # connection.sendall(json.dumps(data).encode('utf-8'))
                # connection.sendall(data.encode('utf-8'))
            time.sleep(0.01)  # limita frec de envio a 100Hz
    finally:
        connection.close()
        server_socket.close()

# ///////////////////////////////////////////////////

# ///////////////////////////////////////////////////
def run_sn_egm_server(address='localhost', port=65433):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((address, port))
    server_socket.listen(1)
    print("Esperando conexion")
    connection, client_address = server_socket.accept()
    print("Conexion de", client_address)

    # joystick = XInputJoystick(0)
    Edmouse = DeviceSpec

    try:
        while True:
            # state = joystick.get_state()
            state = read()
            if state:
                data = {
                # data = json.dumps({
                    'x': state.x,
                    'y': state.y,
                    'z': state.z,
                    'roll': state.roll,
                    'pitch': state.pitch,
                    'yaw': state.yaw,
                    't': state.t,
                    'buttons': list(state.buttons)
                }
                # })
                message = json.dumps(data) + '\n'
                connection.sendall(message.encode('utf-8'))
                # # connection.sendall(json.dumps(data).encode('utf-8'))
                # connection.sendall(data.encode('utf-8'))
            time.sleep(0.01)  # limita frec de envio a 100Hz
    finally:
        connection.close()
        server_socket.close()

# ///////////////////////////////////////////////////

if __name__ == "__main__":
    # server_thread = threading.Thread(target=run_sn_server)
    # server_thread.start()

    print("Devices found:\n\t%s" % "\n\t".join(list_devices()))
    dev = open(callback=print_state, button_callback=toggle_led)
    print(dev.describe_connection())

    if dev:
        dev.set_led(0)
        server_thread = threading.Thread(target=run_sn_server)
        server_egm = threading.Thread(target=run_sn_egm_server)
        server_thread.start()
        server_egm.start()
        while 1:
            sleep(1)
            dev.set_led(1)
            sleep(1)
            dev.set_led(0)

