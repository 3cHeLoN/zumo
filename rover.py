from threading import Thread
from io import StringIO
from typing import Callable
import argparse
import serial
import socket
import numpy as np
import inputs
from enum import auto, Enum


class SerialConnection:

    def __init__(self, device):
        self.ser = serial.Serial(port=device, baudrate=9600)
        self._encoding = 'utf-8'

    @property
    def encoding(self):
        return self._encoding

    @encoding.setter
    def encoding(self, value):
        self._encoding = value

    def send(self, msg_string):
        #msg_string += '\n'
        self.ser.write(msg_string.encode(self._encoding))
        print(msg_string.encode(self._encoding))

    def recv(self):
        response = None
        if self.ser.in_waiting:
            # read single line
            response = self.ser.read_until()
        return response

    def available(self):
        return self.ser.in_waiting

    def read_until(self):
        return self.ser.read_until()


class IPConnection:

    def __init__(self, ip_address, port):
        self.ip_address = ip_address
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((ip_address, port))
        self.data_left = None

    def send(self, msg_string):
        msg_string += '\n'
        self.socket.send(msg_string.encode('utf-8'))

    def recv(self):
        raw_data = ''
        if self.data_left:
            raw_data = self.data_left
            self.data_left = None

        while '\n' not in raw_data:
            raw_data += self.socket.recv(1024).decode('utf-8')

        return raw_data


class ZumoState(Enum):
    NEUTRAL = auto()
    HONK1_START = auto()
    HONK2_START = auto()
    HONK_STOP = auto()
    ENABLE_LEDS = auto()
    DISABLE_LEDS = auto()
    CALIBRATE_LINE_FOLLOWER = auto()
    TOGGLE_LINE_FOLLOW = auto()


class ZumoControl:

    state: ZumoState
    state_map = {
        ZumoState.HONK1_START: 241,
        ZumoState.HONK2_START: 242,
        ZumoState.HONK_STOP: 243,
        ZumoState.ENABLE_LEDS: 244,
        ZumoState.DISABLE_LEDS: 245,
        ZumoState.CALIBRATE_LINE_FOLLOWER: 246,
        ZumoState.TOGGLE_LINE_FOLLOW: 247
    }

    def __init__(self, connection):
        super().__init__()
        self.connection = connection
        self.max_speed = 120
        self.max_angle = 75 / 180 * np.pi
        self._last_lspeed = 0
        self._last_rspeed = 0

        self.deadzone = 20
        self.ltrigger = 0
        self.rtrigger = 0
        self.x_axis = 0

    def speed_to_bitvalue(self, speed):
        # map speed between 0 and 240
        return speed + self.max_speed

    def set_speeds(self, left_speed, right_speed):
        if (left_speed == self._last_lspeed and right_speed == self._last_rspeed):
            return

        self.connection.send(chr(self.speed_to_bitvalue(left_speed))
                             + chr(self.speed_to_bitvalue(right_speed)) + '\n')

        self._last_lspeed = left_speed
        self._last_rspeed = right_speed

    def set_ltrigger(self, value):
        self.ltrigger = (value * (value > self.deadzone)) / 255
        self.update_speeds()

    def set_rtrigger(self, value):
        self.rtrigger = (value * (value > self.deadzone)) / 255
        self.update_speeds()

    def set_xaxis(self, value):
        self.x_axis = (value * (abs(value) > self.deadzone)) / (2**15)
        self.update_speeds()

    def update_state(self, state):
        state_speed = self.state_map[state]
        self.connection.send(chr(state_speed) + chr(state_speed) + '\n')

    def update_speeds(self):
        speed = int((self.rtrigger - self.ltrigger) * self.max_speed)
        if speed == 128:
            speed = 127

        # rotate in-place
        if speed == 0:
            l_speed = int(self.x_axis * self.max_speed)
            r_speed = -l_speed
            if abs(r_speed == 128):
                r_speed = np.sign(r_speed) * 127
        # steer
        elif self.x_axis > 0:
            l_speed = speed
            r_speed = int(np.cos(self.x_axis * self.max_angle) * speed)
        elif self.x_axis < 0:
            l_speed = int(np.cos(self.x_axis * self.max_angle) * speed)
            r_speed = speed
        else:
            l_speed = speed
            r_speed = speed

        self.set_speeds(l_speed, r_speed)


class DataWriter(Thread):

    """Write text in a separate thread."""

    def __init__(self, filename: str) -> None:
        super().__init__()
        self.filename = filename
        self.text_block = ''

    def set_text_block(self, text_block):
        self.text_block = text_block

    def reset(self):
        with open(self.filename, 'w', encoding='utf-8') as file:
            file.writelines('')

    def run(self):
        with open(self.filename, 'a', encoding='utf-8') as file:
            file.writelines(self.text_block)


class DataRecorder(Thread):

    buffer: StringIO

    def __init__(self, ip_connection: IPConnection, data_writer: DataWriter = None, callback: Callable = None):
        super().__init__()
        self.ip_connection = ip_connection
        self.stopped = False
        self.writer = data_writer
        self.writer.start()
        self.buffer = StringIO()
        self.buffer_size = 4096
        self.callback = callback

    def stop(self):
        self.stopped = True

    def run(self):
        while not self.stopped:
            raw_data = self.ip_connection.recv()
            if self.callback:
                self.callback(self.unpack(raw_data))

            if self.writer:
                self.buffer.writelines(raw_data)
                if self.buffer.tell() > self.buffer_size:
                    self.buffer.seek(0)
                    self.writer.set_text_block(self.buffer.readlines())
                    self.writer.run()
                    self.buffer.flush()

    def unpack(self, raw_data):
        data_values = [int(x) for x in raw_data.split()]
        keys = ['timestamp',
                'encoder_left', 'encoder_right',
                'accel_x', 'accel_y', 'accel_z',
                'gyro_x', 'gyro_y', 'gyro_z',
                'packet_count']
        data = {key: data_point for key, data_point in zip(keys, data_values)}
        return data

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('--outfile', default=None)
    args = parser.parse_args()

    conn = SerialConnection('/dev/ttyUSB0')
    ip_conn = IPConnection("192.168.4.1", 8888)

    #conn = SerialConnection('COM4')
    conn.encoding = 'latin-1'

    writer = None
    if args.outfile:
        writer = DataWriter(args.outfile)
        writer.reset()

    zumo = ZumoControl(conn)
    recorder = DataRecorder(ip_conn, data_writer=writer)
    recorder.start()

    running = True

    # start the event loop
    while running:

        for event in inputs.get_gamepad():
            if event.ev_type == 'Key':
                if event.code == 'BTN_TL' and event.state == 1:
                    zumo.update_state(ZumoState.HONK1_START)
                elif event.code == 'BTN_TR' and event.state == 1:
                    zumo.update_state(ZumoState.HONK2_START)
                elif event.code in ['BTN_TL', 'BTN_TR'] and event.state == 0:
                    zumo.update_state(ZumoState.HONK_STOP)
                elif event.code == 'BTN_SOUTH' and event.state == 1:
                    zumo.update_state(ZumoState.TOGGLE_LINE_FOLLOW)
                elif event.code == 'BTN_EAST' and event.state == 1:
                    zumo.update_state(ZumoState.CALIBRATE_LINE_FOLLOWER)
                elif event.code == 'BTN_NORTH' and event.state == 1:
                    zumo.update_state(ZumoState.ENABLE_LEDS)
                elif event.code == 'BTN_NORTH' and event.state == 0:
                    zumo.update_state(ZumoState.DISABLE_LEDS)

            if event.ev_type == 'Absolute':
                if event.code == 'ABS_RZ':
                    zumo.set_rtrigger(event.state)
                if event.code == 'ABS_Z':
                    zumo.set_ltrigger(event.state)
                if event.code == 'ABS_X':
                    zumo.set_xaxis(event.state)

            print(event.ev_type, event.code, event.state)


if __name__ == '__main__':
    main()
