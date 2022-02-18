import time

import serial
import socket
import numpy as np
import inputs
from threading import Event, Thread
from time import sleep


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

        '''
        while len(raw_data) < 18:
            raw_data = raw_data + self.socket.recv(100)

        if len(raw_data) > 18:
            self.data_left = raw_data[18:]
        raw_data = raw_data[:18]

        # unpack
        timestamp = (raw_data[0] << 8) + raw_data[1]
        count_left = (raw_data[2] << 8) # + raw_data[3]
        count_right = (raw_data[4] << 8) #+ raw_data[5]
        accel_x = np.int16((raw_data[6] << 8) + raw_data[7])
        accel_y = np.int16((raw_data[8] << 8) + raw_data[9])
        accel_z = np.int16((raw_data[10] << 8) + raw_data[11])
        gyro_x = np.int16((raw_data[12] << 8) + raw_data[13])
        gyro_y = np.int16((raw_data[14] << 8) + raw_data[15])
        gyro_z = np.int16((raw_data[16] << 8) + raw_data[17])
        print(timestamp,
              count_left, count_right,
              accel_x, accel_y, accel_z,
              gyro_x, gyro_y, gyro_z)
        '''

        return [int(x) for x in raw_data.split()]


class ZumoControl:

    def __init__(self, connection):
        super().__init__()
        self.connection = connection
        self.max_speed = 128
        self.max_angle = 75 / 180 * np.pi
        self._last_lspeed = 0
        self._last_rspeed = 0

        self.deadzone = 20
        self.ltrigger = 0
        self.rtrigger = 0
        self.x_axis = 0

    def set_speeds(self, left_speed, right_speed):
        if (left_speed == self._last_lspeed and right_speed == self._last_rspeed):
            return

        self.connection.send(chr(left_speed + self.max_speed) + chr(right_speed + self.max_speed) + '\n')
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

    def honk(self, button):
        self.set_speeds(500, 500)

    def low_honk(self, button):
        self.set_speeds(502, 502)

    def dehonk(self, button):
        self.set_speeds(501, 501)

    def blink(self, button):
        self.set_speeds(504, 504)

    def deblink(self, button):
        self.set_speeds(503, 503)


def main():
    #conn = SerialConnection('/dev/ttyUSB0')
    conn = SerialConnection('COM4')
    conn.encoding = 'latin-1'

    zumo = ZumoControl(conn)

    running = True

    # start the event loop
    while running:

        for event in inputs.get_gamepad():
            if event.ev_type == 'Key':
                if event.code == 'BTN_SOUTH' and event.state == 1:
                    print('Button south down')
                print(event.code)
            if event.ev_type == 'Absolute':
                if event.code == 'ABS_RZ':
                    zumo.set_rtrigger(event.state)
                if event.code == 'ABS_Z':
                    zumo.set_ltrigger(event.state)
                if event.code == 'ABS_X':
                    zumo.set_xaxis(event.state)

            print(event.ev_type, event.code, event.state)

    # try:
    #     with Xbox360Controller(0, axis_threshold=0.2) as controller:
    #         zumo = ZumoControl(conn, controller, stop_flag)
    #         zumo.start()
    #         signal.pause()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     stop_flag.set()



    # # data_conn = IPConnection("10.42.0.227", 8888)
    #
    # def joystick_state():
    #     x_axis = controller.axis_l.x
    #     ltrigger = controller.trigger_l.value
    #     rtrigger = controller.trigger_r.value
    #     zumo.update_speeds(x_axis, ltrigger, rtrigger)
    #     #sd.enter(0.01, 1, joystick_state, (sc,))
    #     Timer(0.01, joystick_state, ()).start()
    #     print(x_axis, ltrigger, rtrigger)
    #
    # def two_complement(value):
    #     sign = value & (1 << 7)
    #     return -1 ** sign * (value & 0x7F)
    #
    # def unpack(data_str):
    #     data = {}
    #     data['timestamp'] = int.from_bytes(data_str[0:2],
    #                                        byteorder='little',
    #                                        signed=False)
    #     data['encoder_left'] = int.from_bytes(data_str[2:4],
    #                                           byteorder='little',
    #                                           signed=True)
    #     data['encoder_right'] = int.from_bytes(data_str[4:6],
    #                                            byteorder='little',
    #                                            signed=True)
    #     data['accel_x'] = int.from_bytes(data_str[6:8],
    #                                      byteorder='little',
    #                                      signed=True)
    #     data['accel_y'] = int.from_bytes(data_str[8:10],
    #                                      byteorder='little',
    #                                      signed=True)
    #     data['accel_z'] = int.from_bytes(data_str[10:12],
    #                                      byteorder='little',
    #                                      signed=True)
    #     data['gyro_x'] = int.from_bytes(data_str[12:14],
    #                                     byteorder='little',
    #                                     signed=True)
    #     data['gyro_y'] = int.from_bytes(data_str[14:16],
    #                                     byteorder='little',
    #                                     signed=True)
    #     data['gyro_z'] = int.from_bytes(data_str[16:18],
    #                                     byteorder='little',
    #                                     signed=True)
    #     return data
    #
    # try:
    #     #sd.enter(0.01, 1, joystick_state, (sd,))
    #     #sd.run(blocking=False)
    #     Timer(0.01, joystick_state, ()).start()
    #     print("Going into for loop")
    #     while True:
    #         pass
    #         # data = data_conn.recv()
    #         # print(data)
    # except KeyboardInterrupt:
    #     pass


if __name__ == '__main__':
    main()
