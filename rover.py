import serial
import socket
import numpy as np
from xbox360controller import Xbox360Controller
from threading import Timer

class SerialConnection:

    def __init__(self, device):
        self.ser = serial.Serial(port=device, baudrate=9600)

    def send(self, msg_string):
        msg_string += '\n'
        self.ser.write(msg_string.encode('utf-8'))

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
        self.connection = connection
        self.max_speed = 400
        self.max_angle = 75 / 180 * np.pi
        self._last_lspeed = 0
        self._last_rspeed = 0

    def set_speeds(self, left_speed, right_speed):
        if left_speed == self._last_lspeed and right_speed == self._last_rspeed:
            return
        self._last_lspeed = left_speed
        self._last_rspeed = right_speed
        msg_string = "{} {}".format(
            left_speed,
            right_speed)
        self.connection.send(msg_string)

    def update_speeds(self, x_axis, ltrigger, rtrigger):
        speed = int((rtrigger - ltrigger) * self.max_speed)
        if x_axis < -0.2:
            if speed == 0:
                l_speed = int(x_axis * self.max_speed)
                r_speed = -l_speed
            else:
                l_speed = int(np.cos(x_axis * self.max_angle) * speed)
                r_speed = speed
        elif x_axis < 0.2:
            l_speed = speed
            r_speed = speed
        else:
            if speed == 0:
                l_speed = int(x_axis * self.max_speed)
                r_speed = -l_speed
            else:
                l_speed = speed
                r_speed = int(np.cos(x_axis * self.max_angle) * speed)
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
    conn = SerialConnection('/dev/ttyUSB0')
    zumo = ZumoControl(conn)
    controller = Xbox360Controller(0, axis_threshold=0.2)
    controller.button_trigger_l.when_pressed = zumo.honk
    controller.button_trigger_l.when_released = zumo.dehonk
    controller.button_trigger_r.when_pressed = zumo.low_honk
    controller.button_trigger_r.when_released = zumo.dehonk
    controller.button_a.when_pressed = zumo.blink
    controller.button_a.when_released = zumo.deblink
    #sd = sched.scheduler(time.time, time.sleep)
    data_conn = IPConnection("10.42.0.227", 8888)

    def joystick_state():
        x_axis = controller.axis_l.x
        ltrigger = controller.trigger_l.value
        rtrigger = controller.trigger_r.value
        zumo.update_speeds(x_axis, ltrigger, rtrigger)
        #sd.enter(0.01, 1, joystick_state, (sc,))
        Timer(0.01, joystick_state, ()).start()

    def two_complement(value):
        sign = value & (1 << 7)
        return -1 ** sign * (value & 0x7F)

    def unpack(data_str):
        data = {}
        data['timestamp'] = int.from_bytes(data_str[0:2],
                                           byteorder='little',
                                           signed=False)
        data['encoder_left'] = int.from_bytes(data_str[2:4],
                                              byteorder='little',
                                              signed=True)
        data['encoder_right'] = int.from_bytes(data_str[4:6],
                                               byteorder='little',
                                               signed=True)
        data['accel_x'] = int.from_bytes(data_str[6:8],
                                         byteorder='little',
                                         signed=True)
        data['accel_y'] = int.from_bytes(data_str[8:10],
                                         byteorder='little',
                                         signed=True)
        data['accel_z'] = int.from_bytes(data_str[10:12],
                                         byteorder='little',
                                         signed=True)
        data['gyro_x'] = int.from_bytes(data_str[12:14],
                                        byteorder='little',
                                        signed=True)
        data['gyro_y'] = int.from_bytes(data_str[14:16],
                                        byteorder='little',
                                        signed=True)
        data['gyro_z'] = int.from_bytes(data_str[16:18],
                                        byteorder='little',
                                        signed=True)
        return data

    try:
        #sd.enter(0.01, 1, joystick_state, (sd,))
        #sd.run(blocking=False)
        Timer(0.01, joystick_state, ()).start()
        print("Going into for loop")
        while True:
            data = data_conn.recv()
            print(data)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
