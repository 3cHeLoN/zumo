import serial
import socket
import numpy as np
from xbox360controller import Xbox360Controller
from threading import Timer
from time import sleep

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

    def joystick_state():
        x_axis = controller.axis_l.x
        ltrigger = controller.trigger_l.value
        rtrigger = controller.trigger_r.value
        zumo.update_speeds(x_axis, ltrigger, rtrigger)
        Timer(0.01, joystick_state, ()).start()

    try:
        Timer(0.01, joystick_state, ()).start()
        print("Going into for loop")
        while True:
            sleep(1)
            pass
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
