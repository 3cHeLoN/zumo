import serial
import numpy as np
import signal
from xbox360controller import Xbox360Controller


class Connection:

    def __init__(self, device):
        self.ser = serial.Serial(port=device, baudrate=9600)

    def send(self, msg_string):
        msg_string += '\n'
        self.ser.write(msg_string.encode('utf-8'))

    def recv(self, msg_string):
        response = None
        if self.ser.in_waiting:
            # read single line
            response = self.ser.read_until()
        return response


class ZumoControl:

    def __init__(self, connection):
        self.connection = connection
        self.max_speed = 400
        self.l_speed = 0
        self.r_speed = 0
        self.forward_speed = 0
        self.backward_speed = 0
        self.max_angle = 75 
        self.angle = 0

    def set_speeds(self, left_speed, right_speed):
        msg_string = "{} {}".format(
            left_speed,
            right_speed)
        self.connection.send(msg_string)

    def update_speeds(self):
        speed = self.forward_speed + self.backward_speed
        if self.angle < 0:
            self.l_speed = int(np.cos(self.angle) * speed)
            self.r_speed = speed
        else:
            self.l_speed = speed
            self.r_speed = int(np.cos(self.angle) * speed)
        self.set_speeds(self.l_speed,
                        self.r_speed)

    def stop(self):
        self.set_speeds(0, 0)

    def ltrigger_move(self, axis):
        print("ltrigger moved")
        self.backward_speed = -int(axis.value * self.max_speed)
        self.update_speeds()

    def rtrigger_move(self, axis):
        print("rtrigger moved")
        self.forward_speed = int(axis.value * self.max_speed)
        self.update_speeds()

    def axisl_move(self, axis):
        print(axis.x)
        self.angle = int(axis.x * self.max_angle)
        if np.abs(axis.x) < 0.3:
            self.angle = 0
        print("Left axis moved")
        self.update_speeds()

    def axisr_move(self, axis):
        print("Right axis moved")


def main():
    conn = Connection('/dev/ttyUSB0')
    zumo = ZumoControl(conn)
    try:
        with Xbox360Controller(0, axis_threshold=0.2) as controller:
            controller.trigger_l.when_moved = zumo.ltrigger_move
            controller.trigger_r.when_moved = zumo.rtrigger_move
            controller.axis_l.when_moved = zumo.axisl_move
            controller.axis_r.when_moved = zumo.axisr_move

            signal.pause()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
