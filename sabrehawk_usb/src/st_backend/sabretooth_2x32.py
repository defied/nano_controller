#!/usr/bin/env python

import serial
import time
from sensor_msgs.msg import Joy


class sabreControl:
    def __init__(self, port):
        self.t_bias             =   0.125
        self.m1                 =   0
        self.m2                 =   0
        self.m1_message         =   'M1:0'
        self.m2_message         =   'M2:0'
        self.m1_message_last    =   'M1:0'
        self.m2_message_last    =   'M2:0'
        self.axes_left_stick    =   0
        self.axes_right_stick   =   0

        self.timer_1            =   time.time()
        self.timer_2            =   time.time()

        self.port               =   port
    
    # Collect and publish values from sabertooth controllers.
    def motion_command(self, msg): 
        self.axes_left_stick = msg.axes[1] * 1000
        self.axes_right_stick = msg.axes[4] * 1000
        self.axes_left_trigger = msg.axes[2]
        self.m1 = self.axes_left_stick
        self.m2 = self.axes_right_stick

        if self.axes_left_trigger < -0.4:
            self.m1_message = 'M1:{}\r\n'.format(int(self.m1))
            self.m2_message = 'M2:{}\r\n'.format(int(self.m2))
        else:
            self.m1_message = 'M1:0'
            self.m2_message = 'M2:0'
        self.m1_message_last = self.m1_message
        self.m2_message_last = self.m2_message
        self.timer_1 = time.time()

    def timeout(self):
        if self.time_1 > self.time_2 + self.t_bias:
            self.serial_cmd(m1=0, m2=0)

    def serial_cmd(self, m1=0, m2=0):
        self.m1_message = 'M1:{}\r\n'.format(m1)
        self.m2_message = 'M2:{}\r\n'.format(m2)

    def run(self):
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        return ser.isopen()

    def stop(self):
        self.ser.close()

if __name__ == "__main__":
    sc = sabreControl('/dev/ttyACM1')
    sc.run()
