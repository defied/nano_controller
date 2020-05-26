#!/usr/bin/env python  
''' 
Use /joy commands to manually operate sabertooth 2x32 motors over usb.
'''

import rospy
import serial
import time
from sensor_msgs.msg import Joy


class sabreControl:
    def __init__(self):
        rospy.init_node('sabertooth_controller')
        rospy.loginfo("sabertooth controller node: initializing")
        self.ser = serial.Serial(
            port='/dev/ttyACM1',
            baudrate=115200,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        self.m1 = 0
        self.m2 = 0
        self.m1_message = 'M1:0'
        self.m2_message = 'M2:0'
        self.m1_message_last = 'M1:0'
        self.m2_message_last = 'M2:0'
        self.axes_left_stick = 0
        self.axes_right_stick = 0

        self.timer_1    =   time.time()
        self.timer_2    =   time.time()

        rospy.Subscriber('/joy', Joy, self.joy_to_serial_callback, queue_size = 1)
         
        self.rate           =   rospy.Rate(100)
        
        rospy.loginfo("sabertooth controller node: finished initializing")
    
    # Collect and publish values from sabertooth controllers.
    def joy_to_serial_callback(self, msg): 
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
        if self.m1_message not in self.m1_message_last:
            rospy.loginfo(self.ser.isOpen())
            rospy.loginfo(self.m1_message)
            self.ser.write(self.m1_message)
        if self.m2_message not in self.m2_message_last:
            rospy.loginfo(self.m2_message)
            self.ser.write(self.m2_message)
        self.m1_message_last = self.m1_message
        self.m2_message_last = self.m2_message

    def timeout(self):
        if self.time_1 > self.time_2:
    
    def run(self):
        rospy.loginfo('sabertooth controllers reading')
        rospy.loginfo('Serial Open: {}'.format(self.ser.isOpen()))
        while not rospy.is_shutdown():
            if not self.ser.isOpen():
                rospy.loginfo('Serial Connection not open! Exiting.')
                rospy.signal_shutdown('No Serial Connection Found.')
            else:
                self.rate.sleep()
        self.ser.close()

if __name__ == "__main__":
    sc = sabreControl()
    sc.run()
