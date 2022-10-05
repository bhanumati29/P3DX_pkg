#! /usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
import time

COM_port = '/dev/ttyUSB0'
baud_rate = 57600
timeout_val = 0.1
wait = 0.1
led_state = 'l'

arduino = serial.Serial(COM_port, baud_rate, timeout=timeout_val)
time.sleep(1)

def led_state_cb(msg):
	led_state = msg.data[0];
	if(led_state=='h' or led_state=='l'):
		arduino.write(led_state)

if __name__ == '__main__':
	rospy.init_node('keys_to_led_state')
	rospy.Subscriber('keys', String, led_state_cb)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()

