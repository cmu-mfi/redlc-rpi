#!/usr/bin/env python3
import rospy
import serial

port_name = '/dev/ttyUSB0'

def set_dtr():
    ser = serial.Serial(port=port_name, baudrate=921600, timeout=100)
    ser.dtr = False
    rospy.loginfo('Turning off the Signboard!')
    ser.close()

if __name__ == '__main__':
    rospy.init_node('control_signboard')
    rospy.loginfo('Turning on the Signboard!')
    ser = serial.Serial(port=port_name, baudrate=921600, timeout=100)
    ser.dtr = True
    rospy.on_shutdown(set_dtr)
    rospy.spin()

