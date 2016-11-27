#!/usr/bin/env python
import sys
import rospy
import time
from modules.sensor_publish import ArduinoMonitor
from modules.PS3_control import PS3Controller

#read in serial port
try:
    serial_port = sys.argv[1]
except IndexError:
    print "enter the serial port!"
    sys.exit(0)

def main():
    rospy.init_node('sensor_publish_and_control', anonymous = True)
    learnerRead = ArduinoMonitor(1, serial_port)
    learnerRead.start()
    controller = PS3Controller()
    rospy.loginfo('joystick vehicle controller starting')

    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
    rospy.loginfo('joystick vehicle controller finished')
    learnerRead.stop()
    learnerRead.join()

if __name__ == '__main__':
    main()

sys.exit(0)
