#!/usr/bin/env python
import sys
import rospy
import time
import tf
import numpy as np
from threading import Thread
from sensor_msgs.msg import Range
from math import pi, radians
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry, OccupancyGrid

#read in serial port
global ultrasonic_sensors
try:
    ultrasonic_sensors = sys.argv[1,:]
except IndexError:
    print "provide the sensors"
    sys.exit(0)

def main():
    rospy.init_node('map_server', anonymous = True)
    rospy.loginfo('map_server starting')
    sensor_update_thread = SensorUpdateThread()
    sensor_update_thread.start()
    
    rospy.spin()
    sensor_update_thread.close()
    sensor_update_thread.join()

class SensorUpdateThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self._run = True
        self._update_map = Semaphore()
        self._map_mutex = Semaphore()
        self._position_mutex = Semaphore()
        self._t = tf.Transformer(True, rospy.Duration(20.0))

        self._sensor_position = [0,0]
        self._sensor_orientation = 0
        self._base_link_id = "learner/base_link"

        for sensor in ultrasonic_sensors:
            rospy.Subscriber(sensor, range, self.updateMap)
        rospy.Subscriber("learner/odom", Odometry, self.updateRobotPosition)
        self._update_map.acquire()

    def run(self):
        while self._run:
            if self._update_map.acquire():
                if not(self._run):
                    break
                pass#put map update here

    def close(self):
        self._run = False  
        self._update_step_ready.release()

    def updateRobotPosition(self, data):
        self._position_mutex.acquire()
        (roll,pitch,yaw) = euler_from_quaternion([float(data.pose.pose.orientation.x), float(data.pose.pose.orientation.y), float(data.pose.pose.orientation.z), float(data.pose.pose.orientation.w)])
        self._position = np.matrix([[data.pose.pose.position.x],
                                    [data.pose.pose.position.y],
                                    [yaw]])
        self._position_mutex.release()

    def ultrasonicCallback(self, data):
        print data.header.frame_id
        sensor_transform = self._t.lookupTransform(data.header.frame_id, self._base_link_id)
        self._position_mutex.acquire()
        pose[:] = self._position
        self._position_mutex.release()
        (roll,pitch,yaw) = euler_from_quaternion([sensor_transform[1][0], sensor_transform[1][1], sensor_transform[1][2], sensor_transform[1][2]])
        
        self._update_map.release()

if __name__ == '__main__':
    main()

sys.exit(0)
