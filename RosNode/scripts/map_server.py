#!/usr/bin/env python
import sys
import rospy
import time
import tf
import numpy as np
from threading import Thread, Semaphore
from sensor_msgs.msg import Range
from math import pi, radians
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, OccupancyGrid

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
        origin = Pose()

        self._base_link_id = "learner/base_link"
        self._map = OccupancyGrid()
        self._map.header.frame_id = "map"
        self._map.info.map_load_time = rospy.Time.now()
        self._map.info.resolution = 0.02
        self._map.info.width = 300
        self._map.info.height = 300
        origin.position.x = (self._map.info.width * self._map.info.resolution) /-2.0
        origin.position.y = (self._map.info.height * self._map.info.resolution) /-2.0
        origin.position.z = 0
        origin.orientation.x = 0
        origin.orientation.y = 0
        origin.orientation.z = 0
        origin.orientation.w = 1
        self._map.info.origin = origin
        self._map.data = ([-1] * 300) * 300 #np.ones((2,2),dtype=np.int8)#((self._map.info.width,self._map.info.height))
#       self._map.data *= -1
        ultrasonic_sensors = str(rospy.get_param("~mapping_sensors")).split(",")

        for sensor in ultrasonic_sensors:
            rospy.Subscriber(sensor, Range, self.ultrasonicCallback)
        rospy.Subscriber("learner/odom", Odometry, self.updateRobotPosition)
        self._map_publisher = rospy.Publisher('learner/map',OccupancyGrid,queue_size = 1)
        self._map_publisher.publish(self._map)
        self._update_map.acquire()

    def run(self):
        while self._run:
            self._map_publisher.publish(self._map)
            time.sleep(1)
#           if self._update_map.acquire():
#               if not(self._run):
#                   break
#               pass#put map update here

    def close(self):
        self._run = False  
        self._update_map.release()

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
