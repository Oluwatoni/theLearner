#!/usr/bin/env python
import sys
import rospy
import time
import tf
import numpy as np
from threading import Thread, Semaphore
from sensor_msgs.msg import Range
from math import sqrt, atan, cos, sin, pi, radians
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
        self._listener = tf.TransformListener()

        self._position = []

        origin = Pose()

        self._base_link_id = "learner/base_link"
        self._map = OccupancyGrid()
        self._map.header.frame_id = "/map"
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
        self._map.data = ([-1] * self._map.info.width) * self._map.info.height
        self._car_radius = 0.12 / self._map.info.resolution

        #TODO the robot starts from the origin fix this
        self._old_position = np.matrix([[0],
                                    [0],
                                    [0]])

        self._ray_angle = atan((1.5 * self._map.info.resolution)/ 2.5)
        ultrasonic_sensors = str(rospy.get_param("~mapping_sensors")).split(",")

        for sensor in ultrasonic_sensors:
            rospy.Subscriber(sensor, Range, self.ultrasonicCallback)
        rospy.Subscriber("learner/odom", Odometry, self.updateRobotPosition)
        self._map_publisher = rospy.Publisher('learner/map',OccupancyGrid,queue_size = 1)

        self._update_map.acquire()
        self._map_publisher.publish(self._map)
        self._update_map.release()

    def run(self):
        self.robotSpace()
        while self._run:
            self._map_publisher.publish(self._map)
            time.sleep(1)
#           if self._update_map.acquire():
#               if not(self._run):
#                   break
#               pass#put map update here

    def close(self):
        self._run = False

    def robotSpace(self):
        self._update_map.acquire()
        #ray tracing from 0 to 359 degrees
        neg_bound = -int((pi / self._ray_angle) )
        pos_bound = -neg_bound
        list_of_used_cells = []
        max_rad = self._car_radius

        #TODO convert the nested for loops below into some kind of arc filling function
        for theta in range(neg_bound, pos_bound,1):
            for r in range(int(max_rad/self._map.info.resolution), 1, -1):
                #center point of the boundary of the scan
                r = r * self._map.info.resolution
                x_p = int(self._map.info.width/2 + r*cos(theta*self._ray_angle))
                y_p = int(self._map.info.height/2 + r*sin(theta*self._ray_angle))
                
                if not((x_p,y_p) in list_of_used_cells):
                    self._map.data[y_p * self._map.info.width + x_p] = 0
                    list_of_used_cells.append((x_p,y_p))

        self._update_map.release()
        pass

    def updateRobotPosition(self, data):
        self._position_mutex.acquire()
        (roll,pitch,yaw) = euler_from_quaternion([float(data.pose.pose.orientation.x), float(data.pose.pose.orientation.y), float(data.pose.pose.orientation.z), float(data.pose.pose.orientation.w)])
        self._position = np.matrix([[data.pose.pose.position.x],
                                    [data.pose.pose.position.y],
                                    [yaw]])
        
        if sqrt(((self._position[0] - self._old_position[0])**2) + ((self._position[1] - self._old_position[1])**2)) >=  self._map.info.resolution:
            self.robotSpace()

        #the robot is represented by a circle
        self._old_position = self._position
        self._position_mutex.release()

    def ultrasonicCallback(self, data):
        now = rospy.Time(0)
        #TODO replace the base_link below with the map to get the global transform
        self._listener.waitForTransform(self._map.header.frame_id, data.header.frame_id, now ,rospy.Duration(0.25))
        (trans,rot) = self._listener.lookupTransform(self._map.header.frame_id, data.header.frame_id, now)

        self._position_mutex.acquire()
        pose = np.matrix([[0],[0],[0]])#self._position
        self._position_mutex.release()
        (roll,pitch,yaw) = euler_from_quaternion([rot[0],rot[1], rot[2], rot[3]])

        #set the cell at the source of the ray to not occupied
        x = int(trans[0]/self._map.info.resolution + self._map.info.width/2)
        y = int(trans[1]/self._map.info.resolution + self._map.info.height/2)

        self._map.data[y * self._map.info.width + x] = -100

        #set the boundary of the scan on the map
        r = 1/ self._map.info.resolution #data.range/self._map.info.resolution
        
        #angle of iteration
        neg_bound = -int((data.field_of_view / self._ray_angle) / 2)
        pos_bound = -neg_bound

        list_of_used_cells = []

        self._update_map.acquire()
        for theta in range(neg_bound, pos_bound,1):
            #center point of the boundary of the scan
            x_p = int(x + r*cos(yaw+(theta*self._ray_angle)))
            y_p = int(y + r*sin(yaw+(theta*self._ray_angle)))

            if not((x_p,y_p) in list_of_used_cells):
                self._map.data[y_p * self._map.info.width + x_p] = -100
                list_of_used_cells = []
                list_of_used_cells.append((x_p,y_p))

        self._update_map.release()

if __name__ == '__main__':
    main()

sys.exit(0)
