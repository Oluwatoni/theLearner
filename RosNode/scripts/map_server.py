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
        self._max_occupancy = 0.75
        self._scan_tolerance = 0.025
        self._map.info.origin = origin
        self._map.data = ([50] * self._map.info.width) * self._map.info.height
        self._car_radius = 0.12

        #TODO the robot starts from the origin fix this
        self._old_position = np.matrix([[0],
                                        [0],
                                        [0]])

        ultrasonic_sensors = str(rospy.get_param("~mapping_sensors")).split(",")

        for sensor in ultrasonic_sensors:
            rospy.Subscriber(sensor, Range, self.ultrasonicCallback)
        rospy.Subscriber("learner/odom", Odometry, self.updateRobotPosition)
        self._map_publisher = rospy.Publisher('learner/map',OccupancyGrid,queue_size = 1)

        self._update_map.acquire()
        self._map_publisher.publish(self._map)
        self._update_map.release()

    def run(self):
      #  self.fillCells(1,0, (pi/6), 0*(pi/6), 0.5)
        self.fillCells(0,0, (2*pi), 0, self._car_radius)
        self._map.data[280 * self._map.info.width + 150] = 127 
        while self._run:
            self._update_map.acquire()
            self._map_publisher.publish(self._map)
            self._update_map.release()
            time.sleep(1)

    def close(self):
        self._run = False
    
    def odds(self,x): 
        return ((x / (1.0001-x), x / (1-x))[x == 1.0])

    def robotUpdateCellProb(self, m_occ, cell):
        if m_occ == None:
            self._map.data[cell[1] * self._map.info.width + cell[0]] = max(1,min(99,((0)/(1+(0))) * 100))
        else:
            try:
                last_p_occ = self._map.data[cell[1] * self._map.info.width + cell[0]] / 100.0
            except IndexError:
                return
            occ = self.odds(m_occ) * self.odds(last_p_occ)
            self._map.data[cell[1] * self._map.info.width + cell[0]] = max(1,min(99,((occ)/(1+(occ))) * 100)) 

    def findBoundary(self, x_origin, y_origin, r, angle):
        bound_y = int(y_origin - r*cos(angle))
        bound_x = int(x_origin + r*sin(angle))
        return (bound_x, bound_y)

    def drawLine(self, origin, end_point, list_of_cells):
        #switch to a modified bresemhams algorithm http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html
        x = origin[0]
        y = origin[1]
        x_p = end_point[0]
        y_p = end_point[1]
        dx = abs(x_p - x)
        dy = abs(y_p - y)
        n = 1 + dx + dy
        x_inc = (1, -1) [x_p > x] 
        y_inc = (1, -1) [y_p > y] 
        error = dx - dy
        dx *= 2
        dy *= 2
        line = [(x,y)]
        while (n > 0):
            if (error > 0):
                x += x_inc
                error -= dy
            else:
                y += y_inc
                error += dx
            n -= 1
            list_of_cells.append((x,y))

    def fillCells(self, center_x, center_y, fov, yaw, radius):
        fov /= 2.0
        ray_angle = atan(self._map.info.resolution / 6.0*radius)
        yaw -= (pi/2.0)
        neg_bound = (-fov + yaw)
        pos_bound = (fov + yaw)
        list_of_used_cells = []
        list_of_bound_cells = set([])
        r = radius / self._map.info.resolution

        #set the cell at the source of the ray to not occupied
        x_origin = int(center_x/self._map.info.resolution + self._map.info.width/2)
        y_origin = int(center_y/self._map.info.resolution + self._map.info.height/2)

        #TODO TODO TODO OPTIMIZE change from list to dictionary
        #generate the set of map cells to update
        bound_angles = []
        x = neg_bound
        while x < pos_bound:
            bound_angles.append(x)
            x += ray_angle

        list_of_bound_cells = set(list(map(lambda x: self.findBoundary(x_origin, y_origin, r, x), bound_angles)))
 
        map(lambda x: self.drawLine((x_origin, y_origin),x, list_of_used_cells), list_of_bound_cells)
        list_of_used_cells = set( list_of_used_cells )

        #TODO TODO TODO OPTIMIZE
        #for each of the cells update the probabilty        
        self._update_map.acquire() 
        if (fov == pi):
            map(lambda x: self.robotUpdateCellProb(None, x), list_of_used_cells)
        else:
            map(lambda x: self.robotUpdateCellProb(self.inverseSensorModel(x, radius, 2.5, fov,(x_origin, y_origin)), x), list_of_used_cells)
        self._update_map.release()

    def inverseSensorModel(self, cell, scan, max_scan, fov, origin):
        r = sqrt((cell[0] - origin[0])**2 + (cell[1] - origin[1])**2) * self._map.info.resolution
        if (r > (scan * (1 - self._scan_tolerance))):
            p_occ = 0.75#(((max_scan - r)/max_scan) + ((fov-theta)/fov)) * self._max_occupancy / 2.0
        else:
            p_occ = 0.25#(1 - (((max_scan - r)/max_scan) + ((fov-theta)/fov)) / 2.0)
        return p_occ

    def updateRobotPosition(self, data):
        self._position_mutex.acquire()
        (roll,pitch,yaw) = euler_from_quaternion([float(data.pose.pose.orientation.x), float(data.pose.pose.orientation.y), float(data.pose.pose.orientation.z), float(data.pose.pose.orientation.w)])
        self._position = np.matrix([[data.pose.pose.position.x],
                                    [data.pose.pose.position.y],
                                    [yaw]])
        self._position_mutex.release()

        if sqrt(((self._position[0] - self._old_position[0])**2) + ((self._position[1] - self._old_position[1])**2)) >=  self._map.info.resolution:
            self.fillCells(self._position[0],self._position[1], (2*pi), 0, self._car_radius)

        #the robot is represented by a circle
        self._old_position = self._position

    def ultrasonicCallback(self, data):
        now = rospy.Time(0)
        self._listener.waitForTransform(self._map.header.frame_id, data.header.frame_id, now ,rospy.Duration(0.1))
        (trans,rot) = self._listener.lookupTransform(self._map.header.frame_id, data.header.frame_id, now)
        (roll,pitch,yaw) = euler_from_quaternion([rot[0],rot[1], rot[2], rot[3]])

        self.fillCells(trans[0], trans[1], data.field_of_view, yaw, 2.5)#data.range)

if __name__ == '__main__':
    main()

sys.exit(0)
