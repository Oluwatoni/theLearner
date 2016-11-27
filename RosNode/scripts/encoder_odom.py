#!/usr/bin/env python
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseWithCovariance, Quaternion
from numpy import zeros
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Time
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import tan, sin, cos, radians
from threading import Semaphore, Thread
from the_learner.msg import RawSpeedEncoder

def main():
    global steering, throttle, command_mutex, speed, speed_available
    speed_available = False
    rospy.init_node('encoder_odom', anonymous = True)
    
    odom_thread = OdomThread()
    odom_thread.start()
    
    rospy.spin()
    odom_thread.close()
    odom_thread.join()

class OdomThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self._run = True
        
        self._br = tf.TransformBroadcaster()
        
        self._speed_available = False
        self._odom = Odometry()
        self._speed = 0
        self._steering = 0
        self._throttle = 0
        self._dbw = 0.114 #distance between wheels   
        self._command_mutex = Semaphore()    
        self._last_t = rospy.Time.now()    
        self._dt = 0

        self._odom.header.frame_id = "learner/odom"
        self._odom.header.stamp = rospy.Time.now()
        self._odom.header.seq = 0
        self._odom.child_frame_id = "learner/base_link"

        self._odom.pose.pose.position.x = 0
        self._odom.pose.pose.position.y = 0
        self._odom.pose.pose.position.z = 0
        self._odom.pose.pose.orientation.x = 0
        self._odom.pose.pose.orientation.y = 0
        self._odom.pose.pose.orientation.z = 0
        self._odom.pose.pose.orientation.w = 1

        self._odom.twist.twist.linear.x = 0
        self._odom.twist.twist.linear.y = 0
        self._odom.twist.twist.linear.z = 0
        self._odom.twist.twist.angular.x = 0
        self._odom.twist.twist.angular.y = 0
        self._odom.twist.twist.angular.z = 0
        self._odom_publisher = rospy.Publisher('learner/odom',Odometry,queue_size = 1)
        rospy.Subscriber("sensors/encoder", RawSpeedEncoder, self.updateOdom)
        rospy.Subscriber("learner/command", Int32, self.updateCommand)
    
    def run(self):
        while (self._run):
            if self._speed_available:
                self._speed_available = False
                (roll,pitch,yaw) = euler_from_quaternion([float(self._odom.pose.pose.orientation.x), float(self._odom.pose.pose.orientation.y), float(self._odom.pose.pose.orientation.z), float(self._odom.pose.pose.orientation.w)])
                
                self._command_mutex.acquire()
                omega = (self._speed * tan(self._steering)) / self._dbw
                vx = self._speed * cos(yaw) 
                vy = self._speed * sin(yaw) 
                self._command_mutex.release()

                self._odom.header.stamp = rospy.Time.now()
                
                self._odom.pose.pose.position.x += (vx * self._dt)
                self._odom.pose.pose.position.y += (vy * self._dt)
#                print self._steering
                yaw = yaw + omega * self._dt
                (x,y,z,w) = quaternion_from_euler(roll, pitch, yaw)
                self._odom.pose.pose.orientation.x = x
                self._odom.pose.pose.orientation.y = y
                self._odom.pose.pose.orientation.z = z
                self._odom.pose.pose.orientation.w = w
                self._odom.twist.twist.linear.x = vx
                self._odom.twist.twist.linear.y = vy
                self._odom.twist.twist.angular.z = omega
                self._br.sendTransform((self._odom.pose.pose.position.x, self._odom.pose.pose.position.x, 0),
                                 (x,y,z,w),
                                 rospy.Time.now(),
                                 "learner/base_link",
                                 "learner/odom")
#                if not(rospy.is_shutdown):
                self._odom_publisher.publish(self._odom)

    def close(self):
        self._run = False        

    def updateOdom(self, data):
        self._dt = (rospy.Time.now() - self._last_t).to_sec()
        self._speed = ((data.speed / 40.0) * 0.187) / self._dt
        print self._speed
        self._last_t = rospy.Time.now()    
        self._speed_available = True

    def updateCommand(self, data):
        self._command_mutex.acquire()
        self._steering = radians((int((data.data >> 1) & 127) - 30) * 0.892)
        self._throttle = int(data.data >> 8)
        self._command_mutex.release()

if __name__ == '__main__':
    main()
