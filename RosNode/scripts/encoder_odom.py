#!/usr/bin/env python
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseWithCovariance
from numpy import zeros
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32, Time
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import tan
from threading import Semaphore
from the_learner.msg import RawSpeedEncoder

def main():
    rospy.init_node('encoder_odom', anonymous = True)
    br = tf.TransformBroadcaster()

    odom_publisher = rospy.Publisher('learner/odom',Odometry,queue_size = 1)
    #rospy.Subscriber("learner/pose", RawEncoderSpeed, )
    rospy.Subscriber("sensors/encoder", RawSpeedEncoder, updateOdom)
    rospy.Subscriber("learner/command", Int32, updateCommand)

    global odom, steering, throttle, command_mutex, dbw   

    odom = Odometry()
    steering = 0
    throttle = 0
    dbw = 0.01 #distance between wheels   
    command_mutex = Semaphore()    

    odom.header.frame_id = "learner/odom"
    odom.header.stamp = rospy.Time.now()
    odom.header.seq = 0
    odom.child_frame_id = "learner/base_link"

    odom.pose.pose.position.x = 0
    odom.pose.pose.position.y = 0
    odom.pose.pose.position.z = 0
    odom.pose.pose.orientation.x = 0
    odom.pose.pose.orientation.y = 0
    odom.pose.pose.orientation.z = 0
    odom.pose.pose.orientation.w = 1

    odom.twist.twist.linear.x = 0
    odom.twist.twist.linear.y = 0
    odom.twist.twist.linear.z = 0
    odom.twist.twist.angular.x = 0
    odom.twist.twist.angular.y = 0
    odom.twist.twist.angular.z = 0
    last_time = rospy.Time.now()    
    print "Hello"
    rospy.spin()

def updateOdom(data):
    #TODO add direction from throttle to the speed
    command_mutex.acquire()
    euler = euler_from_quaternion(list(odom.pose.pose.orientation))
    dt = (last_time - rospy.Time.now()).toSecs()
    omega = (data.speed * tan(radians(steering))) / dbw
    vx = data.speed * cos(euler[0]) 
    vy = data.speed * sin(euler[0]) 
    command_mutex.release()

    odom.pose.pose.position.x += vx * dt
    odom.pose.pose.position.y += vy *dt
    euler[0] += omega * dt
    odom.pose.pose.orientation.x = quaternion_from_euler(euler)
    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = xy
    odom.twist.twist.angular.x = omega
    odom.pose.pose.orientation.x = quaternion_from_euler(euler)
    
    br.sendTransform((x, y, z),
                     quaternion_from_euler(0,0,0),
                     rospy.Time.now(),
                     "learner/base_link",
                     "learner/odom")
    odom_publisher.publish(odom)
    last_time = rospy.Time.now()    

def updateCommand(data):
    command_mutex.acquire()
    steering = int((data.data >> 1) & 127) - 30
    throttle = int(data.data >> 8)
    command_mutex.release()

if __name__ == '__main__':
    main()
