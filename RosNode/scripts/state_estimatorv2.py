#!/usr/bin/env python
import rospy
import numpy as np
import tf
from numpy import zeros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Quaternion, AccelWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32, Time
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import tan, sin, cos, radians
from threading import Semaphore, Thread
from the_learner.msg import RawSpeedEncoder

def main():
    global steering, throttle, command_mutex, speed, speed_available
    speed_available = False
    rospy.init_node('state_estimator', anonymous = True)
    
    ekf_thread = EKFThread()
    ekf_thread.start()
    
    rospy.spin()
    ekf_thread.close()
    ekf_thread.join()

class EKFThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self._run = True
        self._update_step_ready = Semaphore()
        self._br = tf.TransformBroadcaster()
        
        self._speed_available = False
        self._pose = PoseWithCovariance()
        self._speed = 0
        self._dbw = 0.114 #distance between wheels   
        self._last_t = rospy.Time.now()    
        self._dt = 0

        self._pose.header.frame_id = "learner/odom"
        self._pose.header.stamp = rospy.Time.now()
        self._pose.header.seq = 0
        self._pose.child_frame_id = "learner/base_link"

        self._pose.pose.pose.position.x = 0
        self._pose.pose.pose.position.y = 0
        self._pose.pose.pose.position.z = 0
        self._pose.pose.pose.orientation.x = 0
        self._pose.pose.pose.orientation.y = 0
        self._pose.pose.pose.orientation.z = 0
        self._pose.pose.pose.orientation.w = 1

        self._pose.twist.twist.linear.x = 0
        self._pose.twist.twist.linear.y = 0
        self._pose.twist.twist.linear.z = 0
        self._pose.twist.twist.angular.x = 0
        self._pose.twist.twist.angular.y = 0
        self._pose.twist.twist.angular.z = 0

        self._pose_publisher = rospy.Publisher('learner/odom',Odometry,queue_size = 1)
        rospy.Subscriber("sensors/accelerometer", AccelWithCovarianceStamped, self.updateAccelerometer)
        rospy.Subscriber("sensors/imu", Imu, self.updateImu)
        rospy.Subscriber("learner/raw_odom", Odometry, self.updateEncoderOdom)
        self._update_step_ready.acquire()
        
        #initialize EKF matrices
        state_estimate = np.matrix(zeroes()).getT()
        state_covariance = np.matrix([[10,0,0,0,0,0,0,0] #state uncertainty covariance matrix
                                      [0,10,0,0,0,0,0,0]
                                      [0,0,10,0,0,0,0,0]
                                      [0,0,0,10,0,0,0,0]
                                      [0,0,0,0,10,0,0,0]
                                      [0,0,0,0,0,10,0,0]
                                      [0,0,0,0,0,0,10,0]
                                      [0,0,0,0,0,0,0,10]])
        motion_noise_covariance = np.matrix([[0.01,0,0,0,0,0,0,0] #process noise covariance matrix
                                             [0,0.01,0,0,0,0,0,0]
                                             [0,0,0.001,0,0,0,0,0]
                                             [0,0,0,0.01,0,0,0,0]
                                             [0,0,0,0,0.01,0,0,0]
                                             [0,0,0,0,0,0.01,0,0]
                                             [0,0,0,0,0,0,0.01,0]
                                             [0,0,0,0,0,0,0,0.01]]) 
        I = np.matrix([[1,0,0,0,0,0,0,0] #state uncertainty covariance matrix
                       [0,1,0,0,0,0,0,0]
                       [0,0,1,0,0,0,0,0]
                       [0,0,0,1,0,0,0,0]
                       [0,0,0,0,1,0,0,0]
                       [0,0,0,0,0,1,0,0]
                       [0,0,0,0,0,0,1,0]
                       [0,0,0,0,0,0,0,1]])
        self._last_t = rospy,time()

    def run(self):
        while self._run:
            self._update_step_ready.acquire()
            #prediction step
            dt = (self._current_t - self._last_t).to_sec()
            self._last_t = self._current_t
            state_transition = [[1,0,0,dt,0,0,(dt**2)/2.0,0]#same as the state jacobian
                                [0,1,0,0,dt,0,0,(dt**2)/2.0]
                                [0,0,1,0,0,dt,0,0]
                                [0,0,0,1,0,0,dt,0]
                                [0,0,0,0,1,0,0,dt]
                                [0,0,0,0,0,1,0,0]
                                [0,0,0,0,0,0,1,0]
                                [0,0,0,0,0,0,0,1]]
            state_est = state_est * state_transition
            state_covariance = 0
            self._pose_publisher.publish(self._pose)

    def close(self):
        self._run = False  

    def updateAccelerometer(self, data):
        self._update_step_ready.acquire()
        
    def updateEncoderOdom(self, data):
        self._update_step_ready.acquire()

    def updateImu(self, data):
        self._update_step_ready.acquire()

if __name__ == '__main__':
    main()
