#!/usr/bin/env python
import rospy
import numpy as np
import tf
from numpy import zeros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, AccelWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int32, Time
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import tan, sin, cos, radians, pi
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
        self._sensor_mutex = Semaphore()
        self._br = tf.TransformBroadcaster()
        
        self._speed_available = False
        self._odom = Odometry()
        self._velocity_x = 0#integrated acc data used to determine the direction for the encoder speed
        self._acc_filter_factor = 0.94
        self._last_imu_time = rospy.Time.now()
        self._roll = 0
        self._pitch = 0
        self._yaw = 0
        self._speed = 0.0
        self._dbw = 0.114 #distance between wheels   
        self._last_t = rospy.Time.now()    
        self._dt = 0.0

        self._steering = 0
        self._throttle = 0
        self._dbw = 0.114 #distance between wheels   
        self._command_mutex = Semaphore()    
        self._last_encoder_time = rospy.Time.now()

        self._odom.header.frame_id = "learner/odom"
        self._odom.header.stamp = rospy.Time.now()
        self._odom.header.seq = 0
        self._child_link = "learner/base_link"

        self._odom.pose.pose.position.x = 0.0
        self._odom.pose.pose.position.y = 0.0
        self._odom.pose.pose.position.z = 0.0
        self._odom.pose.pose.orientation.x = 0.0
        self._odom.pose.pose.orientation.y = 0.0
        self._odom.pose.pose.orientation.z = 0.0
        self._odom.pose.pose.orientation.w = 1

        self._odom.twist.twist.linear.x = 0.0
        self._odom.twist.twist.linear.y = 0.0
        self._odom.twist.twist.linear.z = 0.0
        self._odom.twist.twist.angular.x = 0.0
        self._odom.twist.twist.angular.y = 0.0
        self._odom.twist.twist.angular.z = 0.0

        self._odom_publisher = rospy.Publisher('learner/odom', Odometry,queue_size = 1)
        self._test = rospy.Publisher('test', Float32,queue_size = 1)
        rospy.Subscriber("imu/data", Imu, self.updateImu)
        rospy.Subscriber("sensors/encoder", RawSpeedEncoder, self.updateEncoderOdom)
        rospy.Subscriber("learner/command", Int32, self.updateCommand)
        self._update_step_ready.acquire()
        
        #initialize EKF matrices
        self._state_estimate = np.matrix(zeros(8)).getT()
        self._state_covariance = np.matrix([[10,0,0,0,0,0,0,0], #state uncertainty covariance matrix
                                            [0,10,0,0,0,0,0,0],
                                            [0,0,10,0,0,0,0,0],
                                            [0,0,0,10,0,0,0,0],
                                            [0,0,0,0,10,0,0,0],
                                            [0,0,0,0,0,10,0,0],
                                            [0,0,0,0,0,0,10,0],
                                            [0,0,0,0,0,0,0,10]])
        self._motion_noise_covariance = np.matrix([[0.01,0,0,0,0,0,0,0], #process noise covariance matrix
                                                   [0,0.01,0,0,0,0,0,0], 
                                                   [0,0,0.1,0,0,0,0,0],
                                                   [0,0,0,0.001,0,0,0,0],
                                                   [0,0,0,0,0.001,0,0,0],
                                                   [0,0,0,0,0,0.001,0,0],
                                                   [0,0,0,0,0,0,0.001,0],
                                                   [0,0,0,0,0,0,0,0.001]]) 
        self._I = np.matrix([[1,0,0,0,0,0,0,0], #state uncertainty covariance matrix
                             [0,1,0,0,0,0,0,0],
                             [0,0,1,0,0,0,0,0],
                             [0,0,0,1,0,0,0,0],
                             [0,0,0,0,1,0,0,0],
                             [0,0,0,0,0,1,0,0],
                             [0,0,0,0,0,0,1,0],
                             [0,0,0,0,0,0,0,1]])
        self._sensor_jacobian = self._I
        self._measurements = self._I
        self._sensor_covariance = self._I
        self._last_t = rospy.Time.now()
        self._current_t = rospy.Time.now()

    def run(self):
        while self._run:
            if self._update_step_ready.acquire():#blocking = False):
                if not(self._run):
                    break
                #prediction step
                self._current_t = rospy.Time.now()
                dt = (self._current_t - self._last_t).to_sec()
                self._last_t = self._current_t
                state_transition = np.matrix([[1,0,0,dt,0,0,(dt**2)/2.0,0],#same as the state jacobian
                                              [0,1,0,0,dt,0,0,(dt**2)/2.0],
                                              [0,0,1,0,0,dt,0,0],
                                              [0,0,0,1,0,0,dt,0],
                                              [0,0,0,0,1,0,0,dt],
                                              [0,0,0,0,0,1,0,0],
                                              [0,0,0,0,0,0,1,0],
                                              [0,0,0,0,0,0,0,1]])
                self._state_estimate = state_transition * self._state_estimate
#               print state_transition
#               print self._state_covariance
#               print (state_transition).getT()
#               print self._motion_noise_covariance
                self._state_covariance = state_transition * self._state_covariance * (state_transition).getT() + self._motion_noise_covariance
                #update step
                measurement_error = self._measurements - (self._sensor_jacobian * self._state_estimate)
                S = self._sensor_jacobian * self._state_covariance * (self._sensor_jacobian).getT() + self._sensor_covariance
                K = self._state_covariance * (self._sensor_jacobian).getT() * (S).getI()
                self._state_estimate = self._state_estimate + (K * measurement_error)
                self._state_covariance = (self._I - K * self._sensor_jacobian) * self._state_covariance

                self._odom.pose.pose.position.x = self._state_estimate[0]
                self._odom.pose.pose.position.y = self._state_estimate[1]
                (x,y,z,w) = quaternion_from_euler(self._roll, self._pitch, self._state_estimate[2], 'rxyz')
                self._odom.pose.pose.orientation.x = x
                self._odom.pose.pose.orientation.y = y
                self._odom.pose.pose.orientation.z = z
                self._odom.pose.pose.orientation.w = w
                self._odom.twist.twist.linear.x = self._state_estimate[3] 
                self._odom.twist.twist.linear.y = self._state_estimate[4]
                self._odom.twist.twist.angular.z = self._state_estimate[5]
                self._odom_publisher.publish(self._odom)
                self._br.sendTransform((self._state_estimate[0],self._state_estimate[1], 0.0),
                                     (x,y,z,w),
                                     rospy.Time.now(),
                                     "learner/base_link",
                                     "learner/odom")
                self._sensor_mutex.release()

    def close(self):
        self._run = False  
        self._update_step_ready.release()

    #from http://www.varesano.net/blog/fabio/simple-gravity-compensation-9-dom-imus
    def gravity_compensate(self, q, acc):
        acc = np.array([[acc.x],[acc.y],[acc.z]])
        acc /= 9.81
        q = np.array([[q.w],[q.x],[q.y],[q.z]])
        g = np.array([[0.0], [0.0], [0.0]])
        # get expected direction of gravity
        g[0] = 2 * (q[1] * q[3] - q[0] * q[2])
        g[1] = 2 * (q[0] * q[1] + q[2] * q[3])
        g[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
        acc -= g
        # compensate accelerometer readings with the expected direction of gravity
        return (acc * 9.81)
 
    def updateEncoderOdom(self, data):
        self._sensor_mutex.acquire()
        dt = (rospy.Time.now() - self._last_encoder_time).to_sec()
        speed = ((data.speed / 40.0) * 0.187) / dt
        self._last_encoder_time = rospy.Time.now()    
        omega = (speed * tan(-self._steering)) / 0.114 
        vx = speed * cos(self._state_estimate[2]) 
        vy = speed * sin(self._state_estimate[2]) 
        self._command_mutex.release()
        self._odom.header.stamp = rospy.Time.now()

        self._measurements = np.matrix([[vx],
                                        [vy],
                                        [omega]])
        self._sensor_jacobian = np.matrix([[0,0,0,1,0,0,0,0],
                                           [0,0,0,0,1,0,0,0],
                                           [0,0,0,0,0,1,0,0]])
        self._sensor_covariance = np.matrix([[0.001,0,0],
                                             [0,0.001,0],
                                             [0,0,0.01]])
        self._update_step_ready.release()

    def updateImu(self, data):
        self._sensor_mutex.acquire()
        dt = (rospy.Time.now() - self._last_imu_time).to_sec()
        (roll,pitch,yaw) = euler_from_quaternion([float(data.orientation.x), float(data.orientation.y), float(data.orientation.z), float(data.orientation.w)])
        ground_acc = np.array(self.gravity_compensate(data.orientation, data.linear_acceleration))
        
        #TODO fix this fam
        #self._velocity_x += (self._acc_filter_factor * self._velocity_x) + ((1-self._acc_filter_factor) * ground_acc[1]*dt)
        #print self._velocity_x
        self._test.publish(self._velocity_x)

        self._roll = roll
        self._pitch = pitch
        self._yaw = yaw

        yaw = -yaw #For some reason the IMU yaw is reversed
        self._measurements = np.matrix([[yaw],
                                       [data.angular_velocity.z]])

        self._sensor_jacobian = np.matrix([[0,0,1,0,0,0,0,0],
                                           [0,0,0,0,0,1,0,0]])
        self._sensor_covariance = np.matrix([[data.orientation_covariance[8],0],
                                            [0,data.angular_velocity_covariance[8]]]) 
        self._last_imu_time = rospy.Time.now()
        self._update_step_ready.release()

    def updateCommand(self, data):
        self._command_mutex.acquire()
        self._steering = radians((int((data.data >> 1) & 127) - 30) * 0.6) # 0.6 scaling factor for the steering command
        self._throttle = int(data.data >> 10)
        self._command_mutex.release()

if __name__ == '__main__':
    main()
