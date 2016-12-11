#!/usr/bin/env python
import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseWithCovariance
from numpy import zeros
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Float32, Time
from tf.transformations import quaternion_from_euler

def main():
    rospy.init_node('state_estimator', anonymous = True)
    state_publisher = rospy.Publisher('learner/odom',Odometry,queue_size = 1)
    rospy.Subscriber("sensors/imu", Imu, EKF_implementation)
    br = tf.TransformBroadcaster()
    learner_state = Odometry()
    learner_pose = learner_state.pose.pose
    """learner_state.pose.position.y = 0
    learner_state.pose.position.z = 0.06
    learner_state.pose.orientation.x = 0
    learner_state.pose.orientation.y = 0
    learner_state.pose.orientation.z = 0
    learner_state.pose.orientation.w = 1
    learner_state.header.frame_id = "learner"
    learner_state.header.stamp = rospy.Time.now()
    """
    rate = rospy.Rate(40.0)

    while not rospy.is_shutdown():
#        state_publisher.publish(learner_state)
        #TODO replace the zeroes below with the state params

        br.sendTransform((0.0, 0.0, 0.0),
                         quaternion_from_euler(0,0,0),
                         rospy.Time.now(),
                         "learner/odom",
                         "map")
        br.sendTransform((0.0, 0.0, 0.0),
                         quaternion_from_euler(0,0,0),
                         rospy.Time.now(),
                         "ardrone_base_link",
                         "map")
        rate.sleep()   

def EKF_implementation(self):
    pass

if __name__ == '__main__':
    main()
