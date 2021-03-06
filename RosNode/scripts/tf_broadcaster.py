#!/usr/bin/env python  
import rospy
from math import pi
import tf
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node("ultrasonic_tf_publisher");
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(15)
    
    while not rospy.is_shutdown():
        br.sendTransform((0.067, 0.076, 0.0),
                         quaternion_from_euler(0,0,0.8116),
                         rospy.Time.now(),
                         "ultrasonic_0_link",
                         "learner/base_link")
        br.sendTransform((0.108, 0.028, 0.0),
                         quaternion_from_euler(0,0,0.2705),
                         rospy.Time.now(),
                         "ultrasonic_1_link",
                         "learner/base_link")
        br.sendTransform((0.108, -0.028, 0.0),
                         quaternion_from_euler(0,0,-0.2705),
                         rospy.Time.now(),
                         "ultrasonic_2_link",
                         "learner/base_link")
        br.sendTransform((0.067, -0.076, 0.0),
                         quaternion_from_euler(0,0,-0.8116),
                         rospy.Time.now(),
                         "ultrasonic_3_link",
                         "learner/base_link")
        br.sendTransform((-0.01, -0.052, 0.0),
                         quaternion_from_euler(0,0,-1.571),
                         rospy.Time.now(),
                         "ultrasonic_4_link",
                         "learner/base_link")
        br.sendTransform((-0.04, 0.0, 0.0),
                         quaternion_from_euler(0,0,pi),
                         rospy.Time.now(),
                         "ultrasonic_5_link",
                         "learner/base_link")
        br.sendTransform((-0.01, .052, 0.0),
                         quaternion_from_euler(0,0,1.571),
                         rospy.Time.now(),
                         "ultrasonic_6_link",
                         "learner/base_link")
        rate.sleep()
