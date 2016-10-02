#!/usr/bin/env python  
import rospy
import tf
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node("ultrasonic_tf_publisher");
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(40.0)
    
    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, 0.0),
                         quaternion_from_euler(0,0,0),
                         rospy.Time.now(),
                         "learner/base_link",
                         "learner/odom")
        br.sendTransform((0.067, 0.076, 0.0),
                         quaternion_from_euler(0,0,0.8116),
                         rospy.Time.now(),
                         "learner_ultrasonic_0_link",
                         "learner/base_link")
        br.sendTransform((0.108, 0.028, 0.0),
                         quaternion_from_euler(0,0,0.2705),
                         rospy.Time.now(),
                         "learner_ultrasonic_1_link",
                         "learner/base_link")
        br.sendTransform((0.108, -0.028, 0.0),
                         quaternion_from_euler(0,0,-0.2705),
                         rospy.Time.now(),
                         "learner_ultrasonic_2_link",
                         "learner/base_link")
        br.sendTransform((0.067, -0.076, 0.0),
                         quaternion_from_euler(0,0,-0.8116),
                         rospy.Time.now(),
                         "learner_ultrasonic_3_link",
                         "learner/base_link")
        br.sendTransform((-0.01, -0.052, 0.0),
                         quaternion_from_euler(0,0,-1.571),
                         rospy.Time.now(),
                         "learner_ultrasonic_4_link",
                         "learner/base_link")
        br.sendTransform((-0.04, 0.0, 0.0),
                         quaternion_from_euler(0,0,3.142),
                         rospy.Time.now(),
                         "learner_ultrasonic_5_link",
                         "learner/base_link")
        br.sendTransform((-0.01, .052, 0.0),
                         quaternion_from_euler(0,0,1.571),
                         rospy.Time.now(),
                         "learner_ultrasonic_6_link",
                         "learner/base_link")
        br.sendTransform((0.04, 0.0, 0.152),
                         quaternion_from_euler(0,0,(3.142/2)),
                         rospy.Time.now(),
                         "learner_imu_link",
                         "learner/base_link")
        rate.sleep()
