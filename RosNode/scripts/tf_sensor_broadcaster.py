#!/usr/bin/env python  
import rospy
import tf
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node("ultrasonic_tf_publisher");
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    
    while not rospy.is_shutdown():
        br.sendTransform((0.067, 0.076, 0.0),
                         quaternion_from_euler(0,0,0.8116),
                         rospy.Time.now(),
                         "learner_ultrasonic_0_link",
                         "learner")
        br.sendTransform((0.108, 0.028, 0.0),
                         quaternion_from_euler(0,0,0.2705),
                         rospy.Time.now(),
                         "learner_ultrasonic_1_link",
                         "learner")
        br.sendTransform((0.108, -0.028, 0.0),
                         quaternion_from_euler(0,0,-0.2705),
                         rospy.Time.now(),
                         "learner_ultrasonic_2_link",
                         "learner")
        br.sendTransform((0.067, -0.076, 0.0),
                         quaternion_from_euler(0,0,-0.8116),
                         rospy.Time.now(),
                         "learner_ultrasonic_3_link",
                         "learner")
        br.sendTransform((-0.01, -0.052, 0.0),
                         quaternion_from_euler(0,0,-1.571),
                         rospy.Time.now(),
                         "learner_ultrasonic_4_link",
                         "learner")
        br.sendTransform((-0.04, 0.0, 0.0),
                         quaternion_from_euler(0,0,3.142),
                         rospy.Time.now(),
                         "learner_ultrasonic_5_link",
                         "learner")
        br.sendTransform((-0.01, .052, 0.0),
                         quaternion_from_euler(0,0,1.571),
                         rospy.Time.now(),
                         "learner_ultrasonic_6_link",
                         "learner")
        br.sendTransform((0.04, 0.0, 0.07),
                         quaternion_from_euler(3.142,3.142/2.0,3.142),
                         rospy.Time.now(),
                         "learner_imu_link",
                         "learner")
        br.sendTransform((-0.06, -0.08, 0.00),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "learner_gps_link",
                         "learner")
        rate.sleep()
