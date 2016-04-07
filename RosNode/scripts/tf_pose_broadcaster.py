#!/usr/bin/env python  
import rospy
import tf
from geometry_msgs.msg import PoseStamped

def handle_learner_pose(msg, name):
    br = tf.TransformBroadcaster()
    #TODO fill out sendTransform
    br.sendTransform((msg.pose.position.x,msg.pose.position.y,msg.pose.position.z),
                     (msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w),
                     rospy.Time.now(),
                     name,
                     "world")

if __name__ == '__main__':
    rospy.init_node('learner_tf_broadcaster')
    rospy.Subscriber('/learner/pose', PoseStamped, handle_learner_pose,
                     "learner")
    rospy.spin()
