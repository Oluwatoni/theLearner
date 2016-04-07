#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('learner_pose_estimator', anonymous = True)
    pose_publisher = rospy.Publisher('learner/pose',PoseStamped,queue_size = 1)
    learner_pose = PoseStamped()
    learner_pose.pose.position.x = 0
    learner_pose.pose.position.y = 0
    learner_pose.pose.position.z = 0.06
    learner_pose.pose.orientation.x = 0
    learner_pose.pose.orientation.y = 0
    learner_pose.pose.orientation.z = 0
    learner_pose.pose.orientation.w = 1
    learner_pose.header.frame_id = "learner"
    learner_pose.header.stamp = rospy.Time.now()
    
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        pose_publisher.publish(learner_pose)
        rate.sleep()   

if __name__ == '__main__':
    main()
