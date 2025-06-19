#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import math

def main():
    rospy.init_node('dummy_tcp_pose_publisher')
    
    # Create publisher for TCP pose
    tcp_pose_pub = rospy.Publisher('/end_effector_pose', PoseStamped, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # Create dummy pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"
        
        # Set some dummy position values
        pose_msg.pose.position.x = 0.5
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.4
        
        # Set orientation to identity quaternion
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        
        # Publish the message
        tcp_pose_pub.publish(pose_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
