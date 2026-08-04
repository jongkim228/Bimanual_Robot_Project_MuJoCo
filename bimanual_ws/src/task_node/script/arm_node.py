#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def callback(msg):
    cmd = PoseStamped()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = msg.header.frame_id
    cmd.pose = msg.pose
    publish(cmd)

def main():
    rospy.init_node("arm_node")
    sub = rospy.Subscriber("target_pose", PoseStamped, callback)
    
    rospy.spin()

if __name__ == "__main__":
    main()