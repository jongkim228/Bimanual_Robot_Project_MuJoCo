#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node("task_node")
    pub = rospy.Publisher("target_pose", PoseStamped, queue_size=10)

    rospy.sleep(1.0)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ""

    msg.pose.position.x = 0.4
    msg.pose.position.y = 0.1
    msg.pose.position.z = 0.45

    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0

    pub.publish(msg)
    rospy.loginfo("Published target pose")


if __name__ == "__main__":
    main()