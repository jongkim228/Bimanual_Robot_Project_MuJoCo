#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def target_pose(x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = ""
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg

def main():
    rospy.init_node("task_node")
    pub_left = rospy.Publisher("target_pose_left", PoseStamped, queue_size=10)
    pub_right = rospy.Publisher("target_pose_right", PoseStamped, queue_size=10)

    rospy.sleep(1.0)

    left_target = target_pose(0.4, 0.1, 0.45)
    right_target = target_pose(0.4, -0.5, 0.45)

    pub_left.publish(left_target)
    rospy.loginfo("Published left target pose")

    pub_right.publish(right_target)
    rospy.loginfo("Published right target pose")


if __name__ == "__main__":
    main()