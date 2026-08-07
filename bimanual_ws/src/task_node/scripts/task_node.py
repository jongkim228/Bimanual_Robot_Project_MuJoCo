#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

HOME_QUAT = dict(qx=0.9239557, qy=-0.3824995, qz=0.0, qw=0.0)


def target_pose(x, y, z, qx=None, qy=None, qz=None, qw=None):

    if qx is None:
        qx = HOME_QUAT["qx"]
    if qy is None:
        qy = HOME_QUAT["qy"]
    if qz is None:
        qz = HOME_QUAT["qz"]
    if qw is None:
        qw = HOME_QUAT["qw"]


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
    pub_right = rospy.Publisher("target_pose_right", PoseStamped, queue_size=10)
    pub_left = rospy.Publisher("target_pose_left", PoseStamped, queue_size=10)

    rospy.sleep(1.0)
    right_target = target_pose(0.45, 0.3, 0.3, qx=-0.7071, qy=0.0, qz=0.0, qw=0.7071)

    left_target = target_pose(0.45, -0.3, 0.5)

    pub_right.publish(right_target)

    rospy.sleep(5.0)

    pub_left.publish(left_target)



if __name__ == "__main__":
    main()