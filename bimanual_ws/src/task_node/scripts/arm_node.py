#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def callback(msg):
    cmd = PoseStamped()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = ""
    cmd.pose = msg.pose
    pub.publish(cmd)

def main():
    global pub
    rospy.init_node("arm_node")
    pub = rospy.Publisher("/left/left_bottle_pose_controller/target_pose", PoseStamped, queue_size=10)
    sub = rospy.Subscriber("target_pose", PoseStamped, callback)
    
    rospy.spin()

if __name__ == "__main__":
    main()