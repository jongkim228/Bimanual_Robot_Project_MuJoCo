#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

def callback(msg):
    cmd = PoseStamped()
    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = ""
    cmd.pose = msg.pose
    pub.publish(cmd)

    try:
        resp = start_srv()
        rospy.loginfo("Start service response: success=%s, message=%s", resp.success, resp.message)
    except rospy.ServiceException as e:
        rospy.logwarn("Service call failed: %s", str(e))

def main():
    global pub, start_srv
    rospy.init_node("arm_node")
    pub = rospy.Publisher("/left/left_bottle_pose_controller/target_pose", PoseStamped, queue_size=10)

    start_srv = rospy.ServiceProxy("/left/left_bottle_pose_controller/start", Trigger)

    sub = rospy.Subscriber("target_pose", PoseStamped, callback)
    
    rospy.spin()

if __name__ == "__main__":
    main()