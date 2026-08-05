#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

pub = None
start_srv = None

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

    rospy.init_node("arm_node", anonymous=True)

    side = rospy.get_param("~side", "left")
    controller_name = "{}_bottle_pose_controller".format(side)

    #publish target coordinate to controller
    pub = rospy.Publisher("/{}/{}/target_pose".format(side, controller_name), PoseStamped, queue_size=10)

    #service to start control
    rospy.wait_for_service("/{}/{}/start".format(side, controller_name))
    start_srv = rospy.ServiceProxy("/{}/{}/start".format(side, controller_name), Trigger)

    sub = rospy.Subscriber("target_pose", PoseStamped, callback)
    
    rospy.spin()

if __name__ == "__main__":
    main()