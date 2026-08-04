#!/usr/bin/env python3
import rospy

def main():
    rospy.init_node()
    rospy.rospy.loginfo("info message")
    rospy.spin()


if __name__ == "__main__":
    main()
    