#!/usr/bin/env python3

import argparse
import sys

import numpy as np
import rospy
import tf.transformations as tft

from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped


def main():
    parser = argparse.ArgumentParser(
        description="Publish a Cartesian target relative to the current Panda EE pose."
    )

    parser.add_argument("--dx", type=float, default=0.0)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=0.01)

    parser.add_argument(
        "--state-topic",
        default="/panda_left/franka_state_controller/franka_states",
    )

    parser.add_argument(
        "--target-topic",
        default="/panda_left/left_fixed_pose_controller/target_pose",
    )

    parser.add_argument(
        "--frame-id",
        default="panda_left_link0",
    )

    args = parser.parse_args()

    rospy.init_node("publish_relative_panda_target", anonymous=True)

    rospy.loginfo("Waiting for Franka state on %s", args.state_topic)

    state = rospy.wait_for_message(
        args.state_topic,
        FrankaState,
        timeout=10.0,
    )

    # Franka transforms are stored column-major.
    transform = np.asarray(
        state.O_T_EE_d,
        dtype=float,
    ).reshape((4, 4), order="F")

    # tf returns quaternion in [x, y, z, w] order.
    quaternion = tft.quaternion_from_matrix(transform)

    target = PoseStamped()
    target.header.stamp = rospy.Time.now()
    target.header.frame_id = args.frame_id

    target.pose.position.x = float(transform[0, 3] + args.dx)
    target.pose.position.y = float(transform[1, 3] + args.dy)
    target.pose.position.z = float(transform[2, 3] + args.dz)

    target.pose.orientation.x = float(quaternion[0])
    target.pose.orientation.y = float(quaternion[1])
    target.pose.orientation.z = float(quaternion[2])
    target.pose.orientation.w = float(quaternion[3])

    publisher = rospy.Publisher(
        args.target_topic,
        PoseStamped,
        queue_size=1,
        latch=True,
    )

    rospy.sleep(1.0)
    publisher.publish(target)
    rospy.sleep(0.5)

    rospy.loginfo(
        "Current pose: x=%.6f, y=%.6f, z=%.6f",
        transform[0, 3],
        transform[1, 3],
        transform[2, 3],
    )

    rospy.loginfo(
        "Published target: x=%.6f, y=%.6f, z=%.6f",
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z,
    )

    rospy.loginfo(
        "Target quaternion: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
        target.pose.orientation.x,
        target.pose.orientation.y,
        target.pose.orientation.z,
        target.pose.orientation.w,
    )

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSException as error:
        rospy.logerr("ROS error: %s", error)
        sys.exit(1)
    except rospy.ROSInterruptException:
        sys.exit(1)
    except Exception as error:
        rospy.logerr("Unexpected error: %s", error)
        sys.exit(1)
