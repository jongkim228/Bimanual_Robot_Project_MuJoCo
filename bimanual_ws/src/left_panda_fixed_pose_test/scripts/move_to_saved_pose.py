#!/usr/bin/env python3

import argparse
import math
import os
import sys
import time

import numpy as np
import rospy
import yaml

from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from tf.transformations import (
    quaternion_from_matrix,
    quaternion_matrix,
    quaternion_slerp,
)


STATE_TOPIC = "/panda_left/franka_state_controller/franka_states"
TARGET_TOPIC = "/panda_left/left_fixed_pose_controller/target_pose"
START_SERVICE = "/panda_left/left_fixed_pose_controller/start"
DEFAULT_POSE_DIR = os.path.expanduser(
    "~/bimanual_ws/src/"
    "left_panda_fixed_pose_test/config/taught_poses"
)


def normalise_quaternion(quaternion):
    quaternion = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(quaternion)

    if norm < 1.0e-10:
        raise ValueError("Quaternion has zero length.")

    return quaternion / norm


def quaternion_angle(q1, q2):
    q1 = normalise_quaternion(q1)
    q2 = normalise_quaternion(q2)

    dot = abs(float(np.dot(q1, q2)))
    dot = np.clip(dot, -1.0, 1.0)

    return 2.0 * math.acos(dot)


def load_pose(path):
    with open(path, "r", encoding="utf-8") as pose_file:
        data = yaml.safe_load(pose_file)

    position = np.array([
        data["position"]["x"],
        data["position"]["y"],
        data["position"]["z"],
    ], dtype=float)

    quaternion = normalise_quaternion([
        data["orientation"]["x"],
        data["orientation"]["y"],
        data["orientation"]["z"],
        data["orientation"]["w"],
    ])

    frame_id = data.get("frame_id", "panda_left_link0")

    return data, position, quaternion, frame_id


def get_actual_pose():
    state = rospy.wait_for_message(
        STATE_TOPIC,
        FrankaState,
        timeout=10.0
    )

    transform = np.asarray(
        state.O_T_EE,
        dtype=float
    ).reshape((4, 4), order="F")

    position = transform[:3, 3].copy()
    quaternion = normalise_quaternion(
        quaternion_from_matrix(transform)
    )

    return position, quaternion


def publish_target(publisher, position, quaternion, frame_id):
    message = PoseStamped()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = frame_id

    message.pose.position.x = float(position[0])
    message.pose.position.y = float(position[1])
    message.pose.position.z = float(position[2])

    message.pose.orientation.x = float(quaternion[0])
    message.pose.orientation.y = float(quaternion[1])
    message.pose.orientation.z = float(quaternion[2])
    message.pose.orientation.w = float(quaternion[3])

    publisher.publish(message)


def wait_for_target(target_position,
                    target_quaternion,
                    position_tolerance,
                    rotation_tolerance,
                    minimum_elapsed,
                    timeout):
    started = time.monotonic()
    next_print = 0.0

    final_position_error = float("inf")
    final_rotation_error = float("inf")

    while not rospy.is_shutdown():
        actual_position, actual_quaternion = get_actual_pose()

        position_error = float(
            np.linalg.norm(target_position - actual_position)
        )

        rotation_error = quaternion_angle(
            target_quaternion,
            actual_quaternion
        )

        elapsed = time.monotonic() - started

        if elapsed >= next_print:
            print(
                "  t={:5.1f}s | "
                "desired XYZ={: .5f} {: .5f} {: .5f} | "
                "actual XYZ={: .5f} {: .5f} {: .5f} | "
                "error={:5.2f} mm".format(
                    elapsed,
                    target_position[0],
                    target_position[1],
                    target_position[2],
                    actual_position[0],
                    actual_position[1],
                    actual_position[2],
                    position_error * 1000.0
                )
            )
            next_print += 0.5

        final_position_error = position_error
        final_rotation_error = rotation_error

        if (
            elapsed >= minimum_elapsed
            and position_error <= position_tolerance
            and rotation_error <= rotation_tolerance
        ):
            return True, final_position_error, final_rotation_error

        if elapsed > timeout:
            return False, final_position_error, final_rotation_error


def resolve_pose_path(pose_argument):
    expanded = os.path.expanduser(pose_argument)

    if os.path.isfile(expanded):
        return expanded

    if pose_argument in ("latest", "latest.yaml"):
        candidate = os.path.join(DEFAULT_POSE_DIR, "latest.yaml")
    else:
        candidate = os.path.join(DEFAULT_POSE_DIR, pose_argument)

        if not candidate.endswith(".yaml"):
            candidate += ".yaml"

    if not os.path.isfile(candidate):
        raise FileNotFoundError(
            "Saved pose was not found: {}".format(candidate)
        )

    return candidate


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Move the left Panda from its current pose to a saved "
            "kinesthetically taught pose."
        )
    )

    parser.add_argument(
        "pose",
        nargs="?",
        default="latest",
        help="Pose YAML path/name, or 'latest'."
    )

    parser.add_argument(
        "--max-step",
        type=float,
        default=0.01,
        help="Maximum Cartesian translation per controller move in metres."
    )

    parser.add_argument(
        "--max-rotation-deg",
        type=float,
        default=7.0,
        help="Maximum rotation per controller move in degrees."
    )

    parser.add_argument(
        "--position-tolerance-mm",
        type=float,
        default=1.5
    )

    parser.add_argument(
        "--rotation-tolerance-deg",
        type=float,
        default=1.5
    )

    parser.add_argument(
        "--motion-duration",
        type=float,
        default=4.0,
        help="Trajectory duration configured in the Cartesian controller."
    )

    parser.add_argument(
        "--settling-time",
        type=float,
        default=0.3,
        help="Additional pause after the complete controller trajectory."
    )

    parser.add_argument(
        "--step-timeout",
        type=float,
        default=7.0
    )

    args = parser.parse_args()

    if args.max_step <= 0.0 or args.max_step > 0.05:
        raise ValueError(
            "--max-step must be greater than zero and no more than 0.05 m."
        )

    rospy.init_node("move_to_saved_pose", anonymous=True)

    pose_path = resolve_pose_path(args.pose)

    data, goal_position, goal_quaternion, frame_id = load_pose(pose_path)
    start_position, start_quaternion = get_actual_pose()

    translation = float(
        np.linalg.norm(goal_position - start_position)
    )

    rotation = quaternion_angle(
        start_quaternion,
        goal_quaternion
    )

    max_rotation = math.radians(args.max_rotation_deg)

    translation_steps = max(
        1,
        int(math.ceil(translation / args.max_step))
    )

    rotation_steps = max(
        1,
        int(math.ceil(rotation / max_rotation))
    )

    number_of_steps = max(
        translation_steps,
        rotation_steps
    )

    print("")
    print("==========================================================")
    print(" MOVE LEFT PANDA TO SAVED BOTTLE POSE")
    print("==========================================================")
    print("Pose file: {}".format(pose_path))
    print("Pose name: {}".format(data.get("name", "unnamed")))
    print("Frame:     {}".format(frame_id))
    print("")
    print(
        "Current XYZ: {:.6f}  {:.6f}  {:.6f}".format(
            start_position[0],
            start_position[1],
            start_position[2]
        )
    )
    print(
        "Target XYZ:  {:.6f}  {:.6f}  {:.6f}".format(
            goal_position[0],
            goal_position[1],
            goal_position[2]
        )
    )
    print("Translation: {:.1f} mm".format(translation * 1000.0))
    print("Rotation:    {:.1f} deg".format(math.degrees(rotation)))
    print("Motion steps: {}".format(number_of_steps))
    print("")

    publisher = rospy.Publisher(
        TARGET_TOPIC,
        PoseStamped,
        queue_size=1,
        latch=True
    )

    rospy.wait_for_service(START_SERVICE, timeout=10.0)
    start_motion = rospy.ServiceProxy(START_SERVICE, Trigger)

    rospy.sleep(0.3)

    for step_index in range(1, number_of_steps + 1):
        fraction = float(step_index) / float(number_of_steps)

        target_position = (
            start_position
            + fraction * (goal_position - start_position)
        )

        target_quaternion = normalise_quaternion(
            quaternion_slerp(
                start_quaternion,
                goal_quaternion,
                fraction
            )
        )

        print(
            "Step {}/{}: target XYZ "
            "{:.6f}  {:.6f}  {:.6f}".format(
                step_index,
                number_of_steps,
                target_position[0],
                target_position[1],
                target_position[2]
            )
        )

        publish_target(
            publisher,
            target_position,
            target_quaternion,
            frame_id
        )


        response = start_motion()

        if not response.success:
            raise RuntimeError(
                "Controller rejected step {}: {}".format(
                    step_index,
                    response.message
                )
            )

        print("  {}".format(response.message))

        reached, position_error, rotation_error = wait_for_target(
            target_position=target_position,
            target_quaternion=target_quaternion,
            position_tolerance=(
                args.position_tolerance_mm / 1000.0
            ),
            rotation_tolerance=math.radians(
                args.rotation_tolerance_deg
            ),
            minimum_elapsed=(
                args.motion_duration + args.settling_time
            ),
            timeout=args.step_timeout
        )

        if not reached:
            raise RuntimeError(
                "Step {} timed out. Position error={:.2f} mm, "
                "rotation error={:.2f} deg.".format(
                    step_index,
                    position_error * 1000.0,
                    math.degrees(rotation_error)
                )
            )

        print(
            "  Step reached: position error={:.2f} mm, "
            "rotation error={:.2f} deg.\n".format(
                position_error * 1000.0,
                math.degrees(rotation_error)
            )
        )

    actual_position, actual_quaternion = get_actual_pose()

    final_position_error = np.linalg.norm(
        goal_position - actual_position
    )

    final_rotation_error = quaternion_angle(
        goal_quaternion,
        actual_quaternion
    )

    print("==========================================================")
    print(" MOTION COMPLETED")
    print("==========================================================")
    print(
        "Desired XYZ: {:.6f}  {:.6f}  {:.6f}".format(
            goal_position[0],
            goal_position[1],
            goal_position[2]
        )
    )
    print(
        "Actual XYZ:  {:.6f}  {:.6f}  {:.6f}".format(
            actual_position[0],
            actual_position[1],
            actual_position[2]
        )
    )
    print(
        "Final position error: {:.2f} mm".format(
            final_position_error * 1000.0
        )
    )
    print(
        "Final rotation error: {:.2f} deg".format(
            math.degrees(final_rotation_error)
        )
    )


if __name__ == "__main__":
    try:
        main()
    except (
        rospy.ROSException,
        FileNotFoundError,
        ValueError,
        RuntimeError
    ) as exception:
        print("ERROR: {}".format(exception), file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nMotion cancelled.")
        sys.exit(130)
