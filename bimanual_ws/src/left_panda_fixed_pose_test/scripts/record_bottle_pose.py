#!/usr/bin/env python3

import argparse
import os
import sys
from datetime import datetime

import numpy as np
import rospy
import yaml

from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from franka_msgs.msg import FrankaState
from tf.transformations import quaternion_from_matrix


STATE_TOPIC = "/panda_left/franka_state_controller/franka_states"
SWITCH_SERVICE = "/panda_left/controller_manager/switch_controller"
CONTROLLER_NAME = "left_fixed_pose_controller"
DEFAULT_FRAME = "panda_left_link0"


def switch_controller(start=None, stop=None):
    start = start or []
    stop = stop or []

    rospy.wait_for_service(SWITCH_SERVICE, timeout=10.0)
    switch = rospy.ServiceProxy(SWITCH_SERVICE, SwitchController)

    request = SwitchControllerRequest()
    request.start_controllers = start
    request.stop_controllers = stop
    request.strictness = SwitchControllerRequest.BEST_EFFORT
    request.start_asap = False
    request.timeout = 5.0

    response = switch(request)
    return response.ok


def read_actual_pose():
    state = rospy.wait_for_message(
        STATE_TOPIC,
        FrankaState,
        timeout=10.0
    )

    transform = np.asarray(
        state.O_T_EE,
        dtype=float
    ).reshape((4, 4), order="F")

    quaternion = quaternion_from_matrix(transform)

    return transform, quaternion


def sanitise_name(name):
    allowed = []

    for character in name:
        if character.isalnum() or character in ("-", "_"):
            allowed.append(character)
        else:
            allowed.append("_")

    return "".join(allowed).strip("_")


def main():
    parser = argparse.ArgumentParser(
        description="Kinesthetically teach and save a Panda end-effector pose."
    )

    parser.add_argument(
        "--name",
        default="",
        help="Optional pose name, for example bottle_approach."
    )

    parser.add_argument(
        "--output-dir",
        default=os.path.expanduser(
            "~/bimanual_ws/src/"
            "left_panda_fixed_pose_test/config/taught_poses"
        )
    )

    parser.add_argument(
        "--no-restart-controller",
        action="store_true",
        help="Do not restart the Cartesian controller after recording."
    )

    args = parser.parse_args()

    rospy.init_node("record_bottle_pose", anonymous=True)

    os.makedirs(args.output_dir, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if args.name:
        pose_name = sanitise_name(args.name)
        filename = "{}_{}.yaml".format(pose_name, timestamp)
    else:
        pose_name = "bottle_pose"
        filename = "bottle_pose_{}.yaml".format(timestamp)

    output_path = os.path.join(args.output_dir, filename)
    latest_path = os.path.join(args.output_dir, "latest.yaml")

    print("")
    print("Stopping the Cartesian controller...")
    switch_controller(stop=[CONTROLLER_NAME])

    print("")
    print("==========================================================")
    print(" KINESTHETIC BOTTLE-POSE TEACHING")
    print("==========================================================")
    print("")
    print("1. Put the Panda into hand-guiding mode in Franka Desk.")
    print("2. Move the end effector to the desired bottle pose.")
    print("3. Hold the robot stationary.")
    print("4. Press ENTER here to record the pose.")
    print("")
    input("Press ENTER to save the current pose: ")

    transform, quaternion = read_actual_pose()

    position = transform[:3, 3]

    data = {
        "name": pose_name,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "robot": "panda_left",
        "frame_id": DEFAULT_FRAME,
        "description": (
            "Kinesthetically taught end-effector pose associated "
            "with the fixed bottle."
        ),
        "position": {
            "x": float(position[0]),
            "y": float(position[1]),
            "z": float(position[2]),
        },
        "orientation": {
            "x": float(quaternion[0]),
            "y": float(quaternion[1]),
            "z": float(quaternion[2]),
            "w": float(quaternion[3]),
        },
        "O_T_EE_column_major": [
            float(value) for value in transform.flatten(order="F")
        ],
    }

    with open(output_path, "w", encoding="utf-8") as output_file:
        yaml.safe_dump(
            data,
            output_file,
            default_flow_style=False,
            sort_keys=False
        )

    try:
        if os.path.lexists(latest_path):
            os.remove(latest_path)

        os.symlink(filename, latest_path)

    except OSError:
        with open(latest_path, "w", encoding="utf-8") as output_file:
            yaml.safe_dump(
                data,
                output_file,
                default_flow_style=False,
                sort_keys=False
            )

    print("")
    print("Pose saved successfully:")
    print("  {}".format(output_path))
    print("")
    print(
        "Position XYZ: "
        "{:.6f}  {:.6f}  {:.6f}".format(
            position[0],
            position[1],
            position[2]
        )
    )
    print(
        "Quaternion:   "
        "{:.6f}  {:.6f}  {:.6f}  {:.6f}".format(
            quaternion[0],
            quaternion[1],
            quaternion[2],
            quaternion[3]
        )
    )
    print("")
    print("Latest pose:")
    print("  {}".format(latest_path))

    if not args.no_restart_controller:
        print("")
        print("Restarting the Cartesian controller...")

        if switch_controller(start=[CONTROLLER_NAME]):
            print("Controller restart requested successfully.")
        else:
            print(
                "WARNING: controller could not be restarted.",
                file=sys.stderr
            )
            print(
                "Make sure FCI is active and the robot is not User Stopped.",
                file=sys.stderr
            )


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as exception:
        print("ROS error: {}".format(exception), file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nCancelled.")
        sys.exit(130)
