import mujoco
import mujoco.viewer
import numpy as np
import time
from init import *
from diffik_nullspace import reached, ik_nullspace


STATES = {
    "APPROACH": {"pos": np.array([0.500, 0.2, 0.35])},
    "PICKUP": {"pos": np.array([0.500, 0.2, 0.6])},
}

move_y_values = np.arange(0, 0.4 + 0.001, 0.02)
forward = [np.array([0.500, y, 0.6]) for y in move_y_values]
backward = forward[-2:0:-1]
MOVE_WAYPOINTS = forward + backward

move_idx = 0
wait_start_time = None

offset = np.array([0, 0.1, 0.25])

right_quat = np.array([0.5, 0.5, -0.5, -0.5])


q_target_l = np.zeros(4)
mujoco.mju_mat2Quat(q_target_l, data.site(site_id_l).xmat)


with mujoco.viewer.launch_passive(model=model,
    data=data,
    show_left_ui=False,
    show_right_ui=False,) as viewer:

    mujoco.mjv_defaultFreeCamera(model, viewer.cam)
    viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE

    mujoco.mj_resetData(model, data)
    data.qpos[qpos_ids_r] = q0[qpos_ids_r]
    data.qpos[qpos_ids_l] = q0[qpos_ids_l]
    mujoco.mj_forward(model, data)

    state = "APPROACH"
    target_pos_l = data.site_xpos[site_id_l].copy()

    dof_ids_r = np.array([model.jnt_dofadr[model.joint(name).id] for name in joint_names_r])
    print("nq:", model.nq, "nv:", model.nv)

    while viewer.is_running():

        step_start = time.time()
        gravity_compensation: bool = True

        if state == "APPROACH":
            target_pos_r = STATES["APPROACH"]["pos"]
            if reached(site_id_r, target_pos_r, right_quat):
                state = "PICKUP"

        elif state == "PICKUP":
            target_pos_r = STATES["PICKUP"]["pos"]
            if reached(site_id_r, target_pos_r, right_quat):
                state = "ACTIVE_LEFT_ARM"

        elif state == "ACTIVE_LEFT_ARM":
            right_ee_pos = data.site_xpos[site_id_r].copy()
            target_pos_l = right_ee_pos + offset
            if reached(site_id_l, target_pos_l, q_target_l):
                if wait_start_time is None:
                    wait_start_time = time.time()
                elif time.time() - wait_start_time >= 1.0:
                    state = "MOVE"

        elif state == "MOVE":
            target_pos_r = MOVE_WAYPOINTS[move_idx % len(MOVE_WAYPOINTS)]
            right_ee_pos = data.site_xpos[site_id_r].copy()
            target_pos_l = right_ee_pos + offset

            if reached(site_id_r, target_pos_r, right_quat) and reached(site_id_l, target_pos_l, q_target_l):
                move_idx += 1

        #right_ik
        twist_r = ik_nullspace(target_pos_r, right_quat, site_id_r, dof_ids_r, jnt_ids_r, eye_r, qpos_ids_r, actuator_ids_r)
        ik_nullspace(target_pos_l, q_target_l, site_id_l, dof_ids_l, jnt_ids_l, eye_l, qpos_ids_l, actuator_ids_l, feedforward_twist=twist_r)

        mujoco.mj_forward(model, data)
        mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id_r)

        site_q = np.zeros(4)
        mujoco.mju_mat2Quat(site_q, data.site(site_id_r).xmat)


        
        mujoco.mj_step(model, data)
        viewer.sync()

        time_until_next_step = dt - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)