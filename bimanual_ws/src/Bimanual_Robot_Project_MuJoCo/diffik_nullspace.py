import mujoco
import numpy as np
from init import *

def reached(site_id, target_pos, target_quat, pos_tol = 0.05, ori_tol = 0.15):
    current_pos = data.site(site_id).xpos
    pos_err = target_pos - current_pos
    pos_dist = np.linalg.norm(pos_err)

    site_quat = np.zeros(4)
    site_quat_conj = np.zeros(4)
    error_quat = np.zeros(4)
    error_vel = np.zeros(3)

    mujoco.mju_mat2Quat(site_quat, data.site(site_id).xmat)
    mujoco.mju_negQuat(site_quat_conj, site_quat)
    mujoco.mju_mulQuat(error_quat, target_quat, site_quat_conj)
    mujoco.mju_quat2Vel(error_vel, error_quat, 1.0)
    ori_dist = np.linalg.norm(error_vel)

    if pos_dist < pos_tol and ori_dist < ori_tol:
        return True
    return False


def ik_nullspace(target_pos, target_quat, site_id, dof_ids, jnt_ids, eye, qpos_ids, actuator_ids, feedforward_twist=None):
    # Spatial velocity (aka twist).
    dx = target_pos - data.site(site_id).xpos
    twist[:3] = (Kpos * dx) / integration_dt
    mujoco.mju_mat2Quat(site_quat, data.site(site_id).xmat)
    mujoco.mju_negQuat(site_quat_conj, site_quat)
    mujoco.mju_mulQuat(error_quat, target_quat, site_quat_conj)
    mujoco.mju_quat2Vel(twist[3:], error_quat, 1.0)
    twist[3:] *= Kori / integration_dt

    if feedforward_twist is not None:
        twist[:] += feedforward_twist


    mujoco.mj_jacSite(model, data, jac[:3], jac[3:], site_id)
    jac_sub = jac[:, dof_ids]


    # Damped least squares.
    dq = jac_sub.T @ np.linalg.solve(jac_sub @ jac_sub.T + diag, twist)

    # Nullspace control biasing joint velocities towards the home configuration.
    dq += (eye - np.linalg.pinv(jac_sub) @ jac_sub) @ (Kn * (q0[qpos_ids] - data.qpos[qpos_ids]))


    # Clamp maximum joint velocity.
    dq_abs_max = np.abs(dq).max()
    if dq_abs_max > max_angvel:
        dq *= max_angvel / dq_abs_max

    dq_full = np.zeros(model.nv)
    dq_full[dof_ids] = dq

    q = data.qpos.copy()
    mujoco.mj_integratePos(model, q, dq_full, integration_dt)
    
    q[qpos_ids] = np.clip(q[qpos_ids], model.jnt_range[jnt_ids, 0], model.jnt_range[jnt_ids, 1])
    
    data.ctrl[actuator_ids] = q[qpos_ids]

    return twist.copy()
