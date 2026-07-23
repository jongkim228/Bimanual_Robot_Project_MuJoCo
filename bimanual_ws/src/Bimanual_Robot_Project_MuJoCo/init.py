import numpy as np
import mujoco

# Load the model and data.
model = mujoco.MjModel.from_xml_path("franka_emika_panda/scene.xml")
data = mujoco.MjData(model)


# Integration timestep in seconds. This corresponds to the amount of time the joint
# velocities will be integrated for to obtain the desired joint positions.
integration_dt: float = 0.1


# Damping term for the pseudoinverse. This is used to prevent joint velocities from
# becoming too large when the Jacobian is close to singular.
damping: float = 1e-4

# Gains for the twist computation. These should be between 0 and 1. 0 means no
# movement, 1 means move the end-effector to the target in one integration step.
Kpos: float = 0.95
Kori: float = 0.95

# Whether to enable gravity compensation.
gravity_compensation: bool = True


# Simulation timestep in seconds.
dt: float = 0.002

# Nullspace P gain.
Kn = np.asarray([10.0, 10.0, 10.0, 10.0, 5.0, 5.0, 5.0])


# Maximum allowable joint velocity in rad/s.
max_angvel = 0.785

# Enable gravity compensation. Set to 0.0 to disable.
model.body_gravcomp[:] = float(gravity_compensation)
model.opt.timestep = dt


# Get the dof and actuator ids for the joints we wish to control. These are copied
# from the XML file. Feel free to comment out some joints to see the effect on
# the controller.
joint_names_r = [
        "right_joint1",
        "right_joint2",
        "right_joint3",
        "right_joint4",
        "right_joint5",
        "right_joint6",
        "right_joint7"
    ]

joint_names_l = [
        "left_joint1",
        "left_joint2",
        "left_joint3",
        "left_joint4",
        "left_joint5",
        "left_joint6",
        "left_joint7"
    ]

jnt_ids_r = np.array([model.joint(name).id for name in joint_names_r])
jnt_ids_l = np.array([model.joint(name).id for name in joint_names_l])

dof_ids_r = np.array([model.jnt_dofadr[j] for j in jnt_ids_r])
dof_ids_l = np.array([model.jnt_dofadr[j] for j in jnt_ids_l])

qpos_ids_r = np.array([model.jnt_qposadr[j] for j in jnt_ids_r])
qpos_ids_l = np.array([model.jnt_qposadr[j] for j in jnt_ids_l])

actuator_names_r = [f"right_actuator{i}" for i in range(1, 8)]
actuator_names_l = [f"left_actuator{i}" for i in range(1, 8)]
actuator_ids_r = np.array([model.actuator(name).id for name in actuator_names_r])
actuator_ids_l = np.array([model.actuator(name).id for name in actuator_names_l])

# End-effector site we wish to control.
site_name_r = "right_attachment_site"
site_name_l = "left_attachment_site"
site_id_r = model.site(site_name_r).id
site_id_l = model.site(site_name_l).id

key_name = "home"
key_id = model.key(key_name).id

# Pre-allocate numpy arrays.
jac = np.zeros((6, model.nv))
diag = damping * np.eye(6)
eye_r = np.eye(len(dof_ids_r))
eye_l = np.eye(len(dof_ids_l))
twist = np.zeros(6)
site_quat = np.zeros(4)
site_quat_conj = np.zeros(4)
error_quat = np.zeros(4)

qpos_ids_r = model.jnt_qposadr[dof_ids_r]
qpos_ids_l = model.jnt_qposadr[dof_ids_l]
q0 = np.zeros(model.nq)
q0[qpos_ids_r] = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853]
q0[qpos_ids_l] = [0, 0, 0, -1.57079, 0, 1.57079, -0.7853] 

