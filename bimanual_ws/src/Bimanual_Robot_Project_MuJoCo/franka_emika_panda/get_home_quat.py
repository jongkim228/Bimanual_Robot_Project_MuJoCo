import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("scene.xml")
data = mujoco.MjData(model)

key_id = model.key("home").id
mujoco.mj_resetDataKeyframe(model, data, key_id)
mujoco.mj_forward(model, data)

def print_ee(site_name):
    site_id = model.site(site_name).id
    pos = data.site_xpos[site_id]
    mat = data.site_xmat[site_id].reshape(3, 3)
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, mat.flatten())
    print(f"--- {site_name} ---")
    print("position:", pos)
    print("quat (w,x,y,z):", quat)
    print("quat (x,y,z,w) for ROS:", [quat[1], quat[2], quat[3], quat[0]])

print_ee("left_ee_site")
print_ee("right_ee_site")
