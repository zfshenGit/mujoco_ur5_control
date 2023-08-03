import mujoco_py as mp
from joint_ipd import torque_joint
from cartesian_ipd import torque_cartesian
import func
import numpy as np
import math

if __name__ == '__main__':
    model = mp.load_model_from_path('ur5.xml')
    sim = mp.MjSim(model)
    viewer = mp.MjViewer(sim)
    Rbt = func.Robot(sim, 'ee')
    init = [-3.1, -1.6, 1.6, -1.6, -1.6, 0]
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    for i in range(6):
        sim.data.set_joint_qpos(joint_names[i], init[i])
    sim.forward()
    pos, ori = Rbt.fk()
    desired_pos = pos.copy()
    desired_ori = ori.copy()
    kj, dj = np.array([20] * 6, dtype=np.float32), np.array([100] * 6, dtype=np.float32)
    kc = np.array([300, 300, 300, 600, 600, 600], dtype=np.float32)
    dc = np.array([120, 120, 120, 70, 70, 70], dtype=np.float32)
    last_tau = 0
    steps = 0
    body_id = sim.model.body_name2id('wrist_3_link')

    while 1:
        desired_pos[0] = pos[0] + 0.1 * math.cos(steps / 180 * np.pi)
        desired_pos[1] = pos[1] + 0.1 * math.sin(steps / 180 * np.pi)
        # tau, ok = torque_joint(Rbt, sim, kj, dj, desired_pos, desired_ori, last_tau)
        tau = torque_cartesian(Rbt, sim, kc, dc, 'ee', desired_pos, desired_ori)
        if 1000 <= steps < 1100:
            sim.data.xfrc_applied[body_id][0] = 100
        else:
            sim.data.xfrc_applied[body_id][0] = 0
        sim.data.ctrl[:] = tau
        last_tau = tau.copy()
        sim.step()
        steps += 1
        viewer.render()
