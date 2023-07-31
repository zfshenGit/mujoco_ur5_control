import mujoco_py as mp
import numpy as np
import math
from joint_ipd import torque_joint
from cartesian_ipd import torque_cartesian
import func
import time

if __name__ == '__main__':
    model = mp.load_model_from_path('./my_ur5/my_ur5.xml')
    sim = mp.MjSim(model)
    viewer = mp.MjViewer(sim)
    Rbt = func.Robot(sim, 'ee')
    init = [0.343903, -1.79413, -2.079, -1.63227, -1.87035, -1.03724]
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

    start_time = time.time()

    while 1:

        # desired_pos[0] = pos[0] + 0.1 * math.cos(steps / 180 * np.pi)
        # desired_pos[1] = pos[1] + 0.1 * math.sin(steps / 180 * np.pi)

        curr_time = time.time()
        if (curr_time - start_time) < 10:
            desired_pos[0] = pos[0]
            desired_pos[1] = pos[1]
            # print("pos[0]:",pos[0])
            # print("pos[1]:",pos[1])

            # tau, ok = torque_joint(Rbt, sim, kj, dj, desired_pos, desired_ori, last_tau)
            tau = torque_cartesian(Rbt, sim, kc, dc, 'ee', desired_pos, desired_ori)
            # if 1000 <= steps < 1100:
            #     sim.data.xfrc_applied[body_id][0] = 100
            # else:
            #     sim.data.xfrc_applied[body_id][0] = 0
            sim.data.ctrl[:] = tau
            last_tau = tau.copy()
        elif 10 <(curr_time - start_time) < 20:
            sim.data.ctrl[:] = 10
        else:
            sim.data.ctrl[:] = 0
        sim.step()
        steps += 1
        viewer.render()
