import mujoco_py as mp
import numpy as np
import gym
from gym import spaces

class PegInHole(gym.Env):
    def __init__(self):


def get_jacobian():
    jacobian_pos = sim.data.get_body_jacp('ee_link').reshape(3, 6)
    jacobian_ori = sim.data.get_body_jacr('ee_link').reshape(3, 6)
    J_full = np.array(np.vstack([jacobian_pos, jacobian_ori]))
    return J_full

def get_jacobian_pinv():
    J_full = get_jacobian()
    return np.linalg.pinv(J_full)

def get_coriolis_gravity():
    return sim.data.qfrc_bias[:]

def cartesian_velocity_control(step_nums, target_vel):
    for i in range(step_nums):
        jacobian_pinv = get_jacobian_pinv()
        joint_vel = np.dot(jacobian_pinv, target_vel)

        coriolis_gravity = get_coriolis_gravity()
        sim.data.ctrl[:] = coriolis_gravity
        sim.data.qvel[:] = joint_vel

        sim.step()
        viewer.render()


if __name__ == '__main__':
    model = mp.load_model_from_path('./ur5/ur5.xml')
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
    home_pos = pos.copy()
    home_ori = ori.copy()
    desired_pos = pos.copy()
    desired_ori = ori.copy()
    kj, dj = np.array([20] * 6, dtype=np.float32), np.array([100] * 6, dtype=np.float32)
    # kc = np.array([300, 300, 300, 600, 600, 600], dtype=np.float32)
    kc = np.array([300, 300, 300, 600, 600, 600], dtype=np.float32)
    dc = np.array([120, 120, 120, 70, 70, 70], dtype=np.float32)
    steps = 0
    body_id = sim.model.body_name2id('wrist_3_link')

    target_vel = np.array([0.0, 0.0, 0.0, 0.0, 0.1, 0.0])
    cartesian_velocity_control(1000, target_vel)
