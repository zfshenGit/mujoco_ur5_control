import mujoco_py as mp
from joint_ipd import torque_joint
from cartesian_ipd import torque_cartesian
import func
import numpy as np
import math

class UR5_Control:
    def __init__(self, sim, viewer):
        self.sim = sim
        self.viewer = viewer

    def get_jacobian(self):
        jacobian_pos = self.sim.data.get_body_jacp('ee_link').reshape(3, 6)
        jacobian_ori = self.sim.data.get_body_jacr('ee_link').reshape(3, 6)
        J_full = np.array(np.vstack([jacobian_pos, jacobian_ori]))
        return J_full

    def get_jacobian_pinv(self):
        J_full = self.get_jacobian()
        return np.linalg.pinv(J_full)

    def get_coriolis_gravity(self):
        return self.sim.data.qfrc_bias[:]

    def joint_velocity_control(self, step_nums, target_vel):
        for i in range(step_nums):
            coriolis_gravity = self.get_coriolis_gravity()
            self.sim.data.ctrl[:] = coriolis_gravity
            self.sim.data.qvel[:] = target_vel

            self.sim.step()
            self.viewer.render()

    def cartesian_velocity_control(self, step_nums, target_vel):
        for i in range(step_nums):
            jacobian_pinv = self.get_jacobian_pinv()
            joint_vel = np.dot(jacobian_pinv, target_vel)

            coriolis_gravity = self.get_coriolis_gravity()
            self.sim.data.ctrl[:] = coriolis_gravity
            self.sim.data.qvel[:] = joint_vel

            self.sim.step()
            self.viewer.render()
        


if __name__ == '__main__':
    model = mp.load_model_from_path('./my_ur5/my_ur5_2.xml')
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

    print('cartesian_velocity_control start!')
    target_vel = np.array([0.0, 0.0, 0.0, 0.0, 0.1, 0.0])
    cartesian_velocity_control(2000, target_vel)

    print('joint_velocity_control start!')
    target_vel = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    joint_velocity_control(2000, target_vel)
    
