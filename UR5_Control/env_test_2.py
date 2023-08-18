'''
包含功能：
加载xml文件, 关节初始化, 正运动学, 逆运动学, 关节速度控制, 笛卡尔速度控制
'''

import mujoco_py as mp
import numpy as np
import ikfastpy
import utils
import time

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

def joint_velocity_control(step_nums, joint_target_vel):
    for i in range(step_nums):
        coriolis_gravity = get_coriolis_gravity()
        sim.data.ctrl[:] = coriolis_gravity
        sim.data.qvel[:] = joint_target_vel

        sim.step()
        viewer.render()

def cartesian_velocity_control(step_nums, cartesian_target_vel):
    for i in range(step_nums):
        jacobian_pinv = get_jacobian_pinv()
        joint_vel = np.dot(jacobian_pinv, cartesian_target_vel)

        coriolis_gravity = get_coriolis_gravity()
        sim.data.ctrl[:] = coriolis_gravity
        sim.data.qvel[:] = joint_vel

        sim.step()
        viewer.render()

def joint_position_init(joint_value):
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    for i in range(6):
        sim.data.set_joint_qpos(joint_names[i], joint_value[i])
    sim.forward()

def forward_kinematics(end_effector):
    pos = sim.data.get_body_xpos(end_effector)
    ori = np.array(sim.data.body_xmat[sim.model.body_name2id(end_effector)].reshape([3, 3]))         # 四元数, 3*3旋转矩阵可用sim.data.get_body_xquat(end_effector) 
    return pos, ori

def inverse_kinematics(init, pos, ori):
    solver = ikfastpy.PyKinematics()
    n_joints = solver.getDOF()

    pos, ori, ok = utils.check_pos_ori_valid(pos, ori)
    if not ok:
        return []
    ee_pos = np.append(ori, pos, axis=1)
    joint_configs = solver.inverse(ee_pos.reshape(-1).tolist())
    n_solutions = int(len(joint_configs) / n_joints)
    joint_configs = np.asarray(joint_configs).reshape(n_solutions, n_joints)
    min_norm = np.inf
    res = []
    for i in range(n_solutions):
        norm = np.linalg.norm(joint_configs[i] - init)
        if norm < min_norm:
            res = joint_configs[i]
            min_norm = norm
    return res

def admittance_control():
    BF_Train = np.array([10.0, 10.0])
    BT_Train = np.array([1.0, 1.0])
    MF = np.array([[5.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, 5.0]])
    BF = np.array([[BF_Train[0], 0.0, 0.0], [0.0, BF_Train[1], 0.0], [0.0, 0.0, 1000.]])
    MT = np.array([[0.2, 0.0, 0.0], [0.0, 0.2, 0.0], [0.0, 0.0, 0.2]]) 
    BT = np.array([[BT_Train[0], 0.0, 0.0], [0.0, BT_Train[1], 0.0], [0.0, 0.0, 10.]])

    c_Force_Target = np.array([[0.0, 0.0, 10.0]]).transpose()
    c_Torque_Target = np.array([[0.0, 0.0, 0.0]]).transpose()

    # force_torque_read = p.getJointState(self.pandaId,self.ftSensorJointIndex)[2]
    # cur_C_Force = np.array([force_torque_read[0:3]]).transpose()
    # cur_C_Torque = np.array([force_torque_read[3:6]]).transpose()

    # rotate_angle = p.getLinkState(self.pandaId, self.peg_link)[1]
    # r_m_f = p.getMatrixFromQuaternion(rotate_angle)
    # rotate_matrix = np.array([[r_m_f[0],r_m_f[1],r_m_f[2]], [r_m_f[3],r_m_f[4],r_m_f[5]], [r_m_f[6],r_m_f[7],r_m_f[8]]])
    # #这里可能需要修改
    # transfom_matrix = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
    # rotate_matrix = np.matmul(rotate_matrix, transfom_matrix)

    # force_temp = cur_C_Force

    # #注释1 这里可能是一个问题 力的坐标系问题,大问题应该也没有
    # cur_C_Force = np.matmul(rotate_matrix, cur_C_Force)
    # cur_C_Torque = np.matmul(rotate_matrix, cur_C_Torque)

    # error_force = c_Force_Target - cur_C_Force
    # error_torque = c_Torque_Target - cur_C_Torque

# def jonit_position_control(step_nums, joint_target_pos):
#     for i in range(step_nums):
#         coriolis_gravity = get_coriolis_gravity()
#         sim.data.ctrl[:] = coriolis_gravity
#         sim.data.qpos[:] = joint_target_pos

#         sim.step()
#         viewer.render()

if __name__ == '__main__':
    model = mp.load_model_from_path('./urdf/ur5.xml')
    sim = mp.MjSim(model)
    viewer = mp.MjViewer(sim)

    print('joint init!')
    joint_value = [0.343903, -1.79413, -2.079, -1.63227, -1.87035, -1.03724]
    joint_position_init(joint_value)
    print('joint value init:', joint_value)

    print('forward kenematic!')
    pos, ori = forward_kinematics('ee_link')    # end effector is ee_link
    print('pos:', pos)
    print('ori:', ori)

    print('inverse kenematic!')
    joint_value_copy = joint_value.copy()
    
    start = time.time()
    inverse = inverse_kinematics(joint_value_copy, pos, ori) 
    end = time.time()
    print('inverse pos:', inverse)
    print('cost time:', end - start)

    # print('cartesian_velocity_control start!')
    # target_vel = np.array([0.0, 0.0, 0.0, 0.0, 0.1, 0.0])
    # cartesian_velocity_control(2000, target_vel)

    # print('joint_velocity_control start!')
    # target_vel = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    # joint_velocity_control(2000, target_vel)
    
