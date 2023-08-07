from scipy.spatial.transform import Rotation as r
import numpy as np

def quat2R(quat):
    Rm = r.from_quat(quat)
    rotation_matrix = Rm.as_matrix()
    return rotation_matrix

def world2base_link():
    pos = [-0.250, -0.000, 1.650]
    ori = [-0.354, 0.854, 0.146, -0.354]    # in RPY (degree) [144.736, -30.000, -125.264], in RPY (radian) [2.526, -0.524, -2.186]
    return pos, ori

def quat2euler(quat):
    Rm = r.from_quat(quat)
    euler = Rm.as_euler('zyz', degrees=True)
    return euler

def inverse_pose(pos, ori):
    rotation_matrix = quat2R(ori)
    inverse_rotation = np.transpose(rotation_matrix)
    inverse_translation = -np.dot(inverse_rotation, pos)
    return inverse_translation, inverse_rotation



pos, ori = world2base_link()
print('---------------------------')
print('pos', pos)
print('ori', ori)

rotation_matrix = quat2R(ori)
print('---------------------------')
print('rotation_matrix:', rotation_matrix)

euler = quat2euler(ori)
print('---------------------------')
print('euler zyz:', euler)

# base_link2world
inverse_pos, inverse_ori = inverse_pose(pos, ori)
print('---------------------------')
print('inverse_pose:', inverse_pos)
print('inverse_ori:', inverse_ori)

# world下末端点坐标
init_pos = np.array([-0.11686899, 0.29200483, 1.27060058])
init_ori = [
    [0.97294776, 0.05027519, 0.22548849],
    [-0.22688896, 0.02407391, 0.97362305],
    [0.0435207, -0.99844522, 0.03482954]
]

trans = r.from_matrix(init_ori)
trans = r.as_matrix(trans)

# init_pos, init_ori转换到base_link
trans_pos = np.dot(inverse_ori, init_pos) + inverse_pos
trans_ori = np.dot(inverse_ori, init_ori)
print('trans_pos:', trans_pos)
print('trans_ori', trans_ori)