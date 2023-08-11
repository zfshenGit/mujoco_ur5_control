from scipy.spatial.transform import Rotation as R
import numpy as np

r = R.from_rotvec(np.pi/4 * np.array([0, -1, 0]))
rotation_matrix = r.as_matrix()
print('rotation_matrix:', rotation_matrix)

pos = [0,0,1]
pos1 = np.dot(rotation_matrix, pos)
print('pos1:', pos1)