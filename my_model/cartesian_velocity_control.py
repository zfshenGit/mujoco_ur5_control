import mujoco_py as mp
import numpy as np
import time

# 加载模型
model = mp.load_model_from_path('./my_ur5/my_ur5_mocap.xml')
sim = mp.MjSim(model)
viewer = mp.MjViewer(sim)

# 将机械臂移动到初始位置
q0 = np.array([0.343903, -1.79413, -2.079, -1.63227, -1.87035, -1.03724])
sim.data.qpos[:] = q0
sim.step()

# 定义控制周期和控制频率
dt = 0.01
freq = 1 / dt

# 定义控制循环
while True:
    # 计算目标末端执行器位姿
    x_dot = np.array([0.001, 0, 0])
    tcp_pose = sim.data.get_body_xpos('ee_link')
    tcp_quat = sim.data.get_body_xquat('ee_link')
    tcp_velp = sim.data.get_body_xvelp('ee_link')
    tcp_velr = sim.data.get_body_xvelr('ee_link')
    J = sim.data.get_body_jacp('ee_link').reshape((3, 6))
    Jinv = np.linalg.pinv(J)
    q_dot = np.dot(Jinv, x_dot)
    tcp_pose_new = tcp_pose + tcp_velp * dt + tcp_velr * dt
    sim.data.set_mocap_pos('mocap', tcp_pose_new)
    sim.data.set_mocap_quat('mocap', tcp_quat)

    # 控制机械臂运动到目标位置
    for i in range(int(freq * dt)):
        sim.data.ctrl[:] = q_dot
        sim.step()
        viewer.render()
        time.sleep(1)
    


    # 检查机械臂是否到达目标位置，如果到达则退出循环
    tcp_pose_new = sim.data.get_body_xpos('ee_link')
    if np.linalg.norm(tcp_pose_new - tcp_pose) < 0.001:
        break

# # 断开仿真器连接
# sim.close()
