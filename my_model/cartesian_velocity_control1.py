import mujoco_py as mp
import numpy as np

# 加载模型
model = mp.load_model_from_path('./my_ur5/my_ur5_mocap.xml')
sim = mp.MjSim(model)
viewer = mp.MjViewer(sim)

# 设置初始状态
qpos = np.array([0.343903, -1.79413, -2.079, -1.63227, -1.87035, -1.03724])
qvel = np.zeros_like(qpos)
sim.data.qpos[:] = qpos
sim.data.qvel[:] = qvel
sim.forward()

# 控制循环
target_vel = np.array([0.0, 0.0, 0.0])  # 目标末端执行器速度

while True:
    # 计算当前速度
    current_vel = sim.data.get_body_xvelp('ee_link')

    # 计算速度误差
    vel_error = target_vel - current_vel

    # 计算末端执行器速度控制命令
    jacobian = sim.data.get_body_jacp('ee_link').reshape(3, 6)
    print('jacobian:', jacobian)
    joint_vel = np.dot(np.linalg.pinv(jacobian), vel_error)

    # 设置关节速度
    sim.data.qvel[:] = joint_vel

    # 执行一步仿真
    sim.step()
    viewer.render()

    # # 检查是否达到目标速度
    # if np.linalg.norm(current_vel - target_vel) < 0.01:
    #     break

# 输出最终关节角度
final_joint_angles = sim.data.qpos[:]
print('Final joint angles:', final_joint_angles)