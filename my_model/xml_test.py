import mujoco_py as mp
# import time

# load model
# model = mp.load_model_from_path('./ur5_xml/ur5.xml')
model = mp.load_model_from_path('./my_ur5/my_ur5.xml')
sim = mp.MjSim(model)
viewer = mp.MjViewer(sim)

# begin_time = time.clock()
# step and render
for i in range(30000):
    # sim.data.ctrl[:6] = 1
    sim.step()
    viewer.render()

# end_time = time.clock()
# print('time cost:', (begin_time - end_time), 's')