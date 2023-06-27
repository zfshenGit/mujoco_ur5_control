import func_mj
import mujoco_py as mp 

if __name__ == "__main__":
    model = mp.load_model_from_path('ur5.xml')
    sim = mp.MjSim(model)
    rbtmj = func_mj.RobotMj(sim,'ee')
    print(rbtmj.mass_matrix())
    print(rbtmj.coriolis_gravity())