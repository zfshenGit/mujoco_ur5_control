当前路径下,已经编译生成了python3.6,3.7,3.8三种环境对应的so文件

下面操作是自己编译生成对应的so文件

# 1. c文件编译
```commandline
cd orocos_kinematics_dynamics/orocos_kdl
mkdir build
cd build
cmake ..
make
sudo make install
```

# 2.py编译生成so文件
首先激活虚拟环境,在虚拟环境中编译,即cmke ..保证使用的虚拟环境的python文件

```commandline
conda activate mujoco_ur5
```

编译

```commandline
cd orocos_kinematics_dynamics/python_orocos_kdl
mkdir build
cd build
cmake ..
make
```

如果系统使用的不是虚拟环境的python文件,首先查询虚拟环境python路径

```commandline
python
import sys
print(sys.executable)
```

然后cmke ..使用下列命令编译
```commandline
cmake ../ -DPYTHON_EXECUTABLE:FILEPATH=/home/amax/anaconda3/envs/mujoco210/bin/python
```

最后可在devel/lib里找到对应的so文件
