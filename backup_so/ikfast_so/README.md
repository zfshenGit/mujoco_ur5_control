# 1.编译产生ikfastpy.so文件
将./ikfastpy里的ikfast61.cpp替换为./ikfast里的ikfast61.cpp
	
激活虚拟环境,例如conda activate mujoco210	

ikfastpy路径下执行python setup.py build_ext --inplace,将编译生成对应虚拟环境的so文件,例如ikfastpy.cpython-38-x86_64-linux-gnu.so文件

将ikfastpy.cpython-38-x86_64-linux-gnu.so重命名为ikfastpy.so,复制到import路径下,即可import ikfastpy成功

当前路径下,已经编译生成了python3.6,3.7,3.8三种环境对应的so文件
