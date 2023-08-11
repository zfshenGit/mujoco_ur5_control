# IKFastPy - UR5 IKFast Python Package

## Installation and Using


1.Install docker

```
sudo apt update

sudo apt install apt-transport-https ca-certificates curl gnupg2 software-properties-common

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"

sudo apt install docker-ce

```

2.Install openrave image
```
# /home/xxx/test为本地目录挂在地址,请根据实际地址修改
sudo docker run -it -v /home/xxx/test/docker_openrave:/home fishros2/openrave
cd /home
```

3.查看连杆信息
```
openrave-robot.py ur5.dae --info links
```

3.生成Iikfast61.cpp
```
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=ur5.dae --iktype=transform6d --baselink=0 --eelink=13 --savefile=$(pwd)/ikfast61.cpp
```

4.编译测试
```
g++ ikfast61.cpp -o ikfast -llapack -std=c++11
./ikfast 0.05027519 -0.97294776  0.22548849 -0.11273134 0.02407391  0.22688896  0.97362305 0.29398612 -0.99844522 -0.0435207   0.03482954 1.18842854
```

5.生成ikfastpy
复制ikfast61.cpp和/usr/local/lib/python2.7/dist-packages/openravepy/_openravepy_0_53/ikfast.h到ikfastpy文件下
```
python setup.py build_ext --inplace
```

