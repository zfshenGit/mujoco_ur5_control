# mujoco_ur5_control
ur5 robot control with mujoco simulation

## env
ubuntu20.04

## Install


### 1. mujoco210

#### 1.1 下载压缩包，压缩包链接如下

https://link.zhihu.com/?target=https%3A//mujoco.org/download/mujoco210-linux-x86_64.tar.gz 

#### 1.2 移动文件

mkdir ~/.mujoco 

把压缩包里的mujoco210文件夹拷贝到~/.mujoco文件夹下

#### 1.3 环境变量设置

sudo gedit ~/.bashrc

添加export LD_LIBRARY_PATH=/home/user_name/.mujoco/mujoco210/bin，其中user_name需要改称你的用户名
添加export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia

source ~/.bashrc

### 2. anaconda

#### 2.1 下载anaconda

官网下载

sh Anaconda3-2023.03-1-Linux-x86_64.sh进行安装

#### 2.2 环境变量设置

添加export PATH=/home/user_name/anaconda3/bin:$PATH，其中user_name需要改称你的用户名

source ~/.bashrc

#### 2.3 新建虚拟环境

conda create -n mujoco_ur5 python=3.6

conda activate mujoco_ur5

#### 2.4 安装mujoco_py

pip install mujoco_py

