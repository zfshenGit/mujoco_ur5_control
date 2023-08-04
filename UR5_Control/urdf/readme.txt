1.ur5_origin.urdf修改mesh路径保存为ur5.urdf

2.urdf添加mujoco标签
<mujoco>
    <compiler
    meshdir="./ur5/collision"
    balanceinertia="true"
    discardvisual="false"/>
</mujoco>

3.在mujoco bin路径下./compile ur5.urdf ur5.xml生成ur5.xml
