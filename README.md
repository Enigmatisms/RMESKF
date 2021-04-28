## ESKF

---

2021 RM 自动步兵的多传感器融合定位：

- UWB定位为位姿观测（2D位置+朝向，低频）
- IMU为速度状态 / 角度状态 / 位置状态
- 轮速为速度观测

---

### 现有的问题

- 轮速融合：轮速融合结果需要和电控进行联调（通信 并且确定解算的结果正确（方向上正确））
- UWB与intializer：初始化问题，state->G_R_I需从UWB initializer处获得初始的角度，然后不断通过UWB修正      <6,15>大型ESKF
- UWB角度修正的算法部分需要审查一下公式正确性
- 有几个角度以及坐标系相对关系：
  - 轮速里面，云台相对底盘的角度
  - UWB给的角度，这个还没有用过，但好像现在我有这个东西，所以自己可以看一下
  - 电控发的积分角度，这个正负定义以及范围大小
