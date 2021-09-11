# 程序说明
该程序用于测试激光雷达和相机的外参是否准确。在config中可以给定外参，在opencv的画面中，将会实时地出现激光点云在相机FOV中的投影点，根据图像中环境的几何一致性，用户可以对外参的精度有一个定性的了解。用户还通过Dynamic reconfiguraton功能实时改变外参，以便微调外参。

具体的参数配置及其含义见于config/params.yaml，验证过程中的每一帧图像都会保存至image下

传感器型号：RS-LiDAR-16，ZED2

## 依赖
### 主依赖
opencv 3.4.11(主要是为了和CUDA兼容，实际上只要是opencv3,应该都可以)
opencv contrib 3.4.11（版本必须与opencv保持一致）
ROS Melodic
zed sdk
Ceres
GLOG
### 副依赖
CUDA 10.2（由显卡以及ZED sdk决定）

# 使用说明
## 运行
```
roslaunch lv_align_verify lv_align_verify.launch
```
## 动态调参
```
rosrun rqt_reconfiguration rqt_reconfiguration
```

# 补充说明
1. 整个程序pose的定义规则：transform_c_l表示lidar在camera中的pose
2. 动态调参中的r q y为常见的欧拉角定义，即代表以ZYX顺序进行内旋。
3. 外参输入说明：从其他标定程序得到齐次外参矩阵，将其填入yaml的`transform_c_l`即可，程序启动后会提示对应的`x,y,z,roll,pitch,yaw`，并且自动修改dynamic reconfigure里的参数。
4. 在画面中，按下方按键会执行相应指令
* q : 关闭程序
* s : 保存当前帧
5. 画面中的点云颜色与反射强度直接相关，随着反射强度的增加，颜色经历以下变换:红色->绿色->蓝色。
6. 程序退出时，会打印最终的变换矩阵。

# 修改日志
1.2021-09-11 实现外参的可视化验证功能

# TODO
[]激光点的投影和图像存在明显的延迟
* 原因应该和grab retrieve的机制有关，比如图像可能比激光晚了很久，才到的。

[]代码清理

[]IMU运动畸变校正
