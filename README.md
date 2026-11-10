# hero_chassis_controller

RoboMaster 英雄机器人底盘控制器作业项目，面向 ROS Noetic / catkin。

## 当前进度

目前只完成建包：统一包名与 CMake project 名称，清理团队模板温度测量示例的构建配置和专用依赖。
尚未实现底盘控制器、运动学解算、速度指令订阅、轮速控制或仿真验证。
当前没有可运行的底盘节点或控制器插件，构建成功仅表示包配置有效。

模板的 src/、include/、test/、config/、launch/ 和 doc/ 文件暂保留供参考，
其中温度订阅、求平均值算法及其测试均不参与构建、导出或安装；模板启动文件不能作为底盘启动入口。

## 构建

本包使用 catkin 构建，ROS 依赖见 package.xml

CLion 复用此前 simple_chassis_controller 的 Docker 工具链，配置参数为：

```text
-DCMAKE_PREFIX_PATH=/opt/ros/noetic -DPYTHON_EXECUTABLE=/usr/bin/python3
```

已在 ROS Noetic Docker 环境中完成构建，并在 Gazebo 中验证控制功能。

## 维护者

- 维护者姓名 Sikai Wang
- 维护者邮箱 wangsikai2008@qq.com

原模板作者署名保留，不代表当前项目维护者。

## 许可证

沿用原模板的 [BSD 3-Clause 许可证](LICENSE)，保留原许可证及版权声明。

## /cmd_vel 与麦克纳姆逆运动学

控制器订阅 `/cmd_vel`（`geometry_msgs/Twist`），将其解释为 `base_link` 底盘坐标系速度：

- `linear.x`：向前为正
- `linear.y`：向左为正
- `angular.z`：逆时针为正

轮子顺序为 `front_left`、`front_right`、`back_left`、`back_right`。设轮半径为 `r`，底盘长度为 `L`，宽度为 `W`，则目标角速度为：

```text
front_left  = (vx - vy - (L + W) * wz / 2) / r
front_right = (vx + vy + (L + W) * wz / 2) / r
back_left   = (vx + vy - (L + W) * wz / 2) / r
back_right  = (vx - vy + (L + W) * wz / 2) / r
```

当前默认参数来自 `config/controllers.yaml`：`r=0.07625 m`、`L=0.4 m`、`W=0.4 m`。
没有收到 `/cmd_vel` 时，目标轮速保持为零。

## 速度坐标模式

`config/controllers.yaml` 中的 `velocity_mode` 支持 `body` 和 `world`：

- `body`：默认模式。将 `/cmd_vel` 解释为 `base_link` 坐标系速度，直接进行麦克纳姆逆运动学。
- `world`：将 `/cmd_vel` 的线速度解释为 `odom` 坐标系速度，使用当前 `odom -> base_link` TF 转换到车体坐标系后再进行逆运动学。`angular.z` 始终保留 ROS z 轴角速度含义，不做坐标旋转。

TF 转换失败时，控制器会输出 `ROS_WARN` 并将四个目标轮速清零。

## 特色功能：可配置的 `/cmd_vel` 指令加速度限制

当前 `rm-noetic-dev:sim` 镜像未安装 `teleop_twist_keyboard`，因此本项目提供可配置的 `/cmd_vel` 指令速度斜坡限制：

```yaml
max_linear_acceleration: 1.0  # m/s^2，限制 linear.x 和 linear.y
max_angular_acceleration: 2.0 # rad/s^2，限制 angular.z
```

控制器根据相邻 `/cmd_vel` 回调的时间间隔限制指令变化率，再将限制后的指令交给坐标模式、麦克纳姆逆运动学和 PID 控制链路。该功能验收的是指令或目标速度变化率：

```text
|Δlinear| / Δt <= max_linear_acceleration
|Δangular.z| / Δt <= max_angular_acceleration
```

它不保证 `/odom` 经过 PID、Gazebo 物理引擎和机械动力学后的实际车辆加速度低于相同上限。发送零速度指令后，限制器会让目标速度逐步回到零，车辆随后减速停止。
