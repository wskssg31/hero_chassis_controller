# hero_chassis_controller

RoboMaster 英雄机器人底盘控制器作业项目，面向 ROS Noetic / catkin。

## 当前进度

目前只完成建包：统一包名与 CMake project 名称，清理团队模板温度测量示例的构建配置和专用依赖。
尚未实现底盘控制器、运动学解算、速度指令订阅、轮速控制或仿真验证。
当前没有可运行的底盘节点或控制器插件，构建成功仅表示包配置有效。

模板的 src/、include/、test/、config/、launch/ 和 doc/ 文件暂保留供参考，
其中温度订阅、求平均值算法及其测试均不参与构建、导出或安装；模板启动文件不能作为底盘启动入口。

## 构建

当前仅依赖 catkin，不需要模板示例使用的 sensor_msgs、Eigen、Boost 或 roslint。

在已安装 ROS Noetic 的 catkin 工作空间中：

```bash
source /opt/ros/noetic/setup.bash
cd <catkin_workspace>
catkin_make --only-pkg-with-deps hero_chassis_controller
```

CLion 复用此前 simple_chassis_controller 的 Docker 工具链，配置参数为：

```text
-DCMAKE_PREFIX_PATH=/opt/ros/noetic -DPYTHON_EXECUTABLE=/usr/bin/python3
```

CMake 配置和构建验证结果由本次操作报告提供；尚未进行控制功能或硬件测试。

## 维护者信息待填写

请提供以下 package.xml 字段：

- 维护者姓名（maintainer 文本，当前为 TODO）
- 维护者邮箱（maintainer email，当前为 todo@example.com）

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
