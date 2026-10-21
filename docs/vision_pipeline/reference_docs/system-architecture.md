# 系统架构

## 工作区结构

```text
vision_double_arm_ros1/
  src/double_arm_robot            # URDF、mesh、基础机器人模型
  src/double_arm_robot_moveit     # MoveIt 配置和规划入口
  src/jaka_controller_tcp         # 硬件 action server 与电机通信
  src/moveit_exe                  # 双臂任务消息和执行节点
  src/vision_double_arm_ros1      # 视觉检测和任务桥接
```

`build/`、`devel/` 是 catkin 生成目录，不作为源码设计的一部分。

## 节点关系

```text
camera_left.launch
  -> realsense2_camera/rs_camera.launch
  -> /left_camera/color/image_raw
  -> /left_camera/aligned_depth_to_color/image_raw

detector_left.launch
  -> tomato_detector_node.py
  -> /left_tomato_detections

camera_right.launch
  -> realsense2_camera/rs_camera.launch
  -> /right_camera/color/image_raw
  -> /right_camera/aligned_depth_to_color/image_raw

detector_right.launch
  -> tomato_detector_node.py
  -> /right_tomato_detections

core.launch
  -> double_arm_core.launch

pick_left.launch / pick_right.launch / pick_both.launch
  -> dual_arm_task_bridge_node.py
  -> dual_arm_task_executor.py

leaf_left.launch / leaf_right.launch / leaf_both.launch
  -> dual_arm_task_bridge_node.py
  -> dual_arm_task_executor.py
```

## 消息接口

`vision_double_arm_ros1/TomatoDetection` 描述单个视觉目标，包括相机名、类别、置信度、像素坐标、深度、相机坐标和 base 坐标。

`vision_double_arm_ros1/TomatoDetections` 是检测数组，左右检测节点分别发布到不同 topic。

`moveit_exe/DualArmTask` 是视觉和执行节点之间的任务协议：

```text
Header header
Point left_support_point
int32 left_direction
bool execute_left
Point right_leaf_point
int32 right_direction
bool execute_right
float32 confidence
string source
```

`execute_left/execute_right` 用于区分单臂调试、双臂采摘和打叶模式中的启用臂。

## 模式差异

### `leaf` 打叶模式

- 桥接节点按 `active_arm` 读取 `bridge.topics.leaf_left`、`bridge.topics.leaf_right` 或 `bridge.leaf.detections_topic`。
- 选取最近有效番茄点。
- 生成 `left_support_point = tomato_base + left_support_offset_base`。
- 生成 `right_leaf_point = tomato_base + right_leaf_offset_base`。
- `leaf_left.launch` 只启用左臂，`leaf_right.launch` 只启用右臂，`leaf_both.launch` 同时启用左右臂。

### `tomato_pick` 采摘模式

- 桥接节点按 `active_arm` 订阅 `bridge.topics.left`、`bridge.topics.right` 或两者同时订阅。
- 左相机目标生成左臂采摘点，并设置 `execute_left=true`。
- 右相机目标生成右臂采摘点，并设置 `execute_right=true`。
- `pick_pair_timeout_sec` 控制左右检测结果是否可合并进同一条任务。
- `pick_left.launch` 只启用左臂，`pick_right.launch` 只启用右臂，`pick_both.launch` 同时启用左右臂。
- `pick_both.launch` 默认 `parallel_pick=true`，左右臂并行采摘；单侧失败不会打断另一侧，最终分别记录 `left_ok/right_ok`。
- 实机双臂调试推荐同时启动 `pick_left.launch` 和 `pick_right.launch` 两个终端，而不是 `pick_both.launch`；这样左右臂日志分离，一侧异常不会让排查信息混在同一个窗口。
- 采摘动作到预采摘点后会记录关节值；最终点闭合夹爪后优先按记录关节值退回预采摘点，再回 `L_home/R_home`，回 home 成功后松开对应夹爪。关节退回失败时再回退笛卡尔撤离和普通 Pose 规划。
- 每次 Pose、Joint、Cartesian 和 home 规划前都会显式刷新 MoveIt 当前起始状态，降低左右臂同时运动时使用过期状态规划的概率。
- 当前并行采摘不是双臂整体规划，MoveIt 不会自动按 combined group 做同步避碰。

## 技术栈

- ROS Noetic / catkin
- MoveIt / OMPL / TRAC-IK
- RealSense ROS wrapper
- Python `rospy`、OpenCV、NumPy、PyTorch/Ultralytics
- C++ `roscpp`、`actionlib`、`control_msgs/FollowJointTrajectory`
- `serial`、`libmodbus`、`pymodbus`
