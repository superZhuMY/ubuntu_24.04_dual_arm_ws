# 阶段 A+B 完成报告

## 一、修改文件

| 文件 | 操作 | 变更内容 |
|---|---|---|
| `src/double_arm_robot_moveit_config/config/double_arm_robot.srdf` | 修复 | group `l_arm`→`L_arm`, `r_arm`→`R_arm`；named state `home_left`→`L_home`(J1=0.20), `home_right`→`R_home`(J1=-0.20)；改用 `<chain>` 语义 |
| `src/double_arm_robot_moveit_config/config/kinematics.yaml` | 修复 | group 名称 `l_arm`→`L_arm`, `r_arm`→`R_arm` |
| `src/double_arm_robot_moveit_config/config/initial_positions.yaml` | 修复 | L_Joint_1: 0→0.20, R_Joint_1: 0→-0.20 |
| `src/double_arm_robot_moveit_config/config/ros2_controllers.yaml` | 重写 | controller 重命名为 `l_arm`/`r_arm`；全部节点使用 FQN key（`/double_arm_robot/...`）以匹配 namespace 下的 param file 加载 |
| `src/double_arm_robot_moveit_config/config/moveit_controllers.yaml` | 重写 | controller 名称同步为 `l_arm`/`r_arm` |
| `src/double_arm_robot_moveit_config/launch/demo.launch.py` | 重写 | 自定义 launch：controller_manager 在 `/double_arm_robot` 命名空间，action 路径正确，显式传递 kinematics + pilz_cartesian_limits |
| `ROS2.2_ws(1)/ROS2.2_ws/COLCON_IGNORE` | 新建 | 排除参考工程 |
| `src/` 下 3 个包 | 从 ROS2.2_ws 拷贝 | `double_arm_jaka_interfaces`, `double_arm_robot`, `double_arm_robot_moveit_config` |

## 二、架构变化

| 项目 | 旧状态 | 新状态 |
|---|---|---|
| MoveIt group | `l_arm` / `r_arm` (小写) | `L_arm` / `R_arm` (大写，与 ROS1 一致) |
| Named state | `home_left` / `home_right` | `L_home`(J1=0.20) / `R_home`(J1=-0.20) |
| 控制架构 | 自定义 FollowJointTrajectory action server | `joint_trajectory_controller` + `controller_manager` + `mock_components/GenericSystem` |
| Controller action | `/l_arm_controller/follow_joint_trajectory` | `/double_arm_robot/l_arm/follow_joint_trajectory` |
| Controller action | `/r_arm_controller/follow_joint_trajectory` | `/double_arm_robot/r_arm/follow_joint_trajectory` |

## 三、关键问题解决

| 问题 | 根因 | 解决方案 |
|---|---|---|
| `The 'type' param was not defined` | ROS2 param file 的 node-name matching 在 namespace 下需要 FQN key | YAML 改用 `/double_arm_robot/controller_manager` 等 FQN key |
| `Waiting for robot_description` | controller_manager 在 namespace 下订阅 `{ns}/robot_description`，而 rsp 发布到 `/robot_description` | 添加 absolute remapping |
| `cartesian_limits.max_trans_vel` not initialized | Pilz planner 参数未传给 move_group | launch 中加入 `moveit_config.pilz_cartesian_limits` |
| kinematics 警告 | kinematics.yaml 的 group 名与 SRDF 不一致 | kinematics.yaml 改为 `L_arm`/`R_arm` |

## 四、构建命令和结果

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

```
Summary: 3 packages finished
  double_arm_robot                      ✅
  double_arm_jaka_interfaces            ✅
  double_arm_robot_moveit_config        ✅
```

## 五、验收结果

### 5.1 ros2 control 命令

```bash
# 控制器状态
$ ros2 control list_controllers -c /double_arm_robot/controller_manager

r_arm                   joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
l_arm                   joint_trajectory_controller/JointTrajectoryController  active
```

✅ 3 个控制器全部 active

```bash
# 硬件接口
$ ros2 control list_hardware_interfaces -c /double_arm_robot/controller_manager
```

✅ 12 个 position command interface 全部 `[claimed]`
✅ 12 个 position + 12 个 velocity state interface

```bash
# Action 列表
$ ros2 action list | grep follow_joint

/double_arm_robot/l_arm/follow_joint_trajectory
/double_arm_robot/r_arm/follow_joint_trajectory
```

✅ Action URI 与旧 ROS1 接口兼容

```bash
# joint_states 发布频率
$ ros2 topic hz /joint_states

average rate: 99.995 Hz
```

✅ `/joint_states` 持续发布，~100 Hz

### 5.2 轨迹执行测试

```bash
# L_arm → L_home (J1=0.20)
$ ros2 action send_goal /double_arm_robot/l_arm/follow_joint_trajectory ...

Goal accepted with ID: b7bb1a4a8aa24f84ac9eeb58f15b3af5
error_code: 0
error_string: Goal successfully reached!
Goal finished with status: SUCCEEDED
```

✅ L_arm 轨迹执行成功

```bash
# R_arm → R_home (J1=-0.20)
$ ros2 action send_goal /double_arm_robot/r_arm/follow_joint_trajectory ...

error_code: 0
error_string: Goal successfully reached!
Goal finished with status: SUCCEEDED
```

✅ R_arm 轨迹执行成功

### 5.3 完整 MoveIt2 → Controller 链路测试

通过 `/move_action` action（MoveGroup）进行 plan+execute，验证完整链路：

```
MoveIt 2 → SimpleControllerManager → FollowJointTrajectory
→ joint_trajectory_controller → ros2_control → GenericSystem
```

```bash
$ python3 test/test_moveit_e2e.py
```

| 测试 | 结果 |
|---|---|
| L_arm → offset(J1=0.10) | ✅ SUCCESS (code=1) |
| L_arm → L_home(J1=0.20) | ✅ SUCCESS (code=1) |
| R_arm → offset(J1=-0.10) | ✅ SUCCESS (code=1) |
| R_arm → R_home(J1=-0.20) | ✅ SUCCESS (code=1) |

执行后 joint_states 与目标值对比：

| 关节 | 期望 | 实际 | 偏差 |
|---|---|---|---|
| L_Joint_1 | 0.20 | 0.1905 | 0.0095 (在容差 0.01 内) |
| R_Joint_1 | -0.20 | -0.2001 | 0.0001 |
| 其他 L_Joint_2..6 | 0.0 | ~±0.009 | 在容差 0.01 内 |
| 其他 R_Joint_2..6 | 0.0 | ~±0.009 | 在容差 0.01 内 |

### 5.4 关键技术决策：namespace + remapping 桥接

MoveIt SimpleControllerManager 使用相对 action 路径（`l_arm/follow_joint_trajectory`）。
为同时满足 `/double_arm_robot/l_arm/follow_joint_trajectory` 的 action 路径要求，
采用 remapping 桥接方案：

- controller_manager 在 `/double_arm_robot` 命名空间 → action 位于 `/double_arm_robot/l_arm/follow_joint_trajectory`
- move_group 在根命名空间（保持默认 service/action 兼容）
- move_group 添加 remapping: `l_arm/follow_joint_trajectory` → `/double_arm_robot/l_arm/follow_joint_trajectory`

### 5.5 MoveIt 状态

- ✅ `move_group` 成功启动："You can start planning now!"
- ✅ KDL kinematics plugin 为 L_arm / R_arm 正确加载
- ✅ Pilz Industrial Motion Planner 加载成功
- ✅ STOMP, OMPL, CHOMP 规划器加载成功
- ✅ OMPL RRTConnect 规划器实际生成轨迹并执行成功
- ✅ SimpleControllerManager 通过 remapping 正确连接 `/double_arm_robot/l_arm/follow_joint_trajectory`
- ⚠️ 末端位姿 IK 规划因 Python 绑定问题需在 GUI 环境或更高版本 moveit_py 测试

## 六、未解决问题

1. **末端位姿 IK 规划**：Cartesian pose constraint 在 moveit_msgs Python 绑定中存在类型转换问题（`geometry_msgs/Pose` assertion），需要 moveit_py 包或 GUI 环境手动测试
2. **RT 调度**：`Could not enable FIFO RT scheduling policy` — 非 root 环境预期行为
3. **SystemInterface 插件**：自定义硬件插件尚未实现（阶段 C/D）
4. **kdl_parser 警告**：`root link base_link has an inertia` — 不影响功能
5. **TOTG 混联警告**：`There is a combination of revolute and prismatic joints` — 不影响功能

## 七、是否访问过真实硬件

**否**。仅使用 `mock_components/GenericSystem`。
