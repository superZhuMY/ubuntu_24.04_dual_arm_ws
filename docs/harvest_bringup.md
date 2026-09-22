# 双臂采摘执行层 — 硬件启动与验收程序（H1–H5）

对应分支：`feature/dual-arm-harvest-execution`（基于 main `f1b94fb`）。

## 1. 交付内容总览

新增两个包，未修改任何底层执行行为：

| 包 | 内容 |
|---|---|
| `double_arm_harvest_interfaces` | `HarvestTarget.msg`、`ExecuteDualHarvest.action` |
| `double_arm_harvest_execution` | `dual_arm_harvest_executor`（C++，MGI 分臂规划 + 双臂同步下发 + 阶段屏障）、`send_manual_harvest_goal.py`、`harvest_fake.launch.py`、`harvest_stm32.launch.py`、gtest + launch_testing 测试 |

执行链路（与文档第 2 节一致）：

```text
ExecuteDualHarvest Action (/execute_dual_harvest)
  -> MoveIt 分臂规划 (L_arm / R_arm)
  -> /double_arm_robot/{l,r}_arm/follow_joint_trajectory
  -> streaming_trajectory_executor（header.stamp 同步启动）
  -> ForwardCommandController -> Stm32SystemHardware -> F407 + 电机
```

对现有文件的唯一改动：

- `stm32_moveit.launch.py`：新增 `allow_simultaneous_arms` 参数（默认 `false`，行为不变）；`hardware_mode` choices 增加 `fake`（供 fake 集成测试复用完整链路）。
- `kinematics.yaml`：IK 超时 5ms → 50ms（见第 4 节问题 1）。

## 2. fake 模式验证（无硬件）

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch double_arm_harvest_execution harvest_fake.launch.py
# 另一终端（等 "move groups ready" 日志后）：
ros2 run double_arm_harvest_execution send_manual_harvest_goal.py --timeout 120
```

已验证通过的场景：

1. 双臂完整任务 APPROACH→PICK→RETREAT→HOME 全部 SUCCEEDED，每阶段双臂端点稳定确认；
2. 单臂（`--no-left` / `--no-right`）；
3. `plan_only:=true`（只规划不下发）、`dry_run`；
4. 一侧规划失败 → 任务 ABORTED，无任何下发；
5. 中途取消 → 两条 FollowJointTrajectory Goal 均被取消并保持位置，下一个任务不受残留取消影响；
6. 重复 `target_id` / 非法目标（错误 frame、NaN、零四元数、低置信度）被拒绝。

## 3. H5 真机分级验收（需操作员在场）

配置确认（上电前）：

- [ ] `harvest_motion.yaml`：`sync_start_delay_sec: 0.5`、`velocity_scale/acceleration_scale: 0.10`、阶段偏移（当前保守值 approach −0.03 / retreat −0.04，工具系）
- [ ] `manual_targets.yaml` 换成**实测安全坐标**（fake 模式的 FK 示例坐标严禁上真机）
- [ ] `allow_simultaneous_arms` 公共默认仍为 `false`（`sparse_execution.yaml`），只有 `harvest_stm32.launch.py` 覆盖为 `true`

启动：

```bash
ros2 launch double_arm_harvest_execution harvest_stm32.launch.py
# 等 "move groups ready" 日志；RViz 默认开启
ros2 run double_arm_harvest_execution send_manual_harvest_goal.py --no-right   # 步骤 1
```

分级顺序（每步通过后才进行下一步，异常立即 Ctrl+C 并记录）：

| 步骤 | 命令 | 通过标准 |
|---|---|---|
| 1. 单左臂 | `send_manual_harvest_goal.py --no-right --timeout 180` | 四阶段成功，姿态平顺，无超调报警 |
| 2. 单右臂 | `send_manual_harvest_goal.py --no-left --timeout 180` | 同上 |
| 3. 双臂小范围 | `send_manual_harvest_goal.py --timeout 240`（目标改小位移） | 两臂在同一 `dispatch start` 时刻近似同时起动 |
| 4. 双臂 + RETREAT | 同上（标准偏移） | APPROACH→RETREAT 成功，HOME 阶段暂跳过可用 `dry_run` 校验 |
| 5. 增加 PICK | 标准四阶段 | PICK 段到位稳定 |
| 6. 完整含 HOME | 标准任务 | 最终 SUCCEEDED，两臂回 `L_home`/`R_home` |

中途取消验证（真机，低速下做一次即可）：任务进行中在客户端 Ctrl+C（或 `ros2 action send_goal` 后另终端 cancel）→ 两条控制 Goal 均取消、臂保持当前位置、结果 CANCELED。

记录模板：把每步的 `dispatch start=…`、各阶段 `duration=…`、最终 status 与异常摘录补进本文档第 6 节（或单独的 phase 报告）。

## 4. fake 联调期间发现的问题与决定

1. **IK 超时过紧（已修复）**：`kinematics.yaml` 原为 Setup Assistant 默认 `kinematics_solver_timeout: 0.005`，该 3P+3R 链（棱柱 XYZ + Y-Z-Y 腕）的姿态目标采样反复 `NO_IK_SOLUTION`。放宽到 `0.05` 后随机种子 IK 30/30 成功。
2. **Home 位姿不宜作姿态目标**：home 处腕部 θ5=0 奇异且 `L/R_Joint_3` 距下限仅 0.01，姿态目标采样几乎必然失败。真机目标应由实测挑选**非奇异**位姿（可用 `/compute_ik` + 随机种子自检，参照 `manual_targets.yaml` 注释）。
3. **接近/撤离偏移受棱柱 Z 预算限制**：`L/R_Joint_2` 上限 +0.05，名义目标下方 −0.08m 的偏移不可达。第一轮偏移已收紧为 −0.03/−0.04，真机标定后再调整。
4. **取消状态码**：取消与 rclcpp_action 服务端自身的 CANCELING 转换存在竞态，已用短重试 + `abort()` 兜底；若个别情况结果状态显示 ABORTED 但消息为 "canceled by client"，属预期兜底行为，臂已安全保持。
5. **环境备忘**：conda 会污染链接路径与 Python（构建/测试前把 PATH 中的 miniconda 去掉并用 `/usr/bin/python3`）；SSH 22 端口被本机代理拦截，git 远程操作走 `ssh.github.com:443`。

## 5. 已知边界（第一版设计边界，非缺陷）

- 不做左右臂互碰检测与 12 轴联合规划（两臂工作空间分离前提下使用）；
- `PREPARE_GRIPPER/END_EFFECTOR` 已通过 USB/TTL 双舵机 Action 实现；默认关闭，接线、
  标定与验收见 `docs/dual_gripper_integration.md`；`PLACE/RELEASE` 仍为预留阶段；
- 失败后保持当前位置等待人工处理，不自动回 Home；
- MoveIt 规划期间到达的取消在规划完成后生效（单次规划 ≤ planning_time_sec）。

## 6. 真机验证记录（待 H5 填写）

| 步骤 | 日期 | 结果 | duration(s)/阶段 | 备注 |
|---|---|---|---|---|
| 1 单左臂 | | | | |
| 2 单右臂 | | | | |
| 3 双臂小范围 | | | | |
| 4 双臂+RETREAT | | | | |
| 5 +PICK | | | | |
| 6 完整+HOME | | | | |
| 中途取消 | | | | |
