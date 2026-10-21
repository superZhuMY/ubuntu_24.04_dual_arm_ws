# 标定与坐标系

## 坐标链路

视觉点位使用以下链路进入双臂控制：

```text
像素坐标 (u, v) + depth
  -> 相机内参
  -> camera_point = [x, y, z]
  -> T_camera_to_base
  -> base_link 下的 tomato_base_point
  -> offset 生成左右臂目标点
```

`dual_arm_task_bridge_node.py` 使用配置文件中的 `T_camera_to_base` 做矩阵乘法，不依赖 TF。

## 相机内参

内参位于 `src/vision_double_arm_ros1/config/vision_task.yaml`：

```yaml
cameras:
  left:
    intrinsics:
      fx: 913.7384643554688
      fy: 913.9617919921875
      cx: 641.9942626953125
      cy: 373.4631042480469
  right:
    intrinsics:
      fx: 900.6747436523438
      fy: 901.5196533203125
      cx: 657.5897827148438
      cy: 373.44219970703125
```

如果相机分辨率或 RealSense 标定发生变化，需要重新从 `/left_camera/color/camera_info` 和 `/right_camera/color/camera_info` 读取。

## 手眼外参

配置字段：

```yaml
T_camera_to_base:
  - [...]
  - [...]
  - [...]
  - [0.0, 0.0, 0.0, 1.0]
```

方向必须是：

```text
base_point = T_camera_to_base * camera_point
```

如果手眼标定工具输出的是 `base_link -> camera`，需要先求逆再填入。

## 左右相机当前配置

- 左相机 frame：`left_camera_color_optical_frame`
- 右相机 frame：`right_camera_color_optical_frame`
- 左相机 topic：`/left_camera/color/image_raw`、`/left_camera/aligned_depth_to_color/image_raw`
- 右相机 topic：`/right_camera/color/image_raw`、`/right_camera/aligned_depth_to_color/image_raw`

`calibrated: true` 表示桥接节点允许使用该相机外参发布任务。若外参未确认，应先改为 `false`。

## 工具补偿和目标偏移

视觉桥接偏移在 `vision_task.yaml` 中：

- `bridge.leaf.left_support_offset_base`：打叶模式左臂目标相对番茄点的 base 坐标系偏移。
- `bridge.leaf.right_leaf_offset_base`：打叶模式右臂目标相对番茄点的 base 坐标系偏移。
- `bridge.tomato_pick.left_pick_offset_base`：采摘模式左臂目标相对番茄点的 base 坐标系偏移。
- `bridge.tomato_pick.right_pick_offset_base`：采摘模式右臂目标相对番茄点的 base 坐标系偏移。

执行器工具补偿在 `src/moveit_exe/config/manual_dual_arm_task.yaml`：

- `left_pre_offset` / `right_pre_offset`：预接近点偏移。
- `left_target_offset` / `right_target_offset`：目标点到法兰控制点的补偿，按末端姿态旋转后再加到目标点。

## TF 与 URDF 相机建模

当前控制闭环不强制要求相机加入 URDF。只要 `T_camera_to_base` 准确，视觉点即可转换到 `base_link`。

需要 TF/URDF 建模的场景：

- RViz 中显示真实相机位置。
- MoveIt planning scene 使用相机点云或障碍物。
- 需要 `world/base_link` 和 `left_camera_*` frame 在同一 TF 树内。

推荐后续做法：

- 短期：发布 `base_link -> left_camera_link`、`base_link -> right_camera_link` 的静态 TF，避免直接抢占 RealSense 内部 optical frame。
- 长期：把相机 link 和 fixed joint 纳入 `double_arm_robot.urdf`，并由手眼标定结果更新固定关节。
