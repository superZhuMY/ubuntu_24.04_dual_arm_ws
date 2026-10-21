# 视觉链路提取清单(MANIFEST)

- **来源仓库**:`superZhuMY/vision_double_arm_ros1`(ROS1 Noetic,完整 catkin 工作区)
- **本地全量克隆**:`/home/zmy/tomato/vision_double_arm_ros1`(克隆自 GitHub,HEAD `60b3b3a`,2026-06-10;原样未动)
- **提取时间**:2026-09-22
- **提取方式**:逐文件原样拷贝,已用 `cmp` 与克隆仓逐字节校验一致,**未做任何改写**。
  本目录代码均为 ROS1 实现,仅作链路参考与后续集成底稿,不能直接放进本 ROS2 工作区编译。

## 已拷贝文件

| 本目录路径 | 克隆仓源路径 | 作用 |
|---|---|---|
| `scripts/tomato_detector_core.py` | `src/vision_double_arm_ros1/scripts/` | 检测核心:YOLO 推理 + 深度反投影 + 过滤,发布 `TomatoDetections` |
| `scripts/tomato_detector_node.py` | 同上 | conda 启动包装器(source conda → activate yolov9 → exec 核心脚本) |
| `scripts/vision_task_lib.py` | 同上 | 像素→相机系反投影、4x4 变换、工作空间判定等纯函数 |
| `scripts/dual_arm_task_bridge_node.py` | 同上 | 任务桥:检测→base_link 变换→门控/去重→发布 `/dual_arm_task` |
| `msg/TomatoDetection.msg` | `src/vision_double_arm_ros1/msg/` | 单个番茄检测(像素/深度/相机系点/base 系点/距离) |
| `msg/TomatoDetections.msg` | 同上 | 检测数组 + `selected_index` |
| `config/vision_task.yaml` | `src/vision_double_arm_ros1/config/` | 全链路参数:检测阈值、相机内参、手眼外参、工作空间、偏移 |
| `launch/camera_left.launch` / `camera_right.launch` | `src/vision_double_arm_ros1/launch/` | 双 RealSense 启动(左 `f0211703` / 右 `f1421997`,15fps,align_depth) |
| `launch/detector_left.launch` / `detector_right.launch` | 同上 | 左右检测节点启动(加载 vision_task.yaml,指定输出 topic) |
| `launch/pick_left.launch` / `pick_right.launch` / `pick_both.launch` | 同上 | 采摘模式入口(bridge + executor,task_mode=tomato_pick) |
| `launch/leaf_left.launch` / `leaf_right.launch` / `leaf_both.launch` | 同上 | 打叶模式入口(task_mode=leaf) |
| `launch/core.launch` | 同上 | 老仓库核心链路入口(MoveIt/硬件,一套只开一次) |
| `test/test_vision_task_lib.py` | `src/vision_double_arm_ros1/test/` | 变换库单元测试 |
| `test/test_dual_arm_task_bridge_node.py` | 同上 | 任务桥单元测试 |
| `reference_docs/calibration-and-frames.md` | `docs/`(仓库根) | 标定与坐标系约定(T_camera_to_base 方向、URDF 相机建模建议) |
| `reference_docs/system-architecture.md` | 同上 | 老仓库自己的系统架构文档 |

## 超出原计划的补充(均为后续集成必需)

- `msg/DualArmTask.msg` —— 取自克隆仓 `src/moveit_exe/msg/`,是视觉链路的**输出接口契约**,对接本工作区 `HarvestTarget`/`ExecuteDualHarvest` 时必须逐字段对照(见 README 第 5 节)。
- `test/test_dual_arm_task_bridge_node.py` —— 任务桥是后续集成的主要接缝,其测试锁定了桥接行为。
- `launch/pick_*.launch`、`leaf_*.launch`、`core.launch` —— 链路的可运行入口,展示了 `task_mode`/`active_arm`/`plan_only` 等覆盖方式。
- `reference_docs/system-architecture.md` —— 老仓库对链路的第一手描述。

## 刻意未拷贝(留在克隆仓,集成阶段再处理)

| 内容 | 克隆仓位置 | 原因 |
|---|---|---|
| YOLO 权重 `putong.pt` | `src/vision_double_arm_ros1/weights/putong.pt` | 二进制大文件;集成时直接从克隆仓移动/引用,勿散拷进文档目录 |
| vendored ultralytics | `src/vision_double_arm_ros1/third_party/ultralytics/` | 定制版推理库,导入路径非标准(`ultralytics.cfg.models.utils.*`),必须整体配套使用,散拷无意义;README 明确警告不可用 pip 版替换 |
| `third_party/segval.py` | 同目录 | 独立脚本,与主链路无耦合 |
| `moveit_exe` 执行端 | `src/moveit_exe/`(`dual_arm_task_executor.py` 等) | 本工作区已有新执行层 `double_arm_harvest_execution`,老执行端仅作对照参考 |
| `jaka_controller_tcp` | `src/jaka_controller_tcp/` | 老 JAKA 机械臂硬件链路,与本工作区 STM32 链路无关 |
| `double_arm_robot(_moveit)` | `src/double_arm_robot*/` | 与本工作区同源的旧版机器人描述/MoveIt 配置,以本工作区为准 |
| 其余根目录文档 | `docs/project-overview.md`、`runbook.md`、`extension-roadmap.md` | ROS1 语境的运行/路线文档,需要时直接查阅克隆仓 |
