# 番茄视觉识别链路参考(提取自 vision_double_arm_ros1)

> **来源**:`superZhuMY/vision_double_arm_ros1`(ROS1 Noetic,克隆于 `/home/zmy/tomato/vision_double_arm_ros1`,HEAD `60b3b3a`)。
> 本目录 = 链路文档 + **原样代码**(逐字节未改写,提取清单见 [MANIFEST.md](MANIFEST.md))。
> 所有代码是 ROS1 实现,仅供链路参考与后续集成底稿,**不能直接放入本 ROS2 工作区编译**。
> 本工作区执行层(`double_arm_harvest_execution`)已就绪,视觉链路是缺失的上游 —— `HarvestTarget.msg` 注释明确:"未来的视觉节点负责把检测结果转成 base 坐标系下的 PoseStamped"。

---

## 1. 链路总览

```text
                        ┌─────────────────── 每臂一路,左右完全独立 ───────────────────┐
  RealSense(左 f0211703)                     RealSense(右 f1421997)
  /left_camera/color/image_raw (bgr8)         /right_camera/color/image_raw
  /left_camera/aligned_depth_to_color/...     /right_camera/aligned_depth_to_color/...
        │                                           │
        ▼                                           ▼
  tomato_detector_node(左)                   tomato_detector_node(右)
  YOLO(putong.pt)→ 框中心像素 + 深度反投影 3D   同一套代码,camera_name=right
        │  /left_tomato_detections                  │  /right_tomato_detections
        ▼                                           ▼
  ┌──────────────────── dual_arm_task_bridge_node ────────────────────┐
  │  T_camera_to_base 静态手眼 → base_link                             │
  │  工作空间门控 → 重复目标抑制 → 5s 最小发布间隔                      │
  │  leaf 模式:单流选最近   tomato_pick 模式:左右双流按 1s 时间窗融合  │
  └──────────────────────────────┬────────────────────────────────────┘
                                 ▼
                    /dual_arm_task (moveit_exe/DualArmTask)
                                 ▼
                    dual_arm_task_executor(老执行端,仅对照)
                    MoveIt L_arm/R_arm 规划 → 夹爪 → 采摘序列
```

核心思想:**整条链路不用 TF**。像素 → 相机系用静态内参反投影;相机系 → base_link 用 YAML 里的静态 4x4 手眼矩阵直接乘;鲁棒性靠"深度帧驱动推理 + 多级过滤 + 时间/空间去重",而不是消息频率。

## 2. 分层详解

### 2.1 相机层(`launch/camera_left.launch` / `camera_right.launch`)

| 项 | 左 | 右 |
|---|---|---|
| RealSense 序列号 | `f0211703` | `f1421997` |
| ROS namespace | `/left_camera` | `/right_camera` |
| RGB | `/left_camera/color/image_raw`(bgr8) | 同构 |
| 对齐深度 | `/left_camera/aligned_depth_to_color/image_raw`(16UC1,mm) | 同构 |
| 参数 | 15fps,`align_depth:=true`,`enable_sync:=true`,`initial_reset:=true`,confidence 图关闭 | 同 |

底层是 `realsense2_camera/rs_camera.launch`(ROS1);**深度图已对齐到彩色光轴**,所以检测框中心像素可直接查深度。

### 2.2 检测层(`scripts/tomato_detector_core.py`,包装器 `tomato_detector_node.py`)

- 节点名 `tomato_detector_node`,每个相机一个实例(launch 传 `camera_name=left/right`),输出 topic 按臂分开:`/left_tomato_detections`、`/right_tomato_detections`。
- **conda 包装器**:`tomato_detector_node.py` 不做实际工作,只 `source conda.sh → activate yolov9 → exec tomato_detector_core.py`(路径可由 launch 参数 `conda_sh:=`/`conda_env:=` 覆盖),使 torch/ultralytics 与 ROS 的 Python 隔离。
- 模型:权重 `weights/putong.pt`(2 类,目标类别 0 = 番茄),经仓库内 **vendored 定制版 ultralytics**(`third_party/ultralytics`,`AutoBackend` + `letterbox` 640 + `ops.non_max_suppression`)推理;自动选 CUDA/CPU。
- **深度帧驱动**:color 回调只缓存最新帧,**每来一帧对齐深度才触发一次推理**(单飞:`processing` 标志防重入),保证 RGB 与深度时间上最接近。
- 单目标处理流程(`TomatoModel.detect`):
  1. 框中心 `(u, v)` 查深度;无效则向周围 `depth_patch_radius=2`(5×5 窗口)取有效深度**中值**;
  2. `vision_task_lib.pixel_to_camera_point` 针孔反投影 → 相机系 3D 点;
  3. 过滤:深度 Z ∈ [0.50, 1.20] m,相机系 X ∈ [−0.20, 0.20] m(挡住不属于本臂视野的目标);
  4. 按欧氏距离**由近到远排序**,`selected_index=0` 即最近目标。
- 关键参数(全部来自 `config/vision_task.yaml` 的 `detector:` 段):conf 0.80、IoU 0.10、max_det 10、`publish_empty: false`、`display: true`(OpenCV 调试窗,标注 "Best (Nearest)")。

### 2.3 消息定义(`msg/`)

```text
# TomatoDetection.msg
std_msgs/Header header          # frame_id = 相机光学 frame(bridge 输出时改写为 base_link)
string   camera_name            # left / right
int32    class_id; string class_name
float32  confidence
int32    u; int32 v             # 检测框中心像素
float32  depth_m                # 相机系 Z
geometry_msgs/Point camera_point  # 相机系 3D 点
geometry_msgs/Point base_point      # bridge 填写:手眼变换后的 base_link 点
float32  distance_m             # 相机系欧氏距离(排序用)

# TomatoDetections.msg
std_msgs/Header header
TomatoDetection[] detections
int32 selected_index            # -1 表示无检测
```

### 2.4 手眼 / 3D 变换(`config/vision_task.yaml` + `scripts/vision_task_lib.py`)

- **无 TF、无 URDF 相机建模**:每相机一组静态标定 —— `intrinsics`(fx/fy/cx/cy)+ 4×4 `T_camera_to_base`,以及 `calibrated: true/false` 开关。
- 方向约定(必须遵守):`base_point = T_camera_to_base × camera_point`。若标定工具输出的是 base→camera,**先求逆再填入**(详见 `reference_docs/calibration-and-frames.md`)。
- `bridge.common.require_calibrated_transform: true` 时,`calibrated: false` 的相机检测结果**一律不发布任务** —— 未标定硬件上线前的安全闸。
- 老相机内参(仅记录,新相机需重标):左 fx 913.74/cx 641.99,右 fx 900.67/cx 657.59。

### 2.5 任务桥(`scripts/dual_arm_task_bridge_node.py`,链路与执行层之间的接缝)

订阅检测 topic(按模式/手臂选择,见 `detection_topics_for_mode`),对每个检测:

1. `transform_detection`:手眼矩阵变换 → `base_point`,输出消息 `frame_id` 改写为 `base_link`;
2. **工作空间门控**(`within_workspace`,base_link 下的盒):x ∈ [0, 0.80],y ∈ [−0.20, 0.65],z ∈ [0.010, 0.50] m,盒外丢弃;
3. **重复目标抑制**:同臂 0.05 m 半径内且 7 s 内已发布过 → 跳过(防同一颗番茄反复触发);
4. **最小发布间隔** 5 s(`min_publish_period_sec`),防止检测帧率连续触发机械臂;
5. 组装任务发布到 `/dual_arm_task`,同时在 `/selected_tomato` 发调试用选中目标。

两种模式(参数 `task_mode`,launch 覆盖):

| | `leaf` 打叶 | `tomato_pick` 采摘 |
|---|---|---|
| 输入流 | 单流(默认左检测 topic) | 左右双流各选最近目标 |
| 双臂协同 | 同一点生成左右两个偏移目标 | 左右检测按 `pick_pair_timeout_sec: 1.0` 时间窗融合;`require_both_arms_for_pick: true` 可强制双臂齐备才发 |
| 目标偏移(base 系,相对番茄点) | 左 `[-0.02, 0, 0.045]`,右 `[0, 0, 0.20]` | 左 `[-0.02, 0, 0.045]`,右 `[0.02, 0, 0.035]` |
| 抓取方向编号(direction,查执行端姿态库) | 左右均 3 | 左右均 3 |
| `source` 字段 | `vision_tomato` | `vision_tomato_pick` |
| 单臂开关 | `active_arm`: left / right / both | 同左(未激活臂 `execute_*=false`,点位填 0) |

### 2.6 执行端(老仓库 `moveit_exe`,仅作对照,提取目录不含)

`dual_arm_task_executor.py` 订阅 `/dual_arm_task`:对 `direction` 编号查**固定抓取四元数库**(`get_quaternion_by_direction`),叠加 `manual_dual_arm_task.yaml` 的 pre/target 工具偏移,MoveIt 规划 L_arm/R_arm → 预采摘点 → 笛卡尔接近 → (Modbus 夹爪闭合) → 按记录关节值退回 → 回 `L_home/R_home`。首次调试约定 `plan_only:=true`。新工作区已有对应物(`harvest_motion.yaml` 的 approach/pick/retreat offset + `dry_run`/`plan_only`),对接时**勿重复叠加偏移**(见第 5 节)。

## 3. 运行入口(launch,老仓库)

三层结构:`core.launch`(MoveIt/硬件,一套只开一次)→ 相机/检测左右独立 → 动作链路按 打叶/采摘 × 左/右/双臂 分六个入口。典型顺序:

```bash
roslaunch vision_double_arm_ros1 core.launch moveit_controller_manager:=fake   # 先 fake 验证
roslaunch vision_double_arm_ros1 camera_left.launch        # 终端 1,左相机
roslaunch vision_double_arm_ros1 detector_left.launch      # 终端 2,左检测(自动进 conda yolov9)
roslaunch vision_double_arm_ros1 camera_right.launch       # 终端 3
roslaunch vision_double_arm_ros1 detector_right.launch     # 终端 4
roslaunch vision_double_arm_ros1 pick_both.launch plan_only:=true   # 规划验证;实机改 plan_only:=false dry_run:=false
```

## 4. 依赖(原样运行时)

- ROS1 Noetic + MoveIt;`realsense2_camera`。
- conda 环境(包装器默认 `/home/zmy/miniconda3/etc/profile.d/conda.sh`)。
- 权重:`src/vision_double_arm_ros1/weights/putong.pt`(留在克隆仓)。

### 4.1 本地环境核查结论(2026-09-22 实测)

| 项 | 状态 | 说明 |
|---|---|---|
| conda + 包装器路径 | ✅ | `/home/zmy/miniconda3/etc/profile.d/conda.sh` 存在,与老包装器默认路径一致 |
| conda 环境 `yolov9` | ❌ 不存在 | 现有 `yolo11`、`vision_env` 两个环境;`vision_env` 为空壳(py3.8 无 torch)。**用 `yolo11` 替代**,launch 时传 `conda_env:=yolo11`,或改包装器默认值 |
| `yolo11` 推理环境 | ✅(已补齐) | py3.11 + torch 2.5.1+cu121,CUDA 正常(RTX 3050 Ti Laptop 4GB,跑 2.7M 参数模型绰绰有余);opencv 5.0 / numpy 2.4 / pip ultralytics 8.4.115。核查中补装了 vendored 库缺的 `tqdm/scipy/pandas` |
| 权重 `putong.pt`(11MB) | ✅ 实测可跑 | 实为 **YOLO11-pose**(nc=2,单关键点 `kpt_shape=[1,3]`,应为采摘/茎点),ultralytics 8.3.21 训练。经**克隆仓 vendored ultralytics** 推理正常:AutoBackend 输出标准 `(1, 9, 8400)`,NMS 正常 |
| pip ultralytics 8.4.115 直接推理 | ⚠️ 不可用 | 能加载权重但 predict 会走 detect 后处理报错;老代码的 `letterbox/scale_coords` 导入路径也已变更。**推理必须配 vendored 版**(`third_party/ultralytics/`,加进 `sys.path` 即可),或重写后处理适配 pip 版 |
| ROS1 Noetic | ❌ 未装 | 本机仅 ROS2 Jazzy;老链路四层 ROS 节点需移植(检测核心的 torch 部分已验证独立可用) |
| realsense2_camera / librealsense | ❌ 未装 | ROS2 侧需 `ros-jazzy-realsense2-camera` + librealsense SDK |
| RealSense 硬件 | ❌ 未接 | 当前 USB 上无 Intel 相机(老相机序列号:左 `f0211703` / 右 `f1421997`) |

**结论**:推理环境已就绪且权重实测能出标准检测输出;缺的是相机层(驱动 + 硬件接入)和 ROS1→ROS2 节点移植。

## 5. 与本工作区执行层的对接映射(后续结合的接口说明)

老链路输出 `/dual_arm_task`(`moveit_exe/DualArmTask`,定义见 `msg/DualArmTask.msg`);本工作区执行层接收 `/execute_dual_harvest` action(`ExecuteDualHarvest`,goal 含左右 `double_arm_harvest_interfaces/HarvestTarget`)。字段对应:

| DualArmTask(老) | HarvestTarget / ExecuteDualHarvest(新) | 说明 |
|---|---|---|
| `left_support_point` / `right_leaf_point`(Point,base_link,**已含桥接偏移**) | `target_pose.position`(PoseStamped,frame 必须 = `base_link`) | 桥接偏移与 `harvest_motion.yaml` 的 approach/pick/retreat **工具系偏移是两级不同的偏移**,对接时二选一或明确分工,勿叠加两次 |
| ——(老链路无姿态,用 direction 查四元数库) | `target_pose.orientation`(必须有限且归一化) | 视觉侧需新增固定抓取姿态(可移植老 executor 的 direction→四元数库),或桥接层直接给默认四元数 |
| `confidence`(单值 = 双臂取 max) | `confidence`(每臂独立) | 新执行层已有 `min_confidence: 0.50` 闸;检测阈值 0.80 在上游,两层闸都保留 |
| `source`:`vision_tomato` / `vision_tomato_pick` | `source`(自由字符串) | 建议沿用 `vision_tomato[_pick]` |
| `execute_left` / `execute_right` | `execute_left` / `execute_right`(action goal) | 语义一致,直接映射 |
| ——(无) | `target_id`(重复拒绝 + 日志) | 视觉侧需生成,建议 `vision_<arm>_<时间戳/序号>` |
| `left_direction` / `right_direction`(int,查姿态库) | ——(新接口无此字段) | 姿态选择逻辑需前移到视觉/桥接节点内部 |

桥接节点的门控/去重逻辑(工作空间盒、0.05 m/7 s 抑制、5 s 间隔、双臂 1 s 融合)与新执行层正交,**可整体保留**;但 `vision_task.yaml` 里的工作空间盒和偏移是按老 JAKA 平台标的,**换新机械臂必须全部重标**(新机器人 J1–J3 是移动轴,base_link 布局完全不同)。

## 6. ROS 2 移植备忘(下一步集成时逐条核对)

1. **代码骨架**:`rospy` → `rclpy`;`rosparam` YAML → `declare_parameter`;launch XML → launch_py;`msg/` 三个消息可直接转 ROS2 idl/msg 放进接口包(或 `DualArmTask` 直接废弃,视觉节点改产 `HarvestTarget`)。
2. **相机**:`realsense2_camera` 有 ROS 2 (Jazzy) 版;`align_depth` 对齐深度用法一致;注意 QoS(sensor data)与 `image_transport` 默认行为差异;相机序列号/内参以 `/camera_info` 实测为准,勿沿用 YAML 旧值。
3. **推理依赖**:本地已有 `yolo11` conda 环境(torch 2.5.1+cu121,实测可跑,见 §4.1);ROS2 化时推理部分沿用 **vendored ultralytics + `putong.pt`**(已验证出标准 `(1,9,8400)` 输出),不必重写后处理;`tomato_detector_core.py` 需改的只有 rospy→rclpy 与参数加载方式。相机驱动装 `ros-jazzy-realsense2-camera`,内参以 `/camera_info` 实测为准,勿沿用 YAML 旧值。
4. **手眼标定**:新平台、新相机安装位姿 → `T_camera_to_base` 必须重标(老仓库用 aruco_ros + easy_handeye 的思路可参考,标定约定见 `reference_docs/calibration-and-frames.md`);标定完成前保持 `calibrated: false`,桥接安全闸会自动不发布。
5. **URDF/TF**:老仓库经验 —— 视觉闭环**不强制**把相机建进 URDF;若要 RViz 可视化,先发 `base_link → *_camera_link` 静态 TF,长期再把相机 link 写进 `double_arm_robot.urdf`。
6. **联调顺序**(沿老仓库 runbook):fake/plan_only 验证 → 单臂低速真机 → 双臂;与本工作区 H5 bring-up 流程合流。
