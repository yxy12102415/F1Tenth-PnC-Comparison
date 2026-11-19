# Fitzpatrick v8 实车接口

在 Jetson 本机可用这个观察器核对当前运行的 ROS 2 Foxy 节点、传感器、控制输出和下游守护节点。实车基准是固定起点 Fitzpatrick v2 路线、Hokuyo、GPU PF、VESC 和 `ground_fitzpatrick_joint_20260927_v8`。`inventory` 可以在空图以及其他赛道的会话中运行。采集和影子计算只接受经过身份核验的 Fitzpatrick v8 会话。[原启动器](../real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/stand_follow.py) · [十种方法及重放入口](ALGORITHMS.md)

## 车上命令

在独立仓库根目录执行。包装脚本加载已部署车辆的 Foxy 和工作区 overlay，将观察器限制到本机 DDS 域 85。先使用运行现场已有的会话。如果需要指定安装位置，将 `PNC_COMMISSIONED_WORKSPACE` 设为包含 `candidate_cpp_gpu_20260909` 的已部署目录。

```bash
bash vehicle_interface/on_car.sh inventory --seconds 3
bash vehicle_interface/on_car.sh preflight-zero --seconds 3
bash vehicle_interface/on_car.sh preflight-run --seconds 3
```

`preflight-zero` 用于现场已运行的 v8 零输出会话，`preflight-run` 用于现场已运行、带操作员门控的 v8 地面会话。两条命令都只观察。检查器先解析同机运行的 Python 监督器、规范化脚本路径、会话参数和启动时刻，再核对 SHA-256，包括车上安装的地图 YAML、PGM、起点、路线、速度剖面、运行时资产、关键脚本及两个 C++ 可执行文件。两个 C++ 子进程必须属于该监督器、运行匹配的二进制并引用该版本的资产。从其他赛道复用的节点名和话题名可以出现在同一域，报告会用 `DIFFERENT_RELEASE`、`MISSING_SUPERVISOR` 等说明实际情况。源文件归档存在路径规范化，运行时校验使用车上已部署文件的原始字节摘要。[身份校验及摘要](session_identity.py)

| 模式 | 结果 | 含义 |
|---|---|---|
| `inventory` | `OBSERVED`，退出 0 | 已读取当前节点及指定话题的 DDS 端点，`ready` 为 null。可查看 `identity.status`。`optional_graph` 列出地图、路线、初始化、可视化及诊断端点，`map_services` 记录地图服务的实际类型。 |
| `zero` 或 `run` | `OBSERVED_MATCH`，退出 0 | 观察窗口内确认了进程身份、规定的写入方、必需订阅方、类型、可靠性与持久性兼容、有效同步输入和守护状态。 |
| `zero` 或 `run` | `NOT_READY`，退出 1 | `issues` 给出缺失、冲突、超时或无效状态。未匹配会话时只读取图，不订阅车辆数据。 |
| 任意 | `UNVERIFIED`，退出 2 | 域或环境不符、ROS 或端点查询不可用、文件输出失败等。 |

报告中的 `motion_authorized` 始终为 false。读取 ROS 图和运行进程能验证软件连接，物理车辆对齐、安装外参、可用刹停距离、实际车轮转角和扭矩仍需要现场独立观测。读者节点可以与原系统共存。检查器不发布车辆话题，也避开上游用订阅者数量判定发现成功的 `/stand_pf/permit` 和 `/initialpose`。采样窗口以单调时钟计时，报告给出每个已采样话题的计数和末值年龄。

## ROS 数据契约

| 话题 | 类型及坐标系 | 字段、生产者及 QoS |
|---|---|---|
| `/baseline/raw_scan` → `/baseline/scan` | `sensor_msgs/msg/LaserScan`，`laser` | URG 原始扫描由 `stand_scan_relay` 保留原 header 和角度、量程、光束顺序。自上一次转发不足 45 ms 时跳过。中继发布 best effort、volatile、深度 1。v8 C++ 使用 SensorDataQoS、深度 5。 |
| `/baseline/localization_pose` | `nav_msgs/msg/Odometry`，`map` → `base_link` | GPU PF 发布平面 `pose.pose` 和协方差，header 保留触发 PF 的扫描时间；规划器和控制器取 x、y 和平面四元数。可靠、volatile、深度 1。 |
| `/baseline/odom` | `nav_msgs/msg/Odometry`，`odom` → `base_link` | `stand_vesc_odometry` 发布。两个 C++ 节点读取车体系 `abs(twist.linear.x)`（m/s）、`linear.y`（m/s）、`angular.z`（rad/s），请求可靠、volatile、深度 1。v8 校验横向速度绝对值 ≤2.5、角速度绝对值 ≤8。 |
| `/baseline/localization_valid` | `std_msgs/msg/Bool` | PF 的无 header 质量标志，要求最新值 true。可靠、volatile、深度 1。 |
| `/baseline/local_plan_cpp` | `std_msgs/msg/String`，JSON 中的 `frame_id=map` | 规划器向控制器发送局部路径、计划编号、有效位、生成单调时间和原始扫描、位姿时间。标称计划间隔约 0.20 s。可靠、volatile、深度 1。 |
| `/baseline/drive_nominal` | `ackermann_msgs/msg/AckermannDriveStamped`，`base_link` | 控制器以 ROS 当前时间盖章，前向速度 0–2.00 m/s，转向 ±0.269 rad。50 ms 控制周期。随 `/baseline/planner_valid`（Bool）和 `/baseline/controller_status`（JSON String）发布。三者可靠、volatile、深度 1。 |
| `/baseline/lattice_selected`、`/baseline/confirmed_obstacles` | `nav_msgs/msg/Path`、`visualization_msgs/msg/MarkerArray` | C++ 规划器提供地图系观测。地图服务是 `/map_server/map`，地图话题 `/baseline/map` 使用 transient local、深度 1。 |

扫描角度和姿态角用弧度，位置和激光量程用米。激光相对 `base_link` 前移 0.27 m，平面横移和偏航偏置为零。v8 从最多 80 个 PF 位姿中按扫描时间对齐，区间宽度最多 0.15 s，区间外邻近端点最多允许 0.075 s。先插值车体航向的最短角差，再把激光端点变换到地图系。该策略依赖车体外参和地图坐标现场一致。[C++ 输入检查及发布](../real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/src/cpp_frenet/node.cpp) · [扫描时间对齐及外参](../real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/src/cpp_frenet/perception.hpp) · [车载 PF](../real_vehicle/baseline_pf_pp_20260908/src/baseline_pf_pp/baseline_pf_pp/safe_pf.py)

三个带 header 的传感器流分别要求源时间严格递增。v8 输入判断采用 ROS 当前时间减源时间，允许 −0.020 至 0.505 s，同时要求单调时钟接收年龄在 0 至 0.50 s。质量、许可和使能 Bool 采用接收年龄。内部计划的 `generated_mono` 属于同机单调时钟，需和 ROS header 秒独立处理。监视器实际接收到有效样本且拓扑匹配才输出同步帧。零输出模式中的使能、L1 或马达速度命令允许缺席；驻车时的零电流流仍须存在。

## 从操作者到驱动

1. 运行模式下 `route_operator_gate` 读取 `/baseline/joy` 的按钮 4（PS4 L1）并观察名义控制和有效位，发布 `/baseline/arm`、`/baseline/deadman`、`/baseline/enable`。零输出模式不启动该节点。操作员释放、再次按住、输入新鲜度和监督器就绪条件均由既有车辆侧实施。
2. `final_command_guard` 综合名义 Ackermann、扫描、里程计、规划有效位、arm 和 deadman，向 `/baseline/final` 独占发布 Ackermann，`base_link`、控制上限 2.00 m/s 与 0.27 rad。v8 将相关超时设为 0.50 s，关闭目标信念依赖。报告验证 final 守护的锁存、配置标志和状态。[Final 守护源码](commissioned_guard_sources/final_command_guard.py)
3. `stand_window` 结合 final 时间、L1、监督器 `/stand_pf/permit` 与独立 `/stand_pf/health_permit`，以 50 Hz 向 `/stand_pf/actuator_input` 发命令。它自行盖章并保留默认空 frame_id。单次窗口上限 90 s，释放、超时等情况使窗口停止。健康来自 VESC 遥测桥接的 `/stand_pf/health`，包括原始 ROS 时间、eRPM、电流、电压、占空比、故障和 FET 温度。独立健康节点需要连续有效数据完成启动资格判定。[单次运行窗口](../real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/stand_window.py) · [健康判定](../real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/src/ground_pp/health_core.hpp)
4. `hardware_actuator_guard` 独占发布 Float64 的 `/stand_pf/motor_speed`（7500 eRPM/(m/s)，上限 15000 eRPM）、`/stand_pf/motor_current`（零电流滑行）及 `/stand_pf/servo`（归一化命令，−1.2135 × 转角 + 0.4304，范围 0.10–0.76）。零输出模式把驱动输入改到 `/stand_pf/servo_disabled`，电流和速度上下限均为零。零输出采样还要求窗口报告未启动、许可未生效且速度转角为零，执行器报告零 eRPM、零电流与居中舵机。域内重映射后的制动、电机位置、占空比和禁用舵机端口不得出现其他写入者。[实车执行器守护](commissioned_guard_sources/hardware_actuator_guard.py) · [命令转换与滑行](commissioned_guard_sources/safety_core.py)
5. `/stand_pf/servo_feedback` 的值是 VESC 驱动回发的**已裁剪指令**。运行模式的里程计根据速度与该指令估算角速度，零输出模式关闭此计算。VESC 核心遥测提供速度、电流等测量。车辆没有通过这个接口测量实际舵角，动态控制的转角、横向速度及偏航率观测器要和真实传感器读数区分。[驱动舵机回调](../real_vehicle/stand_speed_15_monitor_v1/src/vesc_driver/src/vesc_driver.cpp)

## 同步帧和实时流

```bash
bash vehicle_interface/on_car.sh capture zero /tmp/fitz-v8-recording.jsonl --seconds 5
set -o pipefail
bash vehicle_interface/on_car.sh preflight-zero --seconds 10 --stream-snapshots 2>/tmp/fitz-observer.json \
  | bash vehicle_interface/on_car.sh shadow --input - --methods pp --output /tmp/fitz-shadow.jsonl
```

文件采集使用新路径，通过复核后原子创建。标准流为逐行 JSON，最后的观察结果写到 stderr。诊断消费者慢时管道会施加背压，消费者对收到的数据再次校验实际延迟。流中已经交付的行对应交付当时的观察条件，最终状态和退出码应随结果保存。

每行 `schema=pnc.vehicle.v1`。三个 ROS 源时间以秒记录为 `stamp`（当前 PF）、`odom_stamp`、`scan_stamp`。当前地图位姿为 `x,y,yaw`，车体运动为 `v,vy,yaw_rate`。扫描时刻的地图系车体位姿在 `scan_pose=[x,y,yaw]`，另附 `laser_offset_x,angle_min,angle_increment,range_min,range_max,ranges`。非有限光束写为 JSON null，在点投影时跳过。每次接受新扫描最多生成一行。姿态与里程计允许在相邻帧复用仍然新鲜的样本。

## 复核

```bash
python3 vehicle_interface/test_preflight.py
source /opt/ros/humble/setup.bash   # Jetson 使用已安装的 Foxy
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=141 python3 vehicle_interface/test_ros_transport.py
bash vehicle_interface/on_car.sh test-guard
```

第一项使用标准库核验传感器、两种时钟、坐标系、拓扑冲突、守护状态，以及其他赛道进程、重复进程、进程重启和资产变更。第二项只在本机域 141 的临时测试命名空间中生成四种合成输入，经过 ROS 序列化进入与实车相同的订阅和同步代码。它支持 `--input` 传入同步的 20 Hz 合成地图扫描和 `--output` 导出经 DDS 的结果。第三项在独立域 142 验证守护消息链，通过包装脚本调用。车辆状态、测试域数据和已有地面圈速应分别解读。[Jetson 现场核查](../results/jetson_interface_20261120) · [影子算法测试证据](../results/vehicle_shadow_20261120)

2026-09-27 的 v8 合格零输出档案保留了 621 个零 final、781 个零电流、781 个归一化值为 0.4304 的舵机命令，速度模式消息数为 0。按日志对齐的 5.009 s 区间有 98 个有效的 waiting_for_operator 控制状态，健康桥的 230 个 VESC 样本均通过当前字段与源时间检查。随后正常关闭引发执行器超时。运动实测档案的对应活动区间为 26.672 s，533 个控制状态有效，健康桥 1331 个样本通过检查。日志保存了状态变化及控制值，原始规划有效 Bool、enable/arm/deadman、端点身份与 DDS QoS 仍须在物理图单独观察。[历史消息字段、原文件摘要与采样定义](../results/jetson_interface_20261120/archived_status_check.json)
