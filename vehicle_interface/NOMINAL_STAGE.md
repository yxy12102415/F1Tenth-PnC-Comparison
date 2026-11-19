# 隔离验证 v8 标称控制输出

`nominal_stage.py` 为一个选定的方法启动物理参数版 `shadow.py`，其输入是标准输入上的实时 `pnc.vehicle.v1` 传感器快照。适配器通过自己创建的子进程管道读取诊断结果，在本机 ROS 域 143 发布三个 v8 消息。输出主题前缀固定为 `/pnc_comparison/staging/baseline`。此模式用于验证选定算法、单位、消息类型和失效处理。实车运动仍由已部署的 v8 链路控制。

| 主题后缀 | ROS 类型 | 内容 |
|---|---|---|
| `drive_nominal` | `ackermann_msgs/msg/AckermannDriveStamped` | `base_link` 坐标系，发布时间戳，速度 m/s，左正前轮转角 rad |
| `planner_valid` | `std_msgs/msg/Bool` | 本次标称输出通过检查时为真 |
| `controller_status` | `std_msgs/msg/String` | JSON 包含方法、原因、有效位、速度和转角。有效行保留三个源时间、输入序号、计算耗时；`actuation_authorized` 固定为假 |

## 台架自检

在已载入 ROS 2 环境、可导入 `ackermann_msgs` 的终端进入仓库根目录。物理算法的 Python 依赖和构建参数见配套说明。[物理算法构建](ALGORITHMS.md)

```bash
bash vehicle_interface/algorithm_build.sh
python3 vehicle_interface/test_nominal_stage.py
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=143 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python3 vehicle_interface/test_nominal_stage.py --dds
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=143 python3 vehicle_interface/test_nominal_stage.py --dds --stale
```

DDS 自检仅在测试代码里为合成地图扫描设置当前时间，实际运行 Pure Pursuit 子进程并订阅三种消息。它检查首次有效标称命令、重复进程拒绝、输入关闭后归零。负对照在初始化完成后注入一条延迟 2 s 的扫描，要求三个主题仅报告无效、零输出，且保留源时间戳拒绝原因。Jetson 的 Foxy 环境可通过 `bash vehicle_interface/on_car.sh test-nominal-stage` 运行正对照。

## 使用实时快照

在已经运行且经过身份匹配的 v8 会话中，下面的命令将域 85 的只读快照管道连接到独立的域 143。第一条命令的工作模式应与实际 v8 会话一致。第一次运行宜选 `--mode zero`。只有上游进程通过身份、图和消息检查才会送出实时快照。

```bash
set -o pipefail
bash vehicle_interface/on_car.sh preflight-zero --seconds 30 --stream-snapshots | \
  bash vehicle_interface/on_car.sh nominal-stage --method pp
```

从另一个已载入 ROS 的终端观察状态。

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=143 \
  ros2 topic echo /pnc_comparison/staging/baseline/controller_status
```

Python 算法、地图和原生模型初始化完成后，子进程通过私有管道报告就绪。初始化期间排队的传感器字节会被丢弃，下一条完整的实时快照开始计入运行。适配器要求单个 `live_stream` 方法，使用系统 ROS 时钟检查扫描、定位、里程计三个原始时间戳，接受窗口为过去 505 ms 至未来 20 ms。每次输入序号和扫描时间严格递增，定位与里程计时间各自保持单调。它检查预览最小净距 0.065 m、计算耗时上限 50 ms、速度范围 0 至 2 m/s，以及前轮转角绝对值上限 0.269 rad。一次有效输入最多发布一条有效命令。来自文件重放的诊断会被拒绝。

算法报告失败、生产进程退出、输入无效或有效输入中断超过 100 ms 后，适配器持续发布零速度、零转角和假有效位。故障锁存到进程退出，恢复时从静止位重新启动。Ctrl+C 结束该台架发布进程。每个本机网络命名空间限制一个适配器，启动前与运行中检查三个主题的发布者数量。三种 ROS 消息独立传输，消息类型匹配和虚拟域数值检查只覆盖标称接口。当整个适配器进程异常退出时，下游必须依据自身指令超时处理发布者丢失。
