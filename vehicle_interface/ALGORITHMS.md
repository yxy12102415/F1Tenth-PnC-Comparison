# 十种方法的实车接口影子测试

这个入口将同步的 PF 位姿、VESC 里程计和 Hokuyo 扫描交给实际算法，输出按方法分开的 JSONL 诊断。它支持实时接收和离线重放。ROS 进程只负责订阅。算法进程通过本地管道调用 C++，没有 ROS publisher，也没有电机或舵机通信接口。每条输出都带有 `actuation_authorized=false`。

## 构建与复现

从仓库根目录运行。先将 `ACADOS_INSTALL_DIR` 设为本机架构的 acados v0.4.0 安装前缀。Python 依赖为 NumPy、SciPy、Pillow、PyYAML。编译使用 C/C++14、Eigen3 和 jsoncpp。Jetson 必须从源码本机编译。命令生成 v8 地图距离场并编译随仓库提供的物理模型 C 求解器。

```sh
bash vehicle_interface/algorithm_build.sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 vehicle_interface/test_shadow.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 vehicle_interface/shadow.py \
  --input results/vehicle_shadow_20261120/obstacle.capture.jsonl.gz \
  --output /tmp/fitz-shadow-results.jsonl
```

输出路径须为新文件。默认同时比较全部十种方法。选择一个方法可传 `--methods lattice`。无参数的首次投影从固定起点开始；中途停车采集可用 `--start-s` 提供已知的本圈起始弧长（米）。首帧速度必须在 0.10 m/s 以内，因为控制器从静止、零转角初始化。重放模式使用记录中的传感器时间；`--input -` 专供当前实车系统时钟的实时流，会拒绝旧数据。

在已运行的 v8 零输出图中，设置本机 Foxy 环境和项目 overlay 后可实时观察。先在同一 Jetson 运行只读预检，选择单个算法，以免其他方法的串行耗时延迟观察。地面试验的既有 ROS 图由现场独立启动。

```sh
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=85
set -o pipefail
python3 vehicle_interface/preflight.py --mode zero --seconds 20 --stream-snapshots \
  | OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 vehicle_interface/shadow.py \
      --input - --methods pp --output /tmp/live-pp-shadow.jsonl
```

如需保留正式采样证据，可另行运行 `preflight.py --mode zero --seconds 20 --snapshots capture.jsonl`，再用 `--input capture.jsonl` 重放。预检对图、QoS、帧名、PF 质量和接收时效的要求见 [实车接口](README.md)。实时流按当前系统时钟再次检查扫描、里程计和定位时间。连续 0.5 秒无数据、源时间倒退、采样间隔不能满足控制周期、分析器超时或无可行解都会留下诊断。方法拒绝后保持无效，直到从静止状态重新启动一轮。退出码 0 表示所有已计算候选通过本轮检查，1 表示至少一个方法失败，2 表示输入或运行环境无效。

## 输入、模型和输出

v8 感知以扫描时刻的 map→base_link 位姿和车前 0.27 m 的激光外参投影真实量程，执行原 C++ 地图滤除和多帧跟踪。局部规划器从确认后的点簇得到圆盘，以点簇中心到最远已观测点的距离加 0.10 m 表示未见深度。此裕量取决于障碍物形状，真实箱体外形需现场核验。v8 里程计的横向速度和角速度缺少独立惯性测量；NMPC 和 MPPI 使用自身模型估计横向运动。PP、LQR、线性 MPC 和 DWA 使用运动学模型。依据 [v8 物理控制参数](../real_vehicle/candidate_cpp_gpu_20260909/ground_fitzpatrick_joint_20260927_v8/controller_config.py)，轴距 0.25 m，最大命令 2.0 m/s，转向限幅 ±0.269 rad。NMPC 和 MPPI 使用物理电机时间常数 0.15 s 和舵机时间常数 0.20 s。所有算法的名义周期为 50 ms。

| 方法 | 实际执行的计算 | 现场尚需验证 |
|---|---|---|
| PP、LQR | 使用比较程序的 PP 预瞄或速度调度 DARE 增益，配置实车轴距、转角和纵向限制 | 预瞄/增益，零速起步，滑行终停 |
| Linear MPC | 同一凝聚线性自行车 QP，Foxy 通过 SciPy SLSQP 求解，约束转角和增量 | 实车 QP 耗时及约束下的跟踪误差 |
| NMPC | v8 物理生成代码、SQP-RTI、v8 横向观测器与 terminal coast，接入本次比较的定时参考 | 新参考和接口组合下的整圈效果 |
| MPPI | CPU 采样器按物理轮胎、轴距、加减速度和舵机模型预测，并检查选中轨迹 | 采样预算、模型偏差及终停，台架输出仍缺闭环测试 |
| Hybrid A*、Lattice、RRT*、TEB | 各自实际调用搜索、采样或弹性带优化，再由物理 NMPC 跟踪；TEB 保留优化后的时间间隔 | 真实障碍和负载下的可行率、重复规划延迟、回线 |
| DWA | 使用实车轴距、转角、加减速度的可达窗口直接生成命令，并检查预测弧；可行集为空时拒绝 | 运动学预测中的转向响应误差及停车距离 |

局部路径缓存每次使用前均用当帧障碍、静态地图和现有前向路径重查间隙。重入目标延长或车辆偏离缓存时重新规划。最终轨迹另作插值检查，空间步长上界由平移与转动共同估算为 0.02 m。v8 地图间隙按车辆中心的 0.25 m 半径计算，动态障碍间隙使用纵向偏移 −0.08、0、+0.08 m 的三个半径 0.15 m 圆。固定路线三种经典跟踪器检查参考走廊，其输出没有独立动力学轨迹预测。

每条记录带有 `schema=pnc.vehicle.shadow.v1` 和 `source_mode`（`capture_replay` 或 `live_stream`）。日志中的 `speed`（m/s）和 `steering`（rad，左正）为候选值，`stamp` 为扫描 ROS 秒。无效时两列为 `null`，`reason` 记录首次错误或已锁存的原因。每个方法的 `compute_ms` 包含当帧共享感知、该方法规划、求解和预览检查。文件重放中的多个方法串行执行，此数字不能用于推算并行运行十种方法的实时性。实时模式另外检查输出时的消息年龄。未实测的环节包括轮胎/地面响应、PF 实际误差和急停停车距离。

## 已运行的验证

在 x86_64 笔记本上，以实际 Fitzpatrick PGM 生成 1081 线扫描，经过本机隔离的 ROS 2 Humble 消息发布、订阅和同步后，从同一静止位置重放各 8 帧。空场和单个圆柱两个场景分别得到 80/80 个通过检查的候选。障碍物第二帧确认，Hybrid A*、Lattice、RRT* 和 TEB 各生成新轨迹，DWA 直接计算命令。第一次主动重规划分别为 25.9、10.7、33.4 和 45.3 ms。另一次直接重放中 TEB 耗时 66.9 ms，50 ms 截止检查正确置空该方法输出。另用沿弯道起步的 32 帧合成数据逐帧更新位姿、里程计和扫描，终速 0.661 m/s、行程 0.341 m，十种方法合计 320/320 个候选通过检查。该段轨迹预先给定，各方法的命令仅作为日志输出。真实车辆图、旧 v8 地面试验、Jetson 本机运行和本次合成 ROS 测试需分别解读。[原始输入、输出和哈希](../results/vehicle_shadow_20261120/manifest.json)

同一接口还在真车 Jetson 上进行了**合成输入重放**。空场得到 80/80 个有效诊断。障碍场为 52/80，五个跟踪器和 DWA 各为 8/8。Hybrid A*、Lattice、RRT*、TEB 首次主动重规划耗时分别为 **185.1、73.2、179.2、238.1 ms**，各方法第一帧直行通过，第二帧超时、其余帧锁存。对应方法的同步实现未达到该板的 20 Hz 截止要求。后续可按 v8 的独立规划与控制线程设计，令 20 Hz 控制器只消费带采样时间、生成时间和有效期限的完整计划，同时在每个控制周期重新检查当前感知和预测轨迹；超过租期仍需归零。该设计仍需用本机负载和真实采样验证。 [Jetson 空场输出](../results/jetson_interface_20261120/jetson_clear.jsonl.gz) · [Jetson 障碍输出](../results/jetson_interface_20261120/jetson_obstacle.jsonl.gz)

已有 v8 地面证据对应当时的 v8 规划器、控制器和安全链。使新方法能够发出实车指令，还需要将选定方法作为唯一 `/baseline/drive_nominal` 与 `/baseline/planner_valid` 写入者接入既有 supervisor，验证控制器状态身份和 freshness、计划失效/终点滑行处理、操作员 L1 释放及独立运动窗口，随后验证传感器失联、截止超时和急停时驱动归零。项目中的 Gym 求解器和 Gym 几何仅用于复现比较；物理构建脚本和编译断言检查选中的模型，尚未完成地面验收的方法当前只生成诊断文件。[已记录的 v8 地面结果](../results/fitzpatrick_v8_20260927/analysis_summary.json)
