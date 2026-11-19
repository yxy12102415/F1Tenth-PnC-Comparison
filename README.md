# F1TENTH Planning and Control Comparison

在同一张 Fitzpatrick 地图上比较单车规划与控制。全局参考路线长 **44.73 m**，起点、朝向、终点停车条件和每种布局都保存在仓库里。固定路线实验比较控制器；静态绕障实验比较局部规划器及其执行链。

| 实验 | 方法 | 已归档结果 |
|---|---|---|
| 路线跟踪，无障碍 | Pure Pursuit、LQR、Linear MPC、NMPC、MPPI | 3 个起始偏差 × 2 个种子；30/30 完成。[逐控制器指标](results/fitzpatrick_tracking_20261120/comparison.csv) |
| 局部静态绕障 | Hybrid A*、Frenet lattice、kinematic RRT*、Ackermann DWA、car-like TEB | 8 种布局 × 3 个种子；112/120 完成。DWA 直接输出控制，其余四种共用 NMPC。[场景图、失败与逐次结果](results/fitzpatrick_local_design_20261120/README.md) |
| 历史实车基线 | 原 Fitzpatrick v8 Frenet + dynamic MPC | 两个约 0.10 m 直径的障碍，一圈后停车。记录使用 PF 定位估计。新方法需各自取得实车结果。[历史轨迹与数据](results/fitzpatrick_v8_20260927/) |

## 笔记本运行

按安装说明配置 Ubuntu 系统依赖，然后在本仓库根目录运行。[仿真安装与环境](docs/SIM_SETUP.md)

```bash
bash tools/setup_sim.sh
bash Code/sim_ws/run_fitz_tracking.sh --cases center --seeds 11
bash Code/sim_ws/run_fitz_local.sh --gui
```

GUI 默认显示 lattice 在直道交错双障碍上的局部路径、执行轨迹和回放。完整的固定跟踪协议和局部规划协议给出了共享参数、种子、障碍摆放、停车判据与计时范围。[跟踪](docs/FITZPATRICK_FIXED_TRACKING.md) · [规划](docs/FITZPATRICK_LOCAL_PLANNING.md)

## Jetson 上先做什么

本仓库在车上使用独立目录 `~/F1Tenth-PnC-Comparison`。先确认已部署 v8 的环境，再读取实际 ROS 图。下面两条命令不启动车辆驱动。

```bash
cd ~/F1Tenth-PnC-Comparison
bash vehicle_interface/on_car.sh installed-check
bash vehicle_interface/on_car.sh inventory --seconds 2
```

运行实车传感器测试时，原 v8 supervisor 继续拥有车辆。在 **Fitzpatrick 现场**按固定起点放车，确认地图对齐、场地清空、可切断动力并松开 PS4 L1。先运行原安装中的 **zero**，在独立终端捕获 PF、odom、LiDAR 与守护状态，再针对这次捕获运行选定算法。新算法生成候选，正式对比需要逐方法记录。保持其他场地的会话独立，ROS 域 85 内同名节点也可能加载另一条路线。[车上操作与话题表](vehicle_interface/README.md) · [十种算法的实时观察和离线重放](vehicle_interface/ALGORITHMS.md)

在 Jetson Xavier NX 上，静止合成扫描的空场短片产生 80/80 条有效候选；有障碍短片中 52/80 有效。Hybrid A*、lattice、RRT* 和 TEB 的首次避障重规划在该次多方法重放中超过 50 ms，候选被拒绝并锁存。仓库保留了原始拒绝记录及单方法复查。**新方法当前可做传感器采集、影子运行与隔离的 ROS 输出检查，尚未验收实车接管。** [Jetson 实测记录](results/jetson_interface_20261120/README.md)

### 第一次实车采集

在 Jetson 的终端 A 先让观察器等待符合条件的零输出窗口。

```bash
cd ~/F1Tenth-PnC-Comparison
mkdir -p vehicle_runs
run_id=$(date +%Y%m%d-%H%M%S)
sample="$PWD/vehicle_runs/zero-$run_id.jsonl"
bash vehicle_interface/on_car.sh wait-zero "$sample"
```

在终端 B 确认没有其他车辆会话，车辆位于 Fitzpatrick 固定起点，朝向与地图一致，PS4 L1 已松开，现场人员能切断动力。再运行已部署版本的零输出流程。它启动实际传感器与 VESC，禁用驱动输出，完成后自动退出。

```bash
cd ~/f1tenth_real_migration_ws_20260829
bash fitzpatrick_avoidance_dynamic_mpc.sh zero
```

终端 A 显示成功并写入文件后，重放这次实车采集：

```bash
bash vehicle_interface/on_car.sh shadow --input "$sample" --methods pp --output "vehicle_runs/pp-$run_id.jsonl"
```

若观察器报告 `NOT_READY`，从 `identity`、时间戳和缺失话题查原因。车上已有模型可直接重放；新克隆先运行 `bash vehicle_interface/on_car.sh build-shadow`。[命令端隔离验证](vehicle_interface/NOMINAL_STAGE.md)

## 来源与接口边界

仿真实验从开发工作区独立导出。源 checkout 的历史锚点为 `f1tenth-takeover@97bdd9c`，本导出同时包含当时尚未提交的比较代码和结果。原数据档中保存了实验时的源码指纹，本仓库另附复现输入；物理归档中的历史文件路径已改成相对于当时车辆工作区的路径。历史配置和运行数据的出处请见归档说明。[复现档案](results/fitzpatrick_local_design_20261120/reproduction/) · [官方 F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym)

`real_vehicle/` 保存实车资产和旧程序源代码。车上已部署的 v8 由现场原安装管理。只读核对工具从同一套 Foxy overlay 加载模块。`vehicle_interface/commissioned_guard_sources/` 归档该 overlay 的六个源文件和上游 MIT 包元数据以便核对版本。构建在目标架构进行。[保护模块来源](vehicle_interface/commissioned_guard_sources/upstream-package.xml)
