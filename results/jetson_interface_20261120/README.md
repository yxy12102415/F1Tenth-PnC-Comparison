# Jetson 接口验证，2026-11-20

在实际 Xavier NX 的 Foxy / aarch64 / Python 3.8 环境执行。原 v8 的 `check` 返回成功，使用了 GPU、桌面 `:1` 并找到了 VESC 设备。新物理模型在车上用已部署的 acados 重新编译；未打开电机或舵机设备。以下三种观测的含义不同。

| 记录 | 输入和实际测量 |
|---|---|
| [ROS inventory](inventory.json) 和 [zero 预检](zero_observation.json) | 只读观察域 85。最后一次采样未发现运行中的 Fitzpatrick supervisor，zero 返回 NOT_READY，采集 0 帧。另一次单独的现场观察发现 381 BLE 会话曾使用同一域和相同节点名，因此预检还核验运行文件、路线上下文和 supervisor 身份。没有在车上启动任何真实 ROS 驱动或运动会话。 |
| [虚拟 DDS 保护测试](virtual_guard.json) | 独立本机域 142，真实 Foxy final guard 接收合成 Ackermann、scan、odom、planner_valid 和操作员信号。验证未解锁时输出零、有效输入时放行、名义指令断流后锁存。独立运动窗口和末端转换逻辑在内存中验证 L1 释放、过期和零电流。 |
| [空场](jetson_clear.jsonl.gz)、[单障碍](jetson_obstacle.jsonl.gz)、[单方法复查](jetson_single_planner.jsonl.gz)、[汇总](replay_summary.json) | 输入取自 [隔离 DDS 合成记录](../vehicle_shadow_20261120/manifest.json)。各 8 个时刻，初始静止。按当前 v8 物理几何和约束运行，使用系统 SciPy、原生 C++、BLAS 单线程。每种方法使用相同输入。 |

空场十种方法全部生成 8/8 条有效候选。障碍场的五个跟踪器及 DWA 各为 8/8；四个路径规划器各为 1/8。主动重规划首次超过 50 ms 时锁存，后续无效行是同一故障的延续，不能当成独立失败试次。短片用于验证时间戳、输入输出、资源开销和拒绝行为，无闭环圈速或实物净距结论。CPU 调度优先级为 nice 10，数据来自短时间窗口，耗时不代表最坏情况或长跑分布。完整 v8 地面成绩另见 [9 月历史记录](../fitzpatrick_v8_20260927/)。
