# 低空无人机物流调度系统

本项目面向城市低空物流配送场景，构建了一个包含任务生成、无人机运动、载重与电量约束、充电站、任务超时统计和地图可视化的仿真环境，并实现多种调度方法进行统一评估。

当前项目主要比较三类方法：

- **Greedy**：基于距离、任务优先级和紧迫度的贪心调度。
- **PSO**：基于粒子群优化的批量任务分配，并结合事件驱动与动态贪心重排。
- **MARL**：基于 PyMARL 的 IQL、VDN、QMIX 等多智能体强化学习方法及其改进版本。

## 项目结构

```text
drone-scheduling/
├─ config/
│  ├─ simulation.json          # 仿真环境、任务、无人机和指标配置
│  └─ positions.json           # 任务位置数据
├─ frontend/
│  ├─ environment.py           # 仿真环境与指标统计
│  ├─ drone.py                 # 无人机运动、电量和载重逻辑
│  ├─ task.py                  # 任务模型与任务生成器
│  ├─ charging_station.py      # 充电站模型
│  ├─ evaluate_metrics.py      # Greedy/PSO 统一评估入口
│  ├─ greedy/                  # 贪心调度器及独立运行入口
│  ├─ data/                    # OSM 地图数据
│  └─ assets/                  # 可视化资源
├─ backend_si/
│  ├─ pso_scheduler.py         # PSO 优化器与事件驱动调度器
│  ├─ fitness_evaluator.py     # 多目标适应度计算
│  └─ config.yaml              # PSO 参数配置
├─ backend_wx/pymarl-master/
│  └─ src/
│     ├─ envs/env_drone.py     # PyMARL 无人机环境适配器
│     ├─ config/algs/          # IQL、VDN、QMIX 等算法配置
│     └─ main.py               # 强化学习训练与测试入口
├─ results/
│  ├─ compare/                 # 各算法统一指标 CSV
│  └─ plot_compare_metrics.py  # 指标汇总、归一化评分与绘图
└─ paper/                      # 课程论文、插图和 Overleaf 工程
```

## 仿真场景

默认配置位于 `config/simulation.json`，当前场景包含：

- 10 架无人机；
- 最多生成 60 个配送任务；
- 任务具有重量、优先级、起点、终点和截止时间；
- 支持多任务装载，总载重上限为 5；
- 任务按高峰期、非高峰期和热点区域动态生成；
- 无人机受速度、电池容量、载重能耗和充电站约束；
- 默认单回合最大长度为 2000 个时间步。

主要配置入口：

| 配置文件 | 作用 |
|---|---|
| `config/simulation.json` | Greedy、共享仿真环境和 PyMARL 环境参数 |
| `backend_si/config.yaml` | PSO 粒子数、迭代次数、触发条件和适应度权重 |
| `backend_wx/pymarl-master/src/config/algs/*.yaml` | 强化学习算法与训练参数 |
| `backend_wx/pymarl-master/src/config/envs/env_drone.yaml` | PyMARL 环境注册配置 |

## 调度方法

### Greedy

Greedy 调度器根据无人机状态和候选任务实时生成动作，综合考虑任务距离、优先级、紧迫度、剩余电量和当前载荷。相关代码位于：

- `frontend/greedy/scheduler.py`
- `frontend/greedy/run_greedy.py`

### PSO

PSO 调度器采用“即时分配 + 缓冲区批量优化”机制：

1. 空闲无人机可立即接收合适任务；
2. 其余任务进入缓冲区；
3. 缓冲区达到数量、紧急度或等待时间阈值时触发 PSO；
4. PSO 完成任务分配后，再通过动态贪心调整每架无人机的任务顺序。

详细说明见 `backend_si/README.md`。

### 多智能体强化学习

`backend_wx/pymarl-master` 在 PyMARL 框架中接入无人机调度环境，当前支持：

- IQL / IQL-U
- VDN / VDN-U
- QMIX / QMIX-U

其中 `-U` 表示项目中的改进配置版本。训练环境将任务分配建模为多智能体协作决策问题，并通过共享全局状态和个体观测学习联合调度策略。

## 环境安装

推荐使用 Python 3.9 或 3.10，并为项目创建独立环境。

基础仿真与结果绘图依赖：

```powershell
pip install osmnx==1.9.4 pygame==2.6.1 numpy pyyaml pandas matplotlib
```

运行 PyMARL 还需要安装与本机 CUDA 或 CPU 环境匹配的 PyTorch，以及 Sacred：

```powershell
pip install torch sacred
```

## 运行方法

以下命令均从项目根目录开始执行。

### 1. 评估 Greedy

```powershell
cd frontend
python evaluate_metrics.py --policy greedy --episodes 5 --episode-steps 2000
```

结果追加到：

```text
results/compare/frontend_greedy_metrics.csv
```

如需运行带地图界面的独立 Greedy 入口：

```powershell
cd frontend
python greedy/run_greedy.py --episodes 1 --episode-steps 2000 --seed 100
```

### 2. 评估 PSO

```powershell
cd frontend
python evaluate_metrics.py --policy pso --episodes 5 --episode-steps 2000
```

结果追加到：

```text
results/compare/backend_si_metrics.csv
```

### 3. 训练 QMIX

```powershell
cd backend_wx/pymarl-master/src
python main.py --config=qmix --env-config=env_drone
```

CPU 快速烟测：

```powershell
python main.py --config=qmix --env-config=env_drone with use_cuda=False t_max=10 test_interval=999999 save_model=False use_tensorboard=False
```

将 `qmix` 替换为 `iql`、`vdn`、`coma` 或 `qtran`，可运行对应算法配置。

强化学习评估指标写入：

```text
results/compare/backend_wx_metrics.csv
```

## 统一评估指标

三类方法使用同一组核心指标：

| 指标 | 含义 | 方向 |
|---|---|---|
| Completion Rate | 已完成任务数占已生成任务数的比例 | 越高越好 |
| Timeout Rate | 超时任务占已完成任务的比例 | 越低越好 |
| Average Delay | 超时任务的平均超时时长 | 越低越好 |
| Generation-to-Assignment Wait | 任务从生成到被分配的平均等待时间 | 越低越好 |
| Assignment-to-Loading Wait | 任务从分配到实际装载的平均等待时间 | 越低越好 |
| Loading-to-Delivery Time | 任务装载后到送达的平均时间 | 越低越好 |
| Avg/Max Generation-to-Completion Time | 任务全流程平均/最大耗时 | 越低越好 |
| Priority Average Delay | 不同优先级任务的平均时延 | 越低越好 |
| Total Energy Consumption | 所有无人机的累计能量消耗 | 越低越好 |

## 生成对比结果

完成各算法评估后，在项目根目录运行：

```powershell
python results/plot_compare_metrics.py
```

脚本会读取：

```text
results/compare/frontend_greedy_metrics.csv
results/compare/backend_si_metrics.csv
results/compare/backend_wx_metrics.csv
```

并在 `results/compare/plots` 下生成：

- `combined_compare_metrics.csv`：算法统一指标汇总；
- `normalized_capability_scores.csv`：各维度归一化能力得分；
- `weighted_overall_score.csv`：加权综合得分；
- 各项指标柱状图和综合对比图。

## 结果复现建议

- 对比实验应保持 `config/simulation.json` 中的无人机数量、任务数量、任务生成模式和截止时间一致。
- Greedy 与 PSO 建议使用相同的 `episodes` 和 `episode-steps`。
- 强化学习方法应固定模型、随机种子和测试回合数，并使用测试阶段结果而非训练阶段即时奖励进行横向比较。
- CSV 中可能同时包含多次实验结果；生成论文图表前应确认采用的是目标实验行或对应算法的聚合结果。
- 修改共享指标字段后，应同步检查 Greedy、PSO、PyMARL 写入逻辑及 `results/plot_compare_metrics.py`。

## 当前局限

- PSO 的粒子编码主要优化任务到无人机的分配，单机内部任务顺序仍由动态贪心处理。
- 当前仿真以离散时间步推进，未模拟天气、通信丢包、禁飞区动态变化等真实低空约束。
- 强化学习效果依赖训练轮数、随机种子和奖励权重，不能仅依据单次训练结果判断算法优劣。
- 项目暂未提供统一的根目录依赖锁定文件，不同模块需按实际运行目标安装依赖。
