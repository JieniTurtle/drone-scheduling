# 项目说明

本项目包含三种可对比的方法：

1. `frontend` 贪心策略（Greedy）
2. `backend_si` PSO 调度策略
3. `backend_wx` QMIX 强化学习策略

三者统一比较以下三个指标：

1. 完成率（completion_rate）
2. 准时率（on_time_rate）
3. 平均时延（avg_delay）


# 环境安装

## 操作系统

Ubuntu 22.04 / Windows（均可）

## Python 依赖

```bash
pip install osmnx==1.9.4
pip install pygame==2.6.1
```


# 配置说明（重点）

## 为什么看起来有两份配置文件

当前保留了两套配置入口：

1. `config/simulation.json`：frontend + backend_wx 的统一共享配置
2. `backend_si/config.yaml`：backend_si 自身配置

这样设计的原因是：`backend_si` 已按要求恢复为原逻辑，不再改其内部读取方式，因此它仍默认读取 `backend_si/config.yaml`。

## 参数修改建议

1. 改 frontend / backend_wx 公共参数：修改 `config/simulation.json`
2. 改 backend_si 的 PSO 参数：修改 `backend_si/config.yaml`


# 运行命令

以下命令以 Windows + `citysim` 环境为例。

## 1) 运行 Greedy（frontend）

```powershell
cd frontend
python evaluate_metrics.py --policy greedy --episodes 5 --episode-steps 1200
```

## 2) 运行 PSO（backend_si）

```powershell
cd frontend
python evaluate_metrics.py --policy pso --episodes 5 --episode-steps 1200
```

说明：评估入口在 frontend，但 `--policy pso` 会调用 backend_si 的调度器。

## 3) 运行 QMIX（backend_wx）

```powershell
cd backend_wx/pymarl-master/src
python main.py --config=qmix --env-config=env_drone
```

快速烟测（建议先跑）：

```powershell
cd backend_wx/pymarl-master/src
python --config=qmix --env-config=env_drone with use_cuda=False t_max=10 test_interval=999999 save_model=False use_tensorboard=False
```


# 结果保存位置

统一对比指标文件都在：`results/compare`

1. `results/compare/frontend_greedy_metrics.csv`
2. `results/compare/backend_si_metrics.csv`
3. `results/compare/backend_wx_metrics.csv`


# 三种方法指标比较

在项目根目录执行：

```powershell
python compare_metrics.py
```

输出会汇总三种方法的：

1. completion_rate 平均值
2. on_time_rate 平均值
3. avg_delay 平均值
