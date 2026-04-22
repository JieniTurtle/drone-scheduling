# PSO调度器集成说明

## 概述
已将粒子群优化（PSO）算法集成到无人机配送调度系统中，替换原有的贪心算法。

## 文件修改

### 1. scheduler.py
- 添加了 `PSOScheduler` 类，封装PSO调度逻辑
- 保留了原有的 `GreedyScheduler` 类（可选使用）
- 自动导入 `backend_sl` 模块中的PSO实现

### 2. test.py
- 将调度器从 `GreedyScheduler` 改为 `PSOScheduler`
- 添加了更详细的输出信息

## 使用方法

### 运行仿真
```bash
cd frontend
python test.py
```

### 切换回贪心算法（可选）
如果需要使用贪心算法，修改 `test.py`：
```python
from scheduler import GreedyScheduler  # 改为导入GreedyScheduler
action = GreedyScheduler.schedule_for_drone(observations)
```

## PSO算法参数

当前配置（在 `scheduler.py` 的 `PSOScheduler` 类中）：
- **粒子数量**: 20
- **最大迭代次数**: 30
- **优化目标权重**:
  - 准时率: 0.4
  - 平均时延: 0.4
  - 能耗: 0.2

### 调整参数
如需调整参数，修改 `frontend/scheduler.py` 中的 `PSOScheduler.schedule_for_drone()` 方法：

```python
scheduler = PSOSchedulerBackend(
    num_particles=20,      # 调整粒子数量
    max_iterations=30,     # 调整迭代次数
    weights={
        'on_time_rate': 0.4,   # 调整准时率权重
        'avg_delay': 0.4,      # 调整时延权重
        'energy': 0.2          # 调整能耗权重
    }
)
```

## 依赖安装

PSO算法需要numpy：
```bash
pip install numpy
```

如果numpy未安装，系统会自动回退到贪心算法。

## 性能对比

| 指标 | 贪心算法 | PSO算法 |
|------|---------|---------|
| 准时率 | 较低 | 较高 |
| 平均时延 | 较高 | 较低 |
| 计算速度 | 快 | 较慢 |

PSO算法在优化效果上优于贪心算法，但计算时间稍长。

## 算法原理

PSO算法通过模拟鸟群觅食行为，在解空间中搜索最优调度方案：
1. 初始化粒子群（随机调度方案）
2. 评估每个粒子的适应度（准时率、时延等）
3. 更新粒子速度和位置
4. 迭代优化直到收敛

适应度函数综合考虑：
- 任务准时完成率
- 平均配送时延
- 能耗（预留扩展）

## 扩展说明

如需添加新的优化目标，参考 `backend_sl/README.md` 中的扩展指南。
