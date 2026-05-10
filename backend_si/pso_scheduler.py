"""
粒子群优化算法（PSO）用于无人机任务调度
优化目标：任务准时率、平均时延

本文件包含两个类：
  - PSOOptimizer：PSO 算法核心，纯优化器（输入 drones+tasks → 输出分配）
  - PSOScheduler：对外调度器，事件驱动，双通道 + 内层动态贪心，
                  内部通过 PSOOptimizer 完成批量分配
配置参数集中在 backend_si/config.yaml。
"""
import os
import numpy as np
import math
import copy
import yaml
from typing import List, Dict, Tuple, Optional


class Particle:
    """
    粒子类：
      position  ∈ R^(num_tasks, num_drones) 实值偏好矩阵
      assignment[i] = argmax_d position[i, d] 得到任务 i 分配的无人机
    """

    def __init__(self,
                 num_drones: int,
                 num_tasks: int,
                 v_max: float = 4.0,
                 pos_max: float = 5.0):
        """
        :param num_drones: 无人机数量
        :param num_tasks: 任务数量
        :param v_max: velocity 每个分量的绝对值上限
        :param pos_max: position 每个分量的绝对值上限（防数值发散）
        """
        self.num_drones = num_drones
        self.num_tasks = num_tasks
        self.v_max = v_max
        self.pos_max = pos_max

        # 偏好矩阵 [task, drone]，连续实值
        self.position = np.random.uniform(-1.0, 1.0, (num_tasks, num_drones))
        self.velocity = np.random.uniform(-0.5, 0.5, (num_tasks, num_drones))

        # 个体最优快照（完整偏好矩阵，不是 assignment 向量）
        self.best_position = self.position.copy()
        self.best_fitness = float('-inf')

        # 当前适应度
        self.fitness = float('-inf')

    def update_velocity(self, global_best_position, w=0.7, c1=1.5, c2=1.5):
        """
        标准连续 PSO 速度更新（在偏好矩阵上做元素级运算）
        :param global_best_position: 全局最优偏好矩阵 (num_tasks, num_drones)
        """
        r1 = np.random.random((self.num_tasks, self.num_drones))
        r2 = np.random.random((self.num_tasks, self.num_drones))

        cognitive = c1 * r1 * (self.best_position - self.position)
        social = c2 * r2 * (global_best_position - self.position)
        self.velocity = w * self.velocity + cognitive + social

        self.velocity = np.clip(self.velocity, -self.v_max, self.v_max)

    def update_position(self):
        """位置更新：偏好矩阵直接累加 velocity，clip 防发散"""
        self.position = self.position + self.velocity
        self.position = np.clip(self.position, -self.pos_max, self.pos_max)


class PSOOptimizer:
    """粒子群优化算法核心：给定无人机集合和任务集合，输出 task→drone 分配"""
    
    def __init__(self,
                 num_particles=30,
                 max_iterations=50,
                 w=0.7,
                 c1=1.5,
                 c2=1.5,
                 v_max: float = 4.0,
                 pos_max: float = 5.0,
                 weights: Optional[Dict[str, float]] = None,
                 seed: Optional[int] = None,
                 battery_consumption_base: float = 0.5,
                 battery_load_penalty_factor: float = 0.3,
                 battery_low_threshold: float = 0.2,
                 env_step_seconds: float = 0.1):
        """
        初始化PSO调度器
        :param num_particles: 粒子数量
        :param max_iterations: 最大迭代次数
        :param w: 惯性权重
        :param c1: 个体学习因子
        :param c2: 社会学习因子
        :param v_max: 粒子 velocity 每个分量的绝对值上限
        :param pos_max: 粒子 position 每个分量的绝对值上限（防数值发散）
        :param weights: 适应度函数权重字典，支持扩展
                       默认: {'on_time_rate': 0.4, 'avg_delay': 0.4, 'energy': 0.2}
        :param seed: 随机数种子，用于实验可复现性
        :param battery_consumption_base: 单位距离基础耗电（与 frontend/drone.py 保持一致）
        :param battery_load_penalty_factor: 满载相对空载的耗电增量比例
        :param battery_low_threshold: 低电量阈值（占容量比例），低于则触发去充电站
        :param env_step_seconds: 1 个 env-step 对应的真实秒数（与 frontend/drone.py
                                 中 DRONE_TIME_STEP 同义；用于把 drone 的"米/秒"、
                                 充电站的"Wh/秒"换算到 env-step 量纲，使 fitness
                                 累计时间与 deadline / current_time 同口径）
        """
        self.num_particles = num_particles
        self.max_iterations = max_iterations
        self.w = w
        self.c1 = c1
        self.c2 = c2
        self.v_max = v_max
        self.pos_max = pos_max

        # 电量参数（与 frontend/drone.py 中的常量一一对应）
        self.battery_consumption_base = battery_consumption_base
        self.battery_load_penalty_factor = battery_load_penalty_factor
        self.battery_low_threshold = battery_low_threshold

        # 时间单位换算：fitness 内累计的"时间"必须与 deadline / current_time 同为
        # env-step；drone 速度以"米/秒"给出，env-step 长度 = env_step_seconds 秒
        self.env_step_seconds = env_step_seconds

        # 设置随机数种子
        if seed is not None:
            np.random.seed(seed)

        # 适应度函数权重（可扩展）
        if weights is None:
            self.weights = {
                'on_time_rate': 0.4,    # 准时率权重
                'avg_delay': 0.4,        # 平均时延权重
                'energy': 0.2            # 能耗权重
            }
        else:
            self.weights = weights
        
        self.particles: List[Particle] = []
        self.global_best_position = None
        self.global_best_fitness = float('-inf')
        self.fitness_history = []
    
    @staticmethod
    def euclidean_distance(pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """计算欧氏距离"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def calculate_travel_time(self, distance: float, speed: float = 200.0) -> float:
        """
        计算飞行 distance 所需的 env-step 数。

        说明：speed 是"米/秒"，env-step 时长 = self.env_step_seconds 秒，
        所以一个 env-step 内 drone 实际飞 speed * env_step_seconds 米。
        返回值与 environment 的 current_time / task.deadline 同口径。
        """
        denom = speed * self.env_step_seconds
        if denom <= 0:
            return 0.0
        return distance / denom

    def _battery_consumption(self, distance: float, current_load: float,
                             carrying_capacity: float) -> float:
        """
        计算飞行 distance 后消耗的电量（Wh），与 frontend/drone.py:consume_battery 保持一致：
            消耗 = distance * BASE * (1 + (load / capacity) * LOAD_PENALTY_FACTOR)
        """
        if carrying_capacity <= 0:
            load_factor = 0.0
        else:
            load_factor = (current_load / carrying_capacity) * self.battery_load_penalty_factor
        return distance * self.battery_consumption_base * (1.0 + load_factor)
    
    def evaluate_fitness(self,
                        position: np.ndarray,
                        drones_info: List[Dict],
                        tasks_info: List[Dict],
                        current_time: float) -> Tuple[float, Dict[str, float]]:
        """
        评估适应度函数（支持多目标优化和扩展）
        :param position: 粒子偏好矩阵 (num_tasks, num_drones)；内部 argmax 解码出分配
        :param drones_info: 无人机信息列表
        :param tasks_info: 任务信息列表
        :param current_time: 当前时间
        :return: (总适应度, 各指标详情字典)
        """
        num_tasks = len(tasks_info)
        num_drones = len(drones_info)

        # 偏好矩阵 → 任务到无人机的离散分配
        assignment = np.argmax(position, axis=1)

        # 为每个无人机分配任务
        drone_tasks = [[] for _ in range(num_drones)]
        for task_idx, drone_idx in enumerate(assignment):
            drone_tasks[int(drone_idx)].append(task_idx)

        # 计算各项指标
        on_time_count = 0
        total_delay = 0.0
        total_completion_time = 0.0
        total_energy = 0.0  # 累积每架无人机消耗的电量（Wh）

        # 用于能耗归一化：取首架无人机的电池容量为参考量级（所有无人机容量一致）
        ref_battery_capacity = drones_info[0].get('battery_capacity', 15000.0) if drones_info else 15000.0

        for drone_idx, task_indices in enumerate(drone_tasks):
            if not task_indices:
                continue

            drone_info = drones_info[drone_idx]
            drone_pos = tuple(drone_info['position'])
            drone_capacity = drone_info.get('capacity', 5)
            battery_capacity = drone_info.get('battery_capacity', 15000.0)
            current_battery = drone_info.get('battery', battery_capacity)
            station_pos = tuple(drone_info.get('charging_station_position', drone_pos))
            charging_power = drone_info.get('charging_power', 50.0)
            warehouse_pos = tuple(drone_info.get('home_position', drone_pos))

            current_pos = drone_pos
            current_load = 0
            accumulated_time = current_time

            # 按优先级排序任务（高优先级优先执行）
            sorted_tasks = sorted(task_indices,
                                key=lambda idx: tasks_info[idx]['priority'],
                                reverse=True)

            for task_idx in sorted_tasks:
                task = tasks_info[task_idx]
                task_weight = task['weight']

                # 任务开始前：若电量低于阈值，先去充电站补电（与 drone.py 中
                # 任务结束后判断 is_low_battery → 飞往充电站的逻辑一致）
                if (battery_capacity > 0
                        and current_battery / battery_capacity < self.battery_low_threshold):
                    dist_to_station = self.euclidean_distance(current_pos, station_pos)
                    consumed = self._battery_consumption(
                        dist_to_station, current_load, drone_capacity)
                    current_battery = max(0.0, current_battery - consumed)
                    total_energy += consumed
                    accumulated_time += self.calculate_travel_time(dist_to_station)
                    current_pos = station_pos
                    # 充满电的耗时：drone.charge() 每个 env-step 充入
                    # charging_power * env_step_seconds Wh，所以补满需要的 env-step 数 =
                    # (capacity - battery) / (charging_power * env_step_seconds)
                    charge_rate_per_step = charging_power * self.env_step_seconds
                    if charge_rate_per_step > 0:
                        accumulated_time += (battery_capacity - current_battery) / charge_rate_per_step
                    current_battery = battery_capacity

                # 检查载重，如果超载需要返回仓库
                if current_load + task_weight > drone_capacity:
                    return_distance = self.euclidean_distance(current_pos, warehouse_pos)
                    consumed = self._battery_consumption(
                        return_distance, current_load, drone_capacity)
                    current_battery = max(0.0, current_battery - consumed)
                    total_energy += consumed
                    accumulated_time += self.calculate_travel_time(return_distance)
                    current_pos = warehouse_pos
                    current_load = 0

                # 飞往取货点
                source = tuple(task['source'])
                distance_to_source = self.euclidean_distance(current_pos, source)
                consumed = self._battery_consumption(
                    distance_to_source, current_load, drone_capacity)
                current_battery = max(0.0, current_battery - consumed)
                total_energy += consumed
                accumulated_time += self.calculate_travel_time(distance_to_source)

                # 装货
                current_load += task_weight
                current_pos = source

                # 飞往目的地（载重提升 → 单位耗电增大）
                destination = tuple(task['destination'])
                distance_to_dest = self.euclidean_distance(source, destination)
                consumed = self._battery_consumption(
                    distance_to_dest, current_load, drone_capacity)
                current_battery = max(0.0, current_battery - consumed)
                total_energy += consumed
                accumulated_time += self.calculate_travel_time(distance_to_dest)

                # 卸货
                current_load -= task_weight
                current_pos = destination

                # 计算完成时间和延迟
                completion_time = accumulated_time
                total_completion_time += completion_time

                deadline = task['deadline']
                if deadline is not None and deadline != float('inf'):
                    delay = max(0, completion_time - deadline)
                    total_delay += delay
                    if delay == 0:
                        on_time_count += 1
                else:
                    # 没有截止时间的任务视为准时
                    on_time_count += 1

        # 计算各项指标
        on_time_rate = on_time_count / num_tasks if num_tasks > 0 else 0
        avg_delay = total_delay / num_tasks if num_tasks > 0 else 0
        avg_energy = total_energy / num_drones if num_drones > 0 else 0

        # 归一化处理（将指标映射到[0, 1]区间）
        # 准时率：越高越好，已在[0, 1]
        normalized_on_time = on_time_rate

        # 平均时延：越低越好，使用负指数函数归一化
        normalized_delay = math.exp(-avg_delay / 1000.0) if avg_delay > 0 else 1.0

        # 能耗：以电池容量为参考量级，平均每机消耗的电量越接近一整块电池得分越低；
        # 超过一整块电池则得 0（强惩罚密集充电场景）
        if ref_battery_capacity > 0:
            normalized_energy = max(0.0, 1.0 - avg_energy / ref_battery_capacity)
        else:
            normalized_energy = 1.0

        # 加权求和计算总适应度
        fitness = (self.weights['on_time_rate'] * normalized_on_time +
                  self.weights['avg_delay'] * normalized_delay +
                  self.weights['energy'] * normalized_energy)
        
        # 返回详细指标
        metrics = {
            'on_time_rate': on_time_rate,
            'avg_delay': avg_delay,
            'avg_energy': avg_energy,
            'normalized_on_time': normalized_on_time,
            'normalized_delay': normalized_delay,
            'normalized_energy': normalized_energy,
            'total_fitness': fitness
        }
        
        return fitness, metrics
    
    def optimize(self, 
                drones_info: List[Dict],
                tasks_info: List[Dict],
                current_time: float,
                verbose: bool = True) -> Tuple[Dict[int, List[str]], Dict]:
        """
        执行PSO优化
        :param drones_info: 无人机信息列表
        :param tasks_info: 任务信息列表
        :param current_time: 当前时间
        :param verbose: 是否打印优化过程
        :return: (调度方案, 优化统计信息)
        """
        num_drones = len(drones_info)
        num_tasks = len(tasks_info)
        
        if num_tasks == 0:
            return {}, {'message': 'No tasks to schedule'}
        
        # 初始化粒子群（每个粒子都持有自己的 (num_tasks, num_drones) 偏好矩阵）
        self.particles = [
            Particle(num_drones, num_tasks, v_max=self.v_max, pos_max=self.pos_max)
            for _ in range(self.num_particles)
        ]
        self.global_best_position = None
        self.global_best_fitness = float('-inf')
        self.fitness_history = []
        
        # 迭代优化
        for iteration in range(self.max_iterations):
            for particle in self.particles:
                # 评估当前位置的适应度
                fitness, metrics = self.evaluate_fitness(
                    particle.position, drones_info, tasks_info, current_time
                )
                particle.fitness = fitness
                
                # 更新个体最优
                if fitness > particle.best_fitness:
                    particle.best_fitness = fitness
                    particle.best_position = particle.position.copy()
                
                # 更新全局最优
                if fitness > self.global_best_fitness:
                    self.global_best_fitness = fitness
                    self.global_best_position = particle.position.copy()
            
            # 记录历史
            self.fitness_history.append(self.global_best_fitness)
            
            # 打印进度
            if verbose and (iteration % 10 == 0 or iteration == self.max_iterations - 1):
                print(f"Iteration {iteration}: Best Fitness = {self.global_best_fitness:.4f}")
            
            # 更新粒子速度和位置
            for particle in self.particles:
                particle.update_velocity(self.global_best_position, self.w, self.c1, self.c2)
                particle.update_position()
        
        # 生成最终调度方案
        assignments = self._generate_assignments(
            self.global_best_position, drones_info, tasks_info
        )
        
        # 计算最终指标
        _, final_metrics = self.evaluate_fitness(
            self.global_best_position, drones_info, tasks_info, current_time
        )
        
        stats = {
            'iterations': self.max_iterations,
            'best_fitness': self.global_best_fitness,
            'fitness_history': self.fitness_history,
            'metrics': final_metrics
        }
        
        return assignments, stats
    
    def _generate_assignments(self,
                             position: np.ndarray,
                             drones_info: List[Dict],
                             tasks_info: List[Dict]) -> Dict[int, List[str]]:
        """
        根据粒子位置（偏好矩阵）生成调度方案
        :param position: 偏好矩阵 (num_tasks, num_drones)，先 argmax 解码
        :param drones_info: 无人机信息
        :param tasks_info: 任务信息
        :return: {drone_index: [task_id_list]}
        """
        num_drones = len(drones_info)
        assignments = {i: [] for i in range(num_drones)}

        assignment = np.argmax(position, axis=1)
        for task_idx, drone_idx in enumerate(assignment):
            task_id = tasks_info[task_idx]['task_id']
            assignments[int(drone_idx)].append(task_id)

        return assignments


# 默认配置文件路径（与本模块同目录下的 config.yaml）
DEFAULT_CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'config.yaml')


class PSOScheduler:
    """
    事件驱动 PSO 调度器（对外）：双通道任务分配 + 内层动态贪心排序。

    所有可调超参数集中在 backend_si/config.yaml；要换实验配置，传 config_path。

    工作流程（每个仿真步调一次 step()）：
      1. 检测新到达任务：
         - 若有"空闲且队列为空"的无人机 → 立即就近贪心分配（直通通道）
         - 否则进入 pending_buffer，等待批量 PSO（缓冲通道）
      2. 检测刚变空闲的无人机 → 对其剩余任务队列做动态贪心重排
      3. 检查 buffer 是否满足 flush 条件（size / emergency / timeout 任一）
         → 调用 PSOOptimizer 做批量分配，结果追加进各机队列
      4. 给空闲无人机派发队头任务
    """

    def __init__(self,
                 num_drones: int,
                 config_path: Optional[str] = None,
                 verbose: bool = True,
                 seed: Optional[int] = None):
        self.num_drones = num_drones
        self.verbose = verbose
        self.config = self._load_config(config_path or DEFAULT_CONFIG_PATH)

        # 解包常用配置项
        dc = self.config['dual_channel']
        self.buffer_size_threshold = dc['buffer_size_threshold']
        self.emergency_ttl = dc['emergency_ttl']
        self.buffer_timeout = dc['buffer_timeout']

        pso_cfg = self.config['pso']
        self.pso_num_particles = pso_cfg['num_particles']
        self.pso_max_iterations = pso_cfg['max_iterations']
        self.pso_inertia = pso_cfg['inertia']
        self.pso_cognitive = pso_cfg['cognitive']
        self.pso_social = pso_cfg['social']
        # 方案 A 偏好矩阵编码新增参数（向后兼容旧 config 缺失的情况）
        self.pso_v_max = pso_cfg.get('v_max', 4.0)
        self.pso_pos_max = pso_cfg.get('pos_max', 5.0)

        self.fitness_weights = dict(self.config['fitness_weights'])
        self.greedy_weights = dict(self.config['dynamic_greedy'])
        self.drone_capacity = self.config['drone']['capacity']
        # frontend Drone.update() 默认 time_step=0.1；fitness 中所有"时间"必须按
        # env-step 累计才能与 deadline / current_time 同口径
        self.env_step_seconds = float(self.config['drone'].get('time_step', 0.1))

        # 电量参数（与 frontend/drone.py 中常量保持同名同义；缺省值即 drone.py 的默认值）
        battery_cfg = self.config.get('battery', {})
        self.battery_capacity = float(battery_cfg.get('capacity', 15000.0))
        self.battery_consumption_base = float(battery_cfg.get('consumption_base', 0.5))
        self.battery_load_penalty_factor = float(battery_cfg.get('load_penalty_factor', 0.3))
        self.battery_low_threshold = float(battery_cfg.get('low_threshold', 0.2))
        
        # 随机数种子（优先使用参数，其次使用配置文件，最后不设置）
        if seed is None:
            seed = self.config.get('seed')
        self.seed = seed
        if self.seed is not None:
            np.random.seed(self.seed)

        # 运行时状态
        self.drone_queues: Dict[int, List[Dict]] = {i: [] for i in range(num_drones)}
        self.pending_buffer: List[Dict] = []
        self.buffer_entry_time: Dict[str, float] = {}
        self.known_task_ids: set = set()
        self.prev_is_free: List[bool] = [True] * num_drones
        self.active_task_ids: Dict[int, str] = {}

        # 事件统计
        self.stats = {
            'immediate_assign': 0,
            'buffered': 0,
            'flush_size': 0,
            'flush_emergency': 0,
            'flush_timeout': 0,
            'reorders': 0,
            'dispatches': 0,
        }

    # ----------- 对外入口 -----------

    def step(self, observation: Dict, current_time: float) -> Dict[int, List[str]]:
        """
        每个仿真步调用一次。
        :param observation: 环境观察空间，需要含 'drone_positions', 'unassigned_tasks',
                            以及 'drone_is_free' 或 'drone_free_masks'
        :param current_time: 当前时间步
        :return: {drone_idx: [task_id]} 动作字典（仅向当前空闲且有队列的无人机派发）
        """
        is_free = self._extract_is_free(observation)

        # 1. 新任务检测与处理
        for task in observation.get('unassigned_tasks', []):
            tid = task['task_id']
            if tid in self.known_task_ids:
                continue
            self.known_task_ids.add(tid)
            self._on_new_task(task, observation, is_free, current_time)

        # 2. 空闲事件检测与处理（无人机刚送完货）
        for i in range(self.num_drones):
            if not self.prev_is_free[i] and is_free[i]:
                self.active_task_ids.pop(i, None)
                self._on_drone_freed(i, observation, current_time)

        # 3. buffer flush 检查
        self._maybe_flush_buffer(observation, current_time)

        # 4. 派发：空闲 + 无活动任务 + 有队列 → 派发队头
        action: Dict[int, List[str]] = {}
        for i in range(self.num_drones):
            if is_free[i] and i not in self.active_task_ids and self.drone_queues[i]:
                next_task = self.drone_queues[i].pop(0)
                action[i] = [next_task['task_id']]
                self.active_task_ids[i] = next_task['task_id']
                self.stats['dispatches'] += 1

        self.prev_is_free = is_free
        return action

    def get_stats(self) -> Dict[str, int]:
        return dict(self.stats)

    # ----------- 事件处理 -----------

    def _on_new_task(self, task, observation, is_free, current_time):
        # 用绝对 deadline 替代陈旧的 ttl，防止任务在队列里躺久后 ttl 过期
        ttl = task.get('remaining_time', float('inf'))
        task['_abs_deadline'] = (current_time + ttl) if ttl != float('inf') else float('inf')

        available = [i for i in range(self.num_drones)
                     if is_free[i]
                     and i not in self.active_task_ids
                     and len(self.drone_queues[i]) == 0]

        if available:
            drone_positions = observation['drone_positions']
            source = tuple(task['source'])
            best = min(available,
                       key=lambda i: self._dist(tuple(drone_positions[i]), source))
            self.drone_queues[best].append(task)
            self.stats['immediate_assign'] += 1
            if self.verbose:
                print(f"[PSOScheduler] 立即分配 {task['task_id']} → drone_{best}（最近空闲机）")
        else:
            self.pending_buffer.append(task)
            self.buffer_entry_time[task['task_id']] = current_time
            self.stats['buffered'] += 1

    def _on_drone_freed(self, drone_idx, observation, current_time):
        queue = self.drone_queues[drone_idx]
        if len(queue) <= 1:
            return
        drone_pos = tuple(observation['drone_positions'][drone_idx])
        self.drone_queues[drone_idx] = self._dynamic_greedy_reorder(
            queue, drone_pos, current_time
        )
        self.stats['reorders'] += 1
        if self.verbose:
            print(f"[PSOScheduler] drone_{drone_idx} 空闲，动态贪心重排 {len(queue)} 个队列任务")

    def _maybe_flush_buffer(self, observation, current_time):
        if not self.pending_buffer:
            return

        reason = None
        if len(self.pending_buffer) >= self.buffer_size_threshold:
            reason = 'size'
        if reason is None:
            for task in self.pending_buffer:
                ttl = task.get('remaining_time', float('inf'))
                if ttl != float('inf') and ttl < self.emergency_ttl:
                    reason = 'emergency'
                    break
        if reason is None:
            oldest = min(self.buffer_entry_time.get(t['task_id'], current_time)
                         for t in self.pending_buffer)
            if current_time - oldest > self.buffer_timeout:
                reason = 'timeout'

        if reason is None:
            return

        self._flush_buffer_with_pso(observation, current_time, reason)

    def _flush_buffer_with_pso(self, observation, current_time, reason):
        # 充电站位置与功率：优先用 observation 中暴露的字段（与实际仿真一致），
        # 缺失时回退到第一架无人机的当前位置 / 默认充电功率
        station_info = observation.get('charging_station', {}) or {}
        station_pos = tuple(station_info.get('position',
                                              observation['drone_positions'][0]))
        charging_power = float(station_info.get('charging_power', 50.0))

        # 电量信息（每机一份）
        battery_obs = observation.get('drone_batteries')

        # 每架无人机的 PSO 起点 = 已有队尾任务的目的地（无队列则用当前机位）
        drones_info = []
        for i in range(self.num_drones):
            if self.drone_queues[i]:
                origin = tuple(self.drone_queues[i][-1]['destination'])
            else:
                origin = tuple(observation['drone_positions'][i])

            if battery_obs is not None and i < len(battery_obs):
                bcap = float(battery_obs[i].get('capacity', self.battery_capacity))
                bcur = float(battery_obs[i].get('current', bcap))
            else:
                bcap = self.battery_capacity
                bcur = bcap

            drones_info.append({
                'index': i,
                'position': origin,
                'capacity': self.drone_capacity,
                'home_position': tuple(observation['drone_positions'][i]),
                'battery_capacity': bcap,
                'battery': bcur,
                'charging_station_position': station_pos,
                'charging_power': charging_power,
            })

        tasks_info = [{
            'task_id': t['task_id'],
            'source': t['source'],
            'destination': t['destination'],
            'deadline': t.get('_abs_deadline', float('inf')),
            'priority': t['priority'],
            'weight': t['weight'],
        } for t in self.pending_buffer]

        optimizer = PSOOptimizer(
            num_particles=self.pso_num_particles,
            max_iterations=self.pso_max_iterations,
            w=self.pso_inertia,
            c1=self.pso_cognitive,
            c2=self.pso_social,
            v_max=self.pso_v_max,
            pos_max=self.pso_pos_max,
            weights=self.fitness_weights,
            seed=self.seed,
            battery_consumption_base=self.battery_consumption_base,
            battery_load_penalty_factor=self.battery_load_penalty_factor,
            battery_low_threshold=self.battery_low_threshold,
            env_step_seconds=self.env_step_seconds,
        )
        assignments, _ = optimizer.optimize(
            drones_info, tasks_info, current_time=current_time, verbose=False
        )

        # drones_info 顺序 = 全局 index，PSO 返回的 local_idx 即 global_idx
        for drone_idx, task_ids in assignments.items():
            for task_id in task_ids:
                task = next((t for t in self.pending_buffer
                             if t['task_id'] == task_id), None)
                if task is not None:
                    self.drone_queues[drone_idx].append(task)

        for task in self.pending_buffer:
            self.buffer_entry_time.pop(task['task_id'], None)
        self.pending_buffer.clear()

        # 对队列长度 > 1 的无人机做一次动态贪心重排修正顺序
        for i in range(self.num_drones):
            if len(self.drone_queues[i]) > 1:
                start_pos = tuple(observation['drone_positions'][i])
                self.drone_queues[i] = self._dynamic_greedy_reorder(
                    self.drone_queues[i], start_pos, current_time
                )

        self.stats[f'flush_{reason}'] += 1
        if self.verbose:
            summary = {k: len(v) for k, v in self.drone_queues.items()}
            print(f"[PSOScheduler] PSO 批量分配（触发: {reason}），当前各机队列: {summary}")

    # ----------- 工具方法 -----------

    def _dynamic_greedy_reorder(self, tasks, start_pos, current_time):
        ordered = []
        remaining = list(tasks)
        current_pos = start_pos
        while remaining:
            best = max(remaining,
                       key=lambda t: self._score_task(t, current_pos, current_time))
            ordered.append(best)
            remaining.remove(best)
            current_pos = tuple(best['destination'])
        return ordered

    def _score_task(self, task, current_pos, current_time):
        priority = task.get('priority', 1)
        abs_deadline = task.get('_abs_deadline')
        if abs_deadline is None:
            ttl = task.get('remaining_time', float('inf'))
        else:
            ttl = (abs_deadline - current_time) if abs_deadline != float('inf') else float('inf')
        source = tuple(task['source'])
        distance = self._dist(current_pos, source)

        if ttl == float('inf'):
            urgency = 0.0
        elif ttl <= 0:
            urgency = 10.0  # 已超期 → 强制前置
        else:
            urgency = 1.0 / max(1.0, ttl)

        w = self.greedy_weights
        return (w['priority'] * priority
                + w['urgency'] * urgency
                - w['distance'] * distance)

    def _extract_is_free(self, observation):
        is_free = observation.get('drone_is_free')
        if is_free is not None and len(is_free) == self.num_drones:
            return list(is_free)
        # 兜底：从旧的 drone_free_masks 推断
        masks = observation.get('drone_free_masks', [])
        return [all(m) if m else True for m in masks]

    @staticmethod
    def _load_config(path: str) -> Dict:
        with open(path, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)

    @staticmethod
    def _dist(p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
