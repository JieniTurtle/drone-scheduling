import osmnx as ox
import numpy as np
import heapq
import time
import datetime
from shapely.geometry import Point, LineString
import math
from drone import Drone, BATTERY_CONSUMPTION_BASE, BATTERY_LOAD_PENALTY_FACTOR
from charging_station import DEFAULT_CHARGING_STATIONS
from config.config_loder import get_shared_config
# from test import OptimizedMapViewer
from tools.osm import load_map_data, get_building_location_by_name, get_global_bounds
from task import TaskGenerator
from map_drawer import OptimizedMapViewer
from task import WAREHOUSE_POS
from seed_interface import apply_seed


CFG = get_shared_config()
ENV_CFG = CFG.get("environment", {})
DRONE_CFG = CFG.get("drone", {})
DEFAULT_EPISODE_MAX_STEPS = int(ENV_CFG.get("episode_max_steps", 1200))
DEFAULT_NUM_DRONES = int(ENV_CFG.get("num_drones", 3))
PRINT_ROUTE_DEBUG = bool(ENV_CFG.get("print_route_debug", False))
ALLOW_MULTI_TASK = bool(ENV_CFG.get("allow_multi_task", True))
MAX_OBS_TASKS = int(ENV_CFG.get("max_obs_tasks", 20))
MAX_MULTI_TASK_TOTAL_WEIGHT = float(ENV_CFG.get("multi_task_max_total_weight", DRONE_CFG.get("carrying_capacity", 5)))

class Environment:
    def __init__(self, osm_file_path, visualize=False, episode_max_steps=DEFAULT_EPISODE_MAX_STEPS):
        _, buildings_with_height = load_map_data(osm_file_path)

        self.global_bounds = get_global_bounds(buildings_with_height)

        self.high_buildings = [b for b in buildings_with_height if b['height'] is not None and b['height'] > 20]

        task_cfg = CFG.get("task", {})
        task_file = task_cfg.get("positions_file", '../config/positions.json')
        self.task_generator = TaskGenerator(file_path=task_file)
        self.unassigned_tasks = []  # 保持向后兼容，通过 task_generator 访问
        self.drones = []  # 初始化无人机列表
        self.episode_max_steps = int(episode_max_steps)
        # 创建多架无人机（从仓库位置出发）
        NUM_DRONES = DEFAULT_NUM_DRONES
        self.drones = [
            Drone(WAREHOUSE_POS[0], WAREHOUSE_POS[1], drone_id=f"drone_{i}")
            for i in range(NUM_DRONES)
        ]
        
        # 初始化模拟时间
        self.current_time = 0
        
        # 跟踪已完成的任务，用于奖励计算
        self.completed_tasks = []
        
        # 跟踪无人机分配的任务：{drone_idx: {'task': task, 'start_time': time}}
        self.drone_assignments = {}
        
        # 跟踪每个无人机之前的空闲状态
        self._prev_free_status = {i: True for i in range(len(self.drones))}

        self.reward_step = float(ENV_CFG.get("reward_step", -0.1))
        self.reward_priority_1 = float(ENV_CFG.get("reward_priority_1", 100.0))
        self.reward_priority_2 = float(ENV_CFG.get("reward_priority_2", 80.0))
        self.reward_priority_3 = float(ENV_CFG.get("reward_priority_3", 50.0))
        self.penalty_late_rate = float(ENV_CFG.get("penalty_late_rate", -0.01))
        self.overdue_initial_penalty = float(ENV_CFG.get("overdue_initial_penalty", -10.0))
        self.overdue_step_penalty = float(ENV_CFG.get("overdue_step_penalty", -0.1))
        self._overdue_task_ids = set()
        self.generated_task_times = []

        # 统计指标（按回合统计）
        self.total_generated_tasks = 0
        self.total_completed_tasks = 0
        self.total_on_time_tasks = 0
        self.total_delay = 0.0
        self.total_generation_to_assignment_wait = 0.0
        self.total_assignment_to_load_wait = 0.0
        self.total_wait_time = 0.0
        self.total_delivery_time = 0.0
        self.total_generation_to_completion_time = 0.0
        self.max_delivery_time = 0.0
        self.max_generation_to_completion_time = 0.0
        self.priority_delay_totals = {1: 0.0, 2: 0.0, 3: 0.0}
        self.priority_delay_counts = {1: 0, 2: 0, 3: 0}
        
        # 耗电量统计
        self.total_energy_consumed = 0.0  # 总耗电量 (Wh)
        self.total_charging_energy = 0.0  # 总充电量 (Wh)
        self.total_charging_sessions = 0  # 总充电次数
        self.total_flight_distance = 0.0  # 总飞行距离

        # 初始生成一批任务
        new_tasks = self.task_generator.generate_initial_tasks(self.current_time)
        self.total_generated_tasks = len(new_tasks)
        self.generated_task_times = [t.get_generation_time() for t in new_tasks]
        print(f"Generated {len(new_tasks)} tasks:")
        for task in new_tasks:
            print(f"  {task}")

        self.viewer = None
        if visualize:
            self.viewer = OptimizedMapViewer('data/map/part_of_yangpu.osm')

        self._episode_seed = None

        # print(get_building_location_by_name( self.high_buildings, "衷和楼"))
        print(f"加载了 {len(self.high_buildings)} 个具有高度信息的建筑物")

    def get_global_bounds(self):
        return self.global_bounds

    def get_high_buildings(self):
        return self.high_buildings
    
    def _print_task_completion(self, drone_id, task_id, delivery_time, expected_time, 
                               delay, is_on_time, priority, weight):
        """打印任务完成信息"""
        status = "✓ 准时" if is_on_time else "✗ 延迟"
        print(f"\n{'='*70}")
        print(f"📦 任务完成: {task_id}")
        print(f"{'='*70}")
        print(f"  无人机: {drone_id}")
        print(f"  优先级: {priority}/3  |  货物重量: {weight}kg")
        print(f"  配送时长: {delivery_time:.1f} 时间单位")
        print(f"  预期时长: {expected_time:.1f} 时间单位")
        if not is_on_time:
            print(f"  延迟时间: {delay:.1f} 时间单位")
        print(f"  状态: {status}")
        print(f"{'='*70}")
    
    def _record_task_completion(self, drone_idx, assignment):
        """记录单个任务完成的统计数据"""
        completed_task = assignment['task']
        assigned_time = assignment.get('assigned_time', assignment.get('start_time', self.current_time))
        load_time = assignment.get('load_time')
        if load_time is None:
            load_time = assigned_time
        completion_time = self.current_time
        generation_time = completed_task.get_generation_time()
        if generation_time is None:
            generation_time = assigned_time

        generation_to_assignment_wait = max(0, assigned_time - generation_time)
        assignment_to_load_wait = max(0, load_time - assigned_time)
        delivery_time = max(0, completion_time - load_time)
        generation_to_completion_time = max(0, completion_time - generation_time)

        deadline = completed_task.get_deadline()
        if deadline is not None:
            delay = max(0, completion_time - deadline)
        else:
            delay = 0

        is_on_time = delay <= 0
        self.total_completed_tasks += 1
        self.total_delay += float(delay)
        self.total_generation_to_assignment_wait += float(generation_to_assignment_wait)
        self.total_assignment_to_load_wait += float(assignment_to_load_wait)
        self.total_wait_time += float(assignment_to_load_wait)
        self.total_delivery_time += float(delivery_time)
        self.total_generation_to_completion_time += float(generation_to_completion_time)
        self.max_delivery_time = max(self.max_delivery_time, float(delivery_time))
        self.max_generation_to_completion_time = max(
            self.max_generation_to_completion_time,
            float(generation_to_completion_time),
        )
        priority = int(completed_task.get_priority())
        if priority in self.priority_delay_totals:
            self.priority_delay_totals[priority] += float(delay)
            self.priority_delay_counts[priority] += 1
        if is_on_time:
            self.total_on_time_tasks += 1

        self.completed_tasks.append({
            'task': completed_task,
            'generation_time': generation_time,
            'assigned_time': assigned_time,
            'load_time': load_time,
            'start_time': load_time,
            'completion_time': completion_time,
            'generation_to_assignment_wait': generation_to_assignment_wait,
            'assignment_to_load_wait': assignment_to_load_wait,
            'wait_time': assignment_to_load_wait,
            'delivery_time': delivery_time,
            'generation_to_completion_time': generation_to_completion_time,
            'delay': delay
        })

        if PRINT_ROUTE_DEBUG:
            drone_id = self.drones[drone_idx].drone_id if drone_idx < len(self.drones) else f"drone_{drone_idx}"
            expected_time = deadline - load_time if deadline else 0
            self._print_task_completion(
                drone_id, completed_task.task_id, delivery_time,
                expected_time, delay, is_on_time,
                completed_task.get_priority(), completed_task.get_weight()
            )

    def get_statistics(self):
        """获取当前统计指标"""
        completion_rate = 0.0
        if self.total_generated_tasks > 0:
            completion_rate = self.total_completed_tasks / self.total_generated_tasks

        avg_generation_time = 0.0
        if len(self.generated_task_times) >= 2:
            diffs = [
                b - a
                for a, b in zip(self.generated_task_times, self.generated_task_times[1:])
                if b >= a
            ]
            if diffs:
                avg_generation_time = sum(diffs) / len(diffs)

        if self.total_completed_tasks == 0:
            return {
                'completion_rate': completion_rate,
                'total_completed': 0,
                'total_generated': self.total_generated_tasks,
                'on_time_rate': 0.0,
                'avg_delay': 0.0,
                'timeout_rate': 0.0,
                'avg_generation_to_assignment_wait': 0.0,
                'avg_assignment_to_load_wait': 0.0,
                'avg_wait_time_to_load': 0.0,
                'avg_load_to_delivery_time': 0.0,
                'avg_delivery_time': 0.0,
                'avg_generation_to_completion_time': 0.0,
                'avg_steps_per_order': 0.0,
                'avg_generation_time': avg_generation_time,
                'max_delivery_time': 0.0,
                'max_generation_to_completion_time': 0.0,
                'avg_delay_priority_1': 0.0,
                'avg_delay_priority_2': 0.0,
                'avg_delay_priority_3': 0.0,
                'total_wait_time': self.total_wait_time,
                'total_generation_to_completion_time': self.total_generation_to_completion_time,
                'total_energy_consumed': self.total_energy_consumed,
                'avg_energy_per_task': 0.0,
                'avg_energy_per_distance': 0.0,
            }
        
        avg_energy_per_task = self.total_energy_consumed / self.total_completed_tasks
        
        avg_steps_per_order = self.total_delivery_time / self.total_completed_tasks
        avg_generation_to_assignment_wait = self.total_generation_to_assignment_wait / self.total_completed_tasks
        avg_assignment_to_load_wait = self.total_assignment_to_load_wait / self.total_completed_tasks
        avg_wait_time_to_load = avg_assignment_to_load_wait
        avg_generation_to_completion_time = self.total_generation_to_completion_time / self.total_completed_tasks
        priority_avg_delays = {}
        for priority in (1, 2, 3):
            count = self.priority_delay_counts.get(priority, 0)
            priority_avg_delays[priority] = (
                self.priority_delay_totals.get(priority, 0.0) / count
                if count > 0 else 0.0
            )

        return {
            'completion_rate': completion_rate,
            'total_completed': self.total_completed_tasks,
            'total_generated': self.total_generated_tasks,
            'on_time_rate': self.total_on_time_tasks / self.total_completed_tasks,
            'timeout_rate': 1.0 - (self.total_on_time_tasks / self.total_completed_tasks),
            'avg_delay': self.total_delay / self.total_completed_tasks,
            'avg_generation_to_assignment_wait': avg_generation_to_assignment_wait,
            'avg_assignment_to_load_wait': avg_assignment_to_load_wait,
            'avg_wait_time_to_load': avg_wait_time_to_load,
            'avg_load_to_delivery_time': self.total_delivery_time / self.total_completed_tasks,
            'avg_delivery_time': self.total_delivery_time / self.total_completed_tasks,
            'avg_generation_to_completion_time': avg_generation_to_completion_time,
            'avg_steps_per_order': avg_steps_per_order,
            'avg_generation_time': avg_generation_time,
            'max_delivery_time': self.max_delivery_time,
            'max_generation_to_completion_time': self.max_generation_to_completion_time,
            'avg_delay_priority_1': priority_avg_delays[1],
            'avg_delay_priority_2': priority_avg_delays[2],
            'avg_delay_priority_3': priority_avg_delays[3],
            'total_wait_time': self.total_wait_time,
            'total_generation_to_completion_time': self.total_generation_to_completion_time,
            'total_energy_consumed': self.total_energy_consumed,
            'avg_energy_per_task': avg_energy_per_task,
        }
    
    def print_statistics(self):
        """打印累计统计信息"""
        stats = self.get_statistics()
        print(f"\n{'='*70}")
        print(f"📊 累计统计")
        print(f"{'='*70}")
        print(f"  完成率: {stats['completion_rate']:.2%}")
        print(f"  完成任务数: {stats['total_completed']}")
        print(f"  准时率: {stats['on_time_rate']:.2%}")
        print(f"  平均延迟: {stats['avg_delay']:.2f} 时间单位")
        print(f"  平均装载等待时间: {stats['avg_wait_time_to_load']:.2f} 时间单位")
        print(f"  平均配送时长: {stats['avg_delivery_time']:.2f} 时间单位")
        print(f"  平均生成到完成时长: {stats['avg_generation_to_completion_time']:.2f} 时间单位")
        print(f"  平均生成间隔: {stats['avg_generation_time']:.2f} 时间单位")
        print(f"  总耗电量: {stats['total_energy_consumed']:.2f} Wh")
        if stats['total_completed'] > 0:
            print(f"  平均每任务耗电: {stats['avg_energy_per_task']:.2f} Wh")
        print(f"{'='*70}\n")

    def step(self, actions):
        # actions 格式: {drone_index: [task_id_list]}
        for drone_idx, task_ids in actions.items():
            if drone_idx < len(self.drones):
                drone = self.drones[drone_idx]

                # 判断无人机是否正在飞往某个取货点（尚未抵达 source 航点）
                pending_source = None
                if not drone.is_free:
                    for wp in drone.scheduled_position:
                        if len(wp) >= 3 and wp[2] == 'source':
                            pending_source = (wp[0], wp[1])
                            break

                for task_id in task_ids:
                    if task_id is not None:
                        # 找到对应的任务
                        task_to_assign = None
                        for task in self.task_generator.unassigned_tasks:
                            if task.task_id == task_id:
                                task_to_assign = task
                                break

                        if task_to_assign is not None:
                            planned_total_weight = float(drone.current_load) + float(task_to_assign.get_weight())
                            if planned_total_weight > MAX_MULTI_TASK_TOTAL_WEIGHT or planned_total_weight > float(drone.carrying_capacity):
                                continue

                            same_source = (
                                ALLOW_MULTI_TASK
                                and pending_source is not None
                                and task_to_assign.get_source() == pending_source
                            )

                            if same_source:
                                # 同源追加：从当前路线最后一个航点规划到新目的地
                                last_wp = drone.scheduled_position[-1]
                                last_pos = (last_wp[0], last_wp[1])
                                new_route = self.plan_route_around_buildings(
                                    last_pos, task_to_assign.get_destination()
                                )
                                typed_route = []
                                for j, pt in enumerate(new_route):
                                    tag = 'dest' if j == len(new_route) - 1 else 'waypoint'
                                    typed_route.append((pt[0], pt[1], tag))
                                drone.append_route(typed_route)

                                # 追加到分配记录（兼容旧格式）
                                if drone_idx not in self.drone_assignments:
                                    self.drone_assignments[drone_idx] = []
                                elif isinstance(self.drone_assignments[drone_idx], dict):
                                    self.drone_assignments[drone_idx] = [
                                        self.drone_assignments[drone_idx]
                                    ]
                                drone.add_load(task_to_assign.get_weight())
                                self.drone_assignments[drone_idx].append({
                                    'task': task_to_assign,
                                    'assigned_time': self.current_time,
                                    'start_time': self.current_time,
                                    'load_time': None,
                                })
                            else:
                                # 新任务：完整规划路线
                                route = self.plan_route_for_tasks(drone, [task_to_assign])
                                drone.schedule_route(route, task_to_assign.task_id)
                                drone.add_load(task_to_assign.get_weight())
                                self.drone_assignments[drone_idx] = [{
                                    'task': task_to_assign,
                                    'assigned_time': self.current_time,
                                    'start_time': self.current_time,
                                    'load_time': None,
                                }]
                                pending_source = task_to_assign.get_source()

                            # 从未分配任务中移除
                            self.task_generator.unassigned_tasks.remove(task_to_assign)
                            # 更新任务状态
                            task_to_assign.update_status("assigned")

        # 每次调用 update 时，时间加一
        self.current_time += 1

        # 根据模式生成新任务
        new_tasks = self.task_generator.step(self.current_time)
        if new_tasks:
            self.total_generated_tasks += len(new_tasks)
            self.generated_task_times.extend([t.get_generation_time() for t in new_tasks])

        # 记录更新前的电量状态和航点，用于统计耗电量和检测任务完成
        prev_batteries = [drone.current_battery for drone in self.drones]
        prev_scheduled = [list(drone.scheduled_position) for drone in self.drones]

        for drone in self.drones:
            drone.update()

        # 统计耗电量和充电量
        for i, drone in enumerate(self.drones):
            battery_change = prev_batteries[i] - drone.current_battery

            if battery_change > 0:
                # 消耗了电量
                self.total_energy_consumed += battery_change

        # 检测任务完成：
        # 1) 'dest' 航点被弹出 → 对应任务完成（支持一机多任务）
        # 2) is_free 转换兜底（充电自动航线等无 dest 航点的场景）
        for i, drone in enumerate(self.drones):
            prev = prev_scheduled[i]
            curr = drone.scheduled_position

            # 航点弹出检测
            if len(prev) > len(curr) and len(prev) > 0:
                popped = prev[0]
                if len(popped) >= 3 and popped[2] == 'source':
                    source_pos = (popped[0], popped[1])
                    if i in self.drone_assignments:
                        assignments = self.drone_assignments[i]
                        if isinstance(assignments, dict):
                            assignments = [assignments]
                        for assignment in assignments:
                            if (
                                assignment.get('load_time') is None
                                and assignment['task'].get_source() == source_pos
                            ):
                                assignment['load_time'] = self.current_time
                if len(popped) >= 3 and popped[2] == 'dest':
                    dest_pos = (popped[0], popped[1])
                    if i in self.drone_assignments:
                        assignments = self.drone_assignments[i]
                        if isinstance(assignments, list):
                            for assignment in list(assignments):
                                if assignment['task'].get_destination() == dest_pos:
                                    self._record_task_completion(i, assignment)
                                    assignments.remove(assignment)
                                    break
                        elif isinstance(assignments, dict):
                            if assignments['task'].get_destination() == dest_pos:
                                self._record_task_completion(i, assignments)
                                del self.drone_assignments[i]

            # is_free 转换兜底：无人机变为空闲时清理剩余分配记录
            if not self._prev_free_status.get(i, True) and drone.is_free:
                if i in self.drone_assignments:
                    assignments = self.drone_assignments[i]
                    if isinstance(assignments, list):
                        for assignment in list(assignments):
                            self._record_task_completion(i, assignment)
                            assignments.remove(assignment)
                        if not assignments:
                            del self.drone_assignments[i]
                    elif isinstance(assignments, dict):
                        self._record_task_completion(i, assignments)
                        del self.drone_assignments[i]

        # 更新每个无人机之前的空闲状态
        self._prev_free_status = {i: drone.is_free for i, drone in enumerate(self.drones)}

        running = True
        if self.viewer:
            running = self.viewer.render(self.drones)

        done_by_horizon = self.current_time >= self.episode_max_steps
        done_by_exhaustion = (
            self.task_generator.mode == "realistic"
            and self.task_generator.is_exhausted
            and len(self.task_generator.unassigned_tasks) == 0
            and all(drone.is_free for drone in self.drones)
        )
        done = bool(done_by_horizon or done_by_exhaustion or (not running))
        stats = self.get_statistics()
        info = {
            "episode_limit": bool(done_by_horizon),
            "episode_step": int(self.current_time),
            "completion_rate": float(stats["completion_rate"]),
            "total_completed": int(stats.get("total_completed", 0)),
            "total_generated": int(stats.get("total_generated", self.total_generated_tasks)),
            "on_time_rate": float(stats["on_time_rate"]),
            "timeout_rate": float(stats.get("timeout_rate", 0.0)),
            "avg_delay": float(stats["avg_delay"]),
            "avg_generation_to_assignment_wait": float(stats.get("avg_generation_to_assignment_wait", 0.0)),
            "avg_assignment_to_load_wait": float(stats.get("avg_assignment_to_load_wait", 0.0)),
            "avg_wait_time_to_load": float(stats.get("avg_wait_time_to_load", 0.0)),
            "avg_load_to_delivery_time": float(stats.get("avg_load_to_delivery_time", 0.0)),
            "avg_delivery_time": float(stats.get("avg_delivery_time", 0.0)),
            "avg_generation_to_completion_time": float(stats.get("avg_generation_to_completion_time", 0.0)),
            "avg_generation_time": float(stats.get("avg_generation_time", 0.0)),
            "avg_steps_per_order": float(stats.get("avg_steps_per_order", 0.0)),
            "max_delivery_time": float(stats.get("max_delivery_time", 0.0)),
            "max_generation_to_completion_time": float(stats.get("max_generation_to_completion_time", 0.0)),
            "avg_delay_priority_1": float(stats.get("avg_delay_priority_1", 0.0)),
            "avg_delay_priority_2": float(stats.get("avg_delay_priority_2", 0.0)),
            "avg_delay_priority_3": float(stats.get("avg_delay_priority_3", 0.0)),
            "total_energy_consumed": float(stats.get("total_energy_consumed", 0.0)),
        }

        return self._obs(), self._reward(), done, info
    
    def set_seed(self, seed):
        """Set the random seed for the next episode.

        Seed range: 0 <= seed <= 2**32 - 1. None disables deterministic seeding.
        """
        self._episode_seed = apply_seed(seed)
        self.task_generator.set_seed(seed)

    def reset(self, seed=None):
        if seed is not None:
            self.set_seed(seed)
        elif self._episode_seed is not None:
            apply_seed(self._episode_seed)

        self.unassigned_tasks = []
        NUM_DRONES = len(self.drones) if self.drones else DEFAULT_NUM_DRONES
        self.drones = [
            Drone(WAREHOUSE_POS[0], WAREHOUSE_POS[1], drone_id=f"drone_{i}")
            for i in range(NUM_DRONES)
        ]
        self.current_time = 0
        self.completed_tasks = []
        self.drone_assignments = {}
        self._prev_free_status = {i: True for i in range(len(self.drones))}
        self._overdue_task_ids = set()
        self.generated_task_times = []

        self.total_generated_tasks = 0
        self.total_completed_tasks = 0
        self.total_on_time_tasks = 0
        self.total_delay = 0.0
        self.total_generation_to_assignment_wait = 0.0
        self.total_assignment_to_load_wait = 0.0
        self.total_wait_time = 0.0
        self.total_delivery_time = 0.0
        self.total_generation_to_completion_time = 0.0
        self.max_delivery_time = 0.0
        self.max_generation_to_completion_time = 0.0
        self.priority_delay_totals = {1: 0.0, 2: 0.0, 3: 0.0}
        self.priority_delay_counts = {1: 0, 2: 0, 3: 0}

        # 重置耗电量统计
        self.total_energy_consumed = 0.0

        # 重置任务生成器并生成初始任务
        self.task_generator.reset()
        self.task_generator.set_seed(self._episode_seed)
        initial_tasks = self.task_generator.generate_initial_tasks(self.current_time)
        self.unassigned_tasks.extend(self.task_generator.unassigned_tasks)
        self.generated_task_times = [t.get_generation_time() for t in initial_tasks]
        self.total_generated_tasks = self.task_generator.total_tasks_generated

        return self._obs()
    
    def _reward(self):
        """
        计算奖励
        1. 每步负奖励：-0.1
        2. 任务完成奖励：根据优先级给予奖励，如果超时则惩罚
        """
        reward = 0.0
        
        # 每步负奖励
        reward += self.reward_step
        
        # 计算刚刚完成的任务的奖励（送达时给奖励）
        for completed in self.completed_tasks:
            task = completed['task']
            completion_time = completed['completion_time']
            
            # 基础完成奖励
            priority = task.get_priority()
            if priority == 1:
                reward += self.reward_priority_1
            elif priority == 2:
                reward += self.reward_priority_2
            else:
                reward += self.reward_priority_3

        # 对未完成且已超时的任务惩罚：首次大惩罚，持续超时小惩罚
        seen_task_ids = set()
        overdue_penalty = 0.0
        current_time = self.current_time

        for task in self.task_generator.unassigned_tasks:
            if task.task_id in seen_task_ids:
                continue
            seen_task_ids.add(task.task_id)
            deadline = task.get_deadline()
            if deadline is None:
                continue
            overdue = current_time - deadline
            if overdue > 0:
                if task.task_id not in self._overdue_task_ids:
                    overdue_penalty += self.overdue_initial_penalty
                    self._overdue_task_ids.add(task.task_id)
                else:
                    overdue_penalty += self.overdue_step_penalty

        for assignments in self.drone_assignments.values():
            if isinstance(assignments, dict):
                assignments = [assignments]
            for assignment in assignments:
                task = assignment.get('task')
                if task is None or task.task_id in seen_task_ids:
                    continue
                seen_task_ids.add(task.task_id)
                deadline = task.get_deadline()
                if deadline is None:
                    continue
                overdue = current_time - deadline
                if overdue > 0:
                    if task.task_id not in self._overdue_task_ids:
                        overdue_penalty += self.overdue_initial_penalty
                        self._overdue_task_ids.add(task.task_id)
                    else:
                        overdue_penalty += self.overdue_step_penalty

        reward += overdue_penalty
        
        # 清空已处理的完成任务
        self.completed_tasks.clear()
        
        return reward
    
    def _obs(self):
        """
        返回观察空间：
        1. list：所有无人机的：当前位置
        2. list：未分配订单的： 取位置 送位置 剩余时间：ttlj=ddlj−now 优先级
        3. list《list》：掩码：所有无人机的is_free
        """
        # 1. 所有无人机的当前位置
        drone_positions = []
        for drone in self.drones:
            pos = drone.get_position()
            drone_positions.append([pos[0], pos[1]])
        
        # 2. 未分配订单的信息
        unassigned_tasks_info = []
        current_time = self.current_time
        for task in self.task_generator.unassigned_tasks:
            source = task.get_source()
            destination = task.get_destination()
            deadline = task.get_deadline()
            priority = task.get_priority()
            task_id = task.task_id
            weight = task.get_weight()
            route_distance = math.dist(source, destination)
            source_to_warehouse = math.dist(source, WAREHOUSE_POS)
            
            # 计算剩余时间：ttlj = ddlj - now
            if deadline is not None:
                ttlj = deadline - current_time
            else:
                ttlj = float('inf')  # 如果没有截止时间，设为无穷大
            
            unassigned_tasks_info.append({
                'task_id': task_id,
                'source': [source[0], source[1]],
                'destination': [destination[0], destination[1]],
                'remaining_time': ttlj,
                'priority': priority,
                'weight': weight,
                'route_distance': route_distance,
                'source_to_warehouse': source_to_warehouse
            })

        # 保持任务的生成顺序（FIFO），不做额外排序，以便贪心策略按先来后到选择
        
        # 3. 所有无人机的is_free掩码
        # 当 allow_multi_task 启用时：不空闲但尚未抵达取货点的 drone 仍可接同源任务
        drone_free_masks = []
        for drone in self.drones:
            if drone.is_free:
                free_mask = [True for _ in self.task_generator.unassigned_tasks]
            elif ALLOW_MULTI_TASK:
                pending_source = None
                for wp in drone.scheduled_position:
                    if len(wp) >= 3 and wp[2] == 'source':
                        pending_source = (wp[0], wp[1])
                        break
                if pending_source is not None:
                    free_mask = [
                        task.get_source() == pending_source
                        for task in self.task_generator.unassigned_tasks
                    ]
                else:
                    free_mask = [False for _ in self.task_generator.unassigned_tasks]
            else:
                free_mask = [False for _ in self.task_generator.unassigned_tasks]
            drone_free_masks.append(free_mask)

        # 固定观察长度：截断或填充到 MAX_OBS_TASKS
        current_len = len(unassigned_tasks_info)
        if current_len > MAX_OBS_TASKS:
            unassigned_tasks_info = unassigned_tasks_info[:MAX_OBS_TASKS]
            for i in range(len(drone_free_masks)):
                drone_free_masks[i] = drone_free_masks[i][:MAX_OBS_TASKS]
        elif current_len < MAX_OBS_TASKS:
            pad_count = MAX_OBS_TASKS - current_len
            pad_tasks = [
                {
                    'task_id': f'__pad_{k}',
                    'source': [0.0, 0.0],
                    'destination': [0.0, 0.0],
                    'remaining_time': -1.0,
                    'priority': 0,
                    'weight': 0.0,
                }
                for k in range(pad_count)
            ]
            unassigned_tasks_info.extend(pad_tasks)
            for i in range(len(drone_free_masks)):
                drone_free_masks[i].extend([False] * pad_count)

        # 3b. 扁平的 is_free 状态（与 unassigned_tasks 是否为空解耦，事件驱动调度依赖此字段）
        drone_is_free = [drone.is_free for drone in self.drones]

        # 4. 电量信息（PSO 等预测式调度器需要据此评估能耗与充电耗时）
        drone_batteries = [
            {
                'current': drone.current_battery,
                'capacity': drone.battery_capacity,
                'is_charging': drone.is_charging,
            }
            for drone in self.drones
        ]

        # 4b. 载重信息
        drone_loads = [
            {
                'current': getattr(drone, 'current_load', 0.0),
                'capacity': drone.carrying_capacity,
            }
            for drone in self.drones
        ]

        # 5. 充电站信息（位置 + 充电功率），与 frontend/drone.py 实际使用的站点一致
        charging_stations_info = [
            {
                'station_id': s.station_id,
                'position': [s.x, s.y],
                'charging_power': s.charging_power,
            }
            for s in DEFAULT_CHARGING_STATIONS
        ]
        # 向后兼容：单站字段
        charging_station_info = charging_stations_info[0] if charging_stations_info else {
            'position': [0.0, 0.0],
            'charging_power': 50.0,
        }

        return {
            'drone_positions': drone_positions,
            'unassigned_tasks': unassigned_tasks_info,
            'drone_free_masks': drone_free_masks,
            'drone_is_free': drone_is_free,
            'drone_batteries': drone_batteries,
            'drone_loads': drone_loads,
            'charging_station': charging_station_info,   # 向后兼容
            'charging_stations': charging_stations_info,  # 新：多站列表
        }
    
    def plan_route_for_tasks(self, drone, tasks):
        """
        为无人机规划多个任务的路线，确保绕过建筑物。
        返回完整的绕行路线，每个点格式为 (x, y, type)。
        type: 'source' = 任务起点, 'dest' = 任务终点, 'waypoint' = 绕飞航点
        """
        start_time = time.perf_counter()
        if not tasks:
            return []
        
        full_route = []  # 格式: [(x, y, type), ...]
        current_pos = drone.get_position()
        
        for task in tasks:
            # 规划从当前位置到任务起始点的路径
            source = task.get_source()
            dest = task.get_destination()
            
            # 第一步：当前位置 → 起始点（绕行）
            source_route = self.plan_route_around_buildings(current_pos, source)
            for point in source_route:
                # 最后一个点是 source，其他是 waypoint
                if len(source_route) > 1:
                    if point == source_route[-1]:
                        full_route.append((point[0], point[1], 'source'))
                    else:
                        full_route.append((point[0], point[1], 'waypoint'))
                else:
                    full_route.append((point[0], point[1], 'source'))
            
            # 第二步：起始点 → 终点（绕行）
            dest_route = self.plan_route_around_buildings(source, dest)
            for point in dest_route:
                if len(dest_route) > 1:
                    if point == dest_route[-1]:
                        full_route.append((point[0], point[1], 'dest'))
                    else:
                        full_route.append((point[0], point[1], 'waypoint'))
                else:
                    full_route.append((point[0], point[1], 'dest'))
            
            current_pos = dest
        
        if PRINT_ROUTE_DEBUG:
            print(f"Full planned route for drone {drone.drone_id}: {full_route}")
            elapsed = time.perf_counter() - start_time
            print(f"assign_task elapsed: {elapsed:.6f} seconds")
        
        # 验证整个路线是否与建筑物相交
        self.verify_route_with_types(full_route)
        
        return full_route
    
    def verify_route_with_types(self, route):
        """验证路线是否与建筑物相交"""
        for i in range(len(route) - 1):
            if not self.is_path_clear(route[i][:2], route[i+1][:2]):
                print(f"WARNING: Route segment {route[i][:2]} -> {route[i+1][:2]} intersects with building!")
                return False
        return True

    def plan_route_around_buildings(self, start_pos, end_pos):
        """
        Plans a route from start_pos to end_pos avoiding buildings using A* algorithm.
        """
        # Check if direct path is possible (stricter check)
        if self.is_path_clear(start_pos, end_pos):
            return [end_pos]
        
        # Try A* pathfinding
        path = self.a_star_pathfinding(start_pos, end_pos)
        
        if path and len(path) > 1:
            # Return path excluding starting position
            return path[1:]
        
        # Last resort: return direct destination with warning
        print(f"Warning: Could not find obstacle-free path from {start_pos} to {end_pos}")
        return [end_pos]

    def a_star_pathfinding(self, start, goal):
        """
        Implements A* pathfinding algorithm considering building obstacles.
        Uses visibility graph approach.
        """
        # Round positions to avoid floating point precision issues
        def round_pos(pos, decimals=2):
            return (round(pos[0], decimals), round(pos[1], decimals))
        
        start = round_pos(start)
        goal = round_pos(goal)
        
        # Get all relevant points (start, goal, and building corner points)
        all_points = [start, goal]
        
        # Add building corner points to our visibility graph
        for building in self.high_buildings:
            geom = building['geometry']
            if hasattr(geom, 'exterior'):  # Polygon
                for coord in list(geom.exterior.coords)[:-1]:
                    rounded_coord = round_pos(coord)
                    if rounded_coord != start and rounded_coord != goal:
                        all_points.append(rounded_coord)
            elif hasattr(geom, 'coords'):  # Point or LineString
                for coord in geom.coords:
                    rounded_coord = round_pos(coord)
                    if rounded_coord != start and rounded_coord != goal:
                        all_points.append(rounded_coord)
        
        # Remove duplicates
        all_points = list(set(all_points))
        
        # Create dictionaries with rounded keys for consistent lookup
        point_to_key = {p: p for p in all_points}
        
        # Implement A* algorithm
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        open_set_set = {start}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            open_set_set.remove(current)
            
            if self.heuristic(current, goal) < 1:  # Close enough to goal
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            for neighbor in all_points:
                if neighbor == current:
                    continue
                    
                # Check if path from current to neighbor intersects any buildings
                if self.is_path_clear(current, neighbor):
                    tentative_g = g_score.get(current, float('inf')) + self.heuristic(current, neighbor)
                    
                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                        
                        if neighbor not in open_set_set:
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))
                            open_set_set.add(neighbor)
        
        # No path found
        return None

    def heuristic(self, pos1, pos2):
        """
        Calculates Euclidean distance between two points.
        """
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def is_path_clear(self, pos1, pos2):
        """
        Checks if the path between two positions intersects any buildings.
        """
        line = LineString([pos1, pos2])
        
        for building in self.high_buildings:
            if building['geometry'].intersects(line):
                # Check if intersection is significant (not just touching corners)
                if building['geometry'].intersection(line).length > 0.1:  # Small threshold to avoid floating point errors
                    return False
        
        return True
