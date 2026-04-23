import osmnx as ox
import numpy as np
import heapq
import time
import datetime
from shapely.geometry import Point, LineString
import math
from drone import Drone
from config.settings import get_shared_config
# from test import OptimizedMapViewer
from tools.osm import load_map_data, get_building_location_by_name, get_global_bounds
from task import TaskGenerator
from map_drawer import OptimizedMapViewer
from task import WAREHOUSE_POS


CFG = get_shared_config()
ENV_CFG = CFG.get("environment", {})
MAX_UNASSIGNED_TASKS = int(ENV_CFG.get("max_unassigned_tasks", 5))
DEFAULT_EPISODE_MAX_STEPS = int(ENV_CFG.get("episode_max_steps", 1200))
DEFAULT_NUM_DRONES = int(ENV_CFG.get("num_drones", 3))
DEFAULT_INITIAL_TASK_COUNT = int(ENV_CFG.get("initial_task_count", 8))

class Environment:
    def __init__(self, osm_file_path, visualize=False, episode_max_steps=DEFAULT_EPISODE_MAX_STEPS):
        _, buildings_with_height = load_map_data(osm_file_path)

        self.global_bounds = get_global_bounds(buildings_with_height)

        self.high_buildings = [b for b in buildings_with_height if b['height'] is not None and b['height'] > 20]

        task_cfg = CFG.get("task", {})
        task_file = task_cfg.get("clicked_positions_file", 'clicked_positions.txt')
        self.task_generator = TaskGenerator(file_path=task_file)
        self.unassigned_tasks = []
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

        # 统计指标（按回合统计）
        self.total_generated_tasks = 0
        self.total_completed_tasks = 0
        self.total_on_time_tasks = 0
        self.total_delay = 0.0
        self.total_delivery_time = 0.0
        
         # 初始生成一批任务
        new_tasks = self.task_generator.generate_random_tasks(num_tasks=DEFAULT_INITIAL_TASK_COUNT, current_time=self.current_time)
        self.unassigned_tasks.extend(new_tasks)
        self.total_generated_tasks += len(new_tasks)
        print(f"Generated {len(new_tasks)} tasks:")
        for task in new_tasks:
            print(f"  {task}")

        self.viewer = None
        if visualize:
            self.viewer = OptimizedMapViewer('data/map/part_of_yangpu.osm')

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
        print(f"  优先级: {priority}/5  |  货物重量: {weight}kg")
        print(f"  配送时长: {delivery_time:.1f} 时间单位")
        print(f"  预期时长: {expected_time:.1f} 时间单位")
        if not is_on_time:
            print(f"  延迟时间: {delay:.1f} 时间单位")
        print(f"  状态: {status}")
        print(f"{'='*70}")
    
    def get_statistics(self):
        """获取当前统计指标"""
        completion_rate = 0.0
        if self.total_generated_tasks > 0:
            completion_rate = self.total_completed_tasks / self.total_generated_tasks

        if self.total_completed_tasks == 0:
            return {
                'completion_rate': completion_rate,
                'total_completed': 0,
                'on_time_rate': 0.0,
                'avg_delay': 0.0,
                'avg_delivery_time': 0.0
            }
        
        return {
            'completion_rate': completion_rate,
            'total_completed': self.total_completed_tasks,
            'on_time_rate': self.total_on_time_tasks / self.total_completed_tasks,
            'avg_delay': self.total_delay / self.total_completed_tasks,
            'avg_delivery_time': self.total_delivery_time / self.total_completed_tasks
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
        print(f"  平均配送时长: {stats['avg_delivery_time']:.2f} 时间单位")
        print(f"{'='*70}\n")

    def step(self, actions):
        # actions 格式: {drone_index: [task_id_list]}
        for drone_idx, task_ids in actions.items():
            if drone_idx < len(self.drones):
                drone = self.drones[drone_idx]
                for task_id in task_ids:
                    if task_id is not None:
                        # 找到对应的任务
                        task_to_assign = None
                        for task in self.unassigned_tasks:
                            if task.task_id == task_id:
                                task_to_assign = task
                                break
                        
                        if task_to_assign is not None:
                            # 记录任务分配（包含开始时间）
                            self.drone_assignments[drone_idx] = {
                                'task': task_to_assign,
                                'start_time': self.current_time
                            }
                            
                            # 规划路线
                            route = self.plan_route_for_tasks(drone, [task_to_assign])
                            # 分配路线给无人机
                            drone.schedule_route(route, task_to_assign.task_id)
                            # 从未分配任务中移除
                            self.unassigned_tasks.remove(task_to_assign)
                            # 更新任务状态
                            task_to_assign.update_status("assigned")

        # 每次调用 update 时，时间加一
        self.current_time += 1

        # 持续生成新任务，保持任务池有足够任务
        if len(self.unassigned_tasks) < MAX_UNASSIGNED_TASKS:
            need_count = MAX_UNASSIGNED_TASKS - len(self.unassigned_tasks)
            new_tasks = self.task_generator.generate_random_tasks(num_tasks=need_count, current_time=self.current_time)
            self.unassigned_tasks.extend(new_tasks)
            self.total_generated_tasks += len(new_tasks)

        for drone in self.drones:
            drone.update()

        # 检测任务完成：通过 executing_task_id 判断（刚从执行中变为None表示任务完成）
        # 遍历所有无人机，检查是否有刚完成的任务
        for i, drone in enumerate(self.drones):
            if not self._prev_free_status.get(i, True) and drone.is_free:
                # 无人机刚刚完成任务
                if i in self.drone_assignments:
                    assignment_info = self.drone_assignments[i]
                    completed_task = assignment_info['task']
                    start_time = assignment_info['start_time']
                    completion_time = self.current_time
                    delivery_time = completion_time - start_time
                    
                    # 计算延迟
                    deadline = completed_task.get_deadline()
                    if deadline is not None:
                        delay = max(0, completion_time - deadline)
                    else:
                        delay = 0

                    is_on_time = delay <= 0
                    self.total_completed_tasks += 1
                    self.total_delay += float(delay)
                    self.total_delivery_time += float(delivery_time)
                    if is_on_time:
                        self.total_on_time_tasks += 1
                    
                    self.completed_tasks.append({
                        'task': completed_task,
                        'completion_time': completion_time,
                        'delivery_time': delivery_time,
                        'delay': delay
                    })
                    # 移除分配记录
                    del self.drone_assignments[i]

        # 更新每个无人机之前的空闲状态
        self._prev_free_status = {i: drone.is_free for i, drone in enumerate(self.drones)}

        running = True
        if self.viewer:
            running = self.viewer.render(self.drones)

        done_by_horizon = self.current_time >= self.episode_max_steps
        done = bool(done_by_horizon or (not running))
        stats = self.get_statistics()
        info = {
            "episode_limit": bool(done_by_horizon),
            "completion_rate": float(stats["completion_rate"]),
            "on_time_rate": float(stats["on_time_rate"]),
            "avg_delay": float(stats["avg_delay"]),
        }

        return self._obs(), self._reward(), done, info
    
    def reset(self):
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

        self.total_generated_tasks = 0
        self.total_completed_tasks = 0
        self.total_on_time_tasks = 0
        self.total_delay = 0.0
        self.total_delivery_time = 0.0

        new_tasks = self.task_generator.generate_random_tasks(num_tasks=DEFAULT_INITIAL_TASK_COUNT, current_time=self.current_time)
        self.unassigned_tasks.extend(new_tasks)
        self.total_generated_tasks += len(new_tasks)

        return self._obs()
    
    def _reward(self):
        """
        计算奖励
        1. 每步负奖励：-0.1
        2. 任务完成奖励：根据优先级给予奖励，如果超时则惩罚
        """
        reward = 0.0
        
        # 每步负奖励
        reward -= 0.1
        
        # 计算刚刚完成的任务的奖励
        for completed in self.completed_tasks:
            task = completed['task']
            completion_time = completed['completion_time']
            
            # 基础完成奖励
            priority = task.get_priority()
            if priority == 1:
                reward += 100
            elif priority == 2:
                reward += 80
            else:
                reward += 50  # 默认奖励
            
            # 检查是否超时
            deadline = task.get_deadline()
            if deadline is not None and completion_time > deadline:
                overtime = completion_time - deadline
                reward -= 10 * overtime
        
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
        for task in self.unassigned_tasks:
            source = task.get_source()
            destination = task.get_destination()
            deadline = task.get_deadline()
            priority = task.get_priority()
            task_id = task.task_id
            weight = task.get_weight()
            
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
                'weight': weight
            })
        
        # 3. 所有无人机的is_free掩码
        drone_free_masks = []
        for drone in self.drones:
            free_mask = [drone.is_free for task in self.unassigned_tasks]
            drone_free_masks.append(free_mask)

        # 3b. 扁平的 is_free 状态（与 unassigned_tasks 是否为空解耦，事件驱动调度依赖此字段）
        drone_is_free = [drone.is_free for drone in self.drones]

        return {
            'drone_positions': drone_positions,
            'unassigned_tasks': unassigned_tasks_info,
            'drone_free_masks': drone_free_masks,
            'drone_is_free': drone_is_free,
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
