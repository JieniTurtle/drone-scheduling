import osmnx as ox
import numpy as np
import heapq
import time
import datetime
from shapely.geometry import Point, LineString
import math
from drone import Drone
from task import Task
# from test import OptimizedMapViewer
from tools.osm import load_map_data, get_building_location_by_name, get_global_bounds
from scheduler import TaskGenerator
from map_drawer import OptimizedMapViewer
from task import WAREHOUSE_POS


MAX_UNASSIGNED_TASKS = 20  # 任务池中最大未分配任务数

class Environment:
    def __init__(self, osm_file_path, visualize=False):
        _, buildings_with_height = load_map_data(osm_file_path)

        self.global_bounds = get_global_bounds(buildings_with_height)

        self.high_buildings = [b for b in buildings_with_height if b['height'] is not None and b['height'] > 20]

        self.task_generator = TaskGenerator(file_path='clicked_positions.txt')
        self.unassigned_tasks = []
        self.drones = []  # 初始化无人机列表
        # 创建多架无人机（从仓库位置出发）
        NUM_DRONES = 3  # 可根据需要调整无人机数量
        self.drones = [
            Drone(WAREHOUSE_POS[0], WAREHOUSE_POS[1], drone_id=f"drone_{i}")
            for i in range(NUM_DRONES)
        ]
        
        # 初始化模拟时间
        self.current_time = 0
        
        # 跟踪已完成的任务，用于奖励计算
        self.completed_tasks = []
        
        # 跟踪无人机分配的任务：{drone_idx: task}
        self.drone_assignments = {}
        
         # 初始生成一批任务
        new_tasks = self.task_generator.generate_random_tasks(num_tasks=8, current_time=self.current_time)
        self.unassigned_tasks.extend(new_tasks)
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
                            # 记录任务分配
                            self.drone_assignments[drone_idx] = task_to_assign
                            
                            # 规划路线
                            route = self.plan_route_for_tasks(drone, [task_to_assign])
                            # 分配路线给无人机
                            drone.schedule_route(route)
                            # 从未分配任务中移除
                            self.unassigned_tasks.remove(task_to_assign)
                            # 更新任务状态
                            task_to_assign.update_status("assigned")

        # 记录更新前的无人机状态，用于检测任务完成
        prev_free_status = [drone.is_free for drone in self.drones]

        # 每次调用 update 时，时间加一
        self.current_time += 1

        # 持续生成新任务，保持任务池有足够任务
        if len(self.unassigned_tasks) < MAX_UNASSIGNED_TASKS:
            new_tasks = self.task_generator.generate_random_tasks(num_tasks=5, current_time=self.current_time)
            self.unassigned_tasks.extend(new_tasks)

        for drone in self.drones:
            drone.update()

        # 检测任务完成：从忙碌变为闲置的无人机
        for i, drone in enumerate(self.drones):
            if not prev_free_status[i] and drone.is_free:
                # 无人机刚刚完成任务
                if i in self.drone_assignments:
                    completed_task = self.drone_assignments[i]
                    self.completed_tasks.append({
                        'task': completed_task,
                        'completion_time': self.current_time
                    })
                    # 移除分配记录
                    del self.drone_assignments[i]

        running = True
        if self.viewer:
            running = self.viewer.render(self.drones)

        return self._obs(), self._reward(), running, None
    
    def reset(self):
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
            free_mask = [drone.is_free for task in self.unassigned_tasks]  # 每个任务对应一个掩码值
            drone_free_masks.append(free_mask)
        
        return {
            'drone_positions': drone_positions,
            'unassigned_tasks': unassigned_tasks_info,
            'drone_free_masks': drone_free_masks
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
    
        """调整航点，确保它在所有建筑物外部"""
        point = Point(waypoint)
        min_distance = 10  # 最小安全距离
        
        for building in self.high_buildings:
            geom = building['geometry']
            dist = geom.exterior.distance(point)
            if dist < min_distance:
                # 需要将航点移出建筑物
                # 简单地增加偏移量
                bounds = geom.bounds
                center_x = (bounds[0] + bounds[2]) / 2
                center_y = (bounds[1] + bounds[3]) / 2
                
                # 向远离建筑物的方向移动
                dx = waypoint[0] - center_x
                dy = waypoint[1] - center_y
                dist = (dx*dx + dy*dy)**0.5
                if dist > 0:
                    waypoint = (
                        waypoint[0] + (dx / dist) * (min_distance - dist + 10),
                        waypoint[1] + (dy / dist) * (min_distance - dist + 10)
                    )
        
        return waypoint

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
