import random
import math
import datetime
from task import Task


def load_destinations_from_file(file_path):
    """从文件读取配送目的地坐标列表"""
    destinations = []
    try:
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 2:
                        x, y = float(parts[0]), float(parts[1])
                        destinations.append((x, y))
    except FileNotFoundError:
        print(f"Warning: {file_path} not found. No destinations loaded.")
    return destinations


class TaskGenerator:
    def __init__(self, possible_destinations=None, file_path='clicked_positions.txt'):
        if possible_destinations is None:
            possible_destinations = load_destinations_from_file(file_path)
        self.destinations = possible_destinations
        self.task_counter = 0

    def generate_random_tasks(self, num_tasks=5, current_time=None):
        """随机生成指定数量的任务，每个任务包含起始点和终点"""
        if len(self.destinations) < 2:
            print("Warning: Need at least 2 destinations for task generation.")
            return []
        
        tasks = []
        for _ in range(num_tasks):
            self.task_counter += 1
            weight = random.randint(1, 4)
            # 从候选点中随机选择两个不同的点作为起始点和终点
            source, destination = random.sample(self.destinations, 2)
            
            # 生成随机截止时间（当前时间后1-24小时）
            deadline_offset = random.randint(3600, 86400)  # 1-24小时
            deadline = current_time + deadline_offset
            
            # 生成随机优先级（1-5）
            priority = random.randint(1, 5)
            
            task = Task(task_id=f"task_{self.task_counter}", weight=weight, source=source, destination=destination, deadline=deadline, priority=priority)
            tasks.append(task)
        return tasks


class GreedyScheduler:
    @staticmethod
    def euclidean_distance(pos1, pos2):
        """计算两点之间的欧氏距离"""
        if isinstance(pos1, tuple) and isinstance(pos2, tuple):
            return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
        return float('inf')

    @staticmethod
    def schedule_for_drone(observation):
        """
        贪心调度算法：为所有空闲无人机分配任务
        1. 从观察空间中获取无人机位置、空闲状态和未分配任务信息
        2. 为每个空闲无人机分配一个离当前位置最近的任务
        返回: {drone_index: [assigned_task_id]} 字典（每个无人机最多分配一个任务）
        """
        drone_positions = observation['drone_positions']
        unassigned_tasks_info = observation['unassigned_tasks']
        drone_free_masks = observation['drone_free_masks']

        if not unassigned_tasks_info:
            return {}

        assignments = {}

        # 为每个空闲无人机分配任务
        for drone_idx, (drone_pos, free_mask) in enumerate(zip(drone_positions, drone_free_masks)):
            # 检查无人机是否空闲（所有任务的掩码都为True表示空闲）
            if not all(free_mask):
                continue

            # 如果没有任务可分配，跳过
            if not unassigned_tasks_info:
                break

            current_pos = tuple(drone_pos)

            # 选择离当前位置最近的任务（不再检查载重）
            nearest_task = min(unassigned_tasks_info, key=lambda t: GreedyScheduler.euclidean_distance(current_pos, tuple(t['source'])))

            # 分配任务
            assignments[drone_idx] = [nearest_task['task_id']]
            # 从输入列表中移除已分配的任务
            unassigned_tasks_info.remove(nearest_task)

        return assignments

    @staticmethod
    def schedule_all_drones(drones, tasks):
        """
        为所有无人机调度任务
        返回: {drone: [assigned_tasks]} 字典
        """
        assignments = {drone: [] for drone in drones}
        unassigned_tasks = list(tasks)  # 复制列表，避免修改原列表
        
        # 创建观察空间格式的任务信息
        unassigned_tasks_info = []
        for task in unassigned_tasks:
            unassigned_tasks_info.append({
                'task_id': task.task_id,
                'source': list(task.get_source()),
                'destination': list(task.get_destination()),
                'remaining_time': float('inf'),  # 简化处理，不使用剩余时间
                'priority': task.get_priority(),
                'weight': task.get_weight()
            })
        
        # 创建观察空间字典
        observation = {
            'unassigned_tasks': unassigned_tasks_info
        }
        
        for drone in drones:
            # 获取分配的任务ID
            assigned_task_ids = GreedyScheduler.schedule_for_drone(drone, observation)
            
            # 从原始任务列表中找到对应的Task对象
            assigned_tasks = []
            for task_id in assigned_task_ids:
                for task in unassigned_tasks[:]:  # 使用切片复制避免修改时的问题
                    if task.task_id == task_id:
                        assigned_tasks.append(task)
                        unassigned_tasks.remove(task)
                        break
            
            assignments[drone] = assigned_tasks
            
            # 为无人机设置路线
            if assigned_tasks:
                route = [drone.get_position()] + [t.get_destination() for t in assigned_tasks]
                drone.schedule_route(route)
        
        return assignments