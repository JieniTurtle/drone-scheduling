import random
import math
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

    def generate_random_tasks(self, num_tasks=5):
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
            task = Task(task_id=f"task_{self.task_counter}", weight=weight, source=source, destination=destination)
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
    def schedule_for_drone(drone, unassigned_tasks):
        """
        贪心调度算法：
        1. 寻找离当前位置最近的任务起始点
        2. 继续寻找满足无人机剩余载重的其他任务
        无人机可携带多个任务（直到载重满）
        返回分配的任务列表
        """
        if not unassigned_tasks:
            return []

        assigned_tasks = []
        remaining_capacity = drone.get_remaining_capacity()
        current_pos = drone.get_position()

        while remaining_capacity > 0:
            # 筛选满足载重约束的任务
            valid_tasks = [t for t in unassigned_tasks if t.weight <= remaining_capacity]
            if not valid_tasks:
                break

            # 选择离当前位置最近的任务（比较到起始点的距离）
            nearest_task = min(valid_tasks, key=lambda t: GreedyScheduler.euclidean_distance(current_pos, t.get_source()))
            
            assigned_tasks.append(nearest_task)
            unassigned_tasks.remove(nearest_task)
            remaining_capacity -= nearest_task.get_weight()
            # 任务完成后，无人机会停在终点位置
            current_pos = nearest_task.get_destination()

        # 不再添加返回仓库的任务，无人机完成任务后停在原地
        return assigned_tasks

    @staticmethod
    def schedule_all_drones(drones, tasks):
        """
        为所有无人机调度任务
        返回: {drone: [assigned_tasks]} 字典
        """
        assignments = {drone: [] for drone in drones}
        unassigned_tasks = list(tasks)  # 复制列表，避免修改原列表
        
        for drone in drones:
            drone_tasks = GreedyScheduler.schedule_for_drone(drone, unassigned_tasks)
            assignments[drone] = drone_tasks
            
            # 为无人机设置路线
            if drone_tasks:
                route = [drone.get_position()] + [t.get_destination() for t in drone_tasks]
                drone.schedule_route(route)
        
        return assignments