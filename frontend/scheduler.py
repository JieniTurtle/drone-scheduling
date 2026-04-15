import random
import traci
from task import Task, WAREHOUSE_ROUTE_ID

class TaskGenerator:
    def __init__(self, possible_destinations):
        self.destinations = possible_destinations
        self.task_counter = 0

    def generate_random_tasks(self, num_tasks=5):
        """随机生成指定数量的任务"""
        tasks = []
        for _ in range(num_tasks):
            self.task_counter += 1
            # 随机重量 1~4 kg (无人机最大载重为5)
            weight = random.randint(1, 4) 
            destination = random.choice(self.destinations)
            task = Task(task_id=f"task_{self.task_counter}", weight=weight, destination=destination)
            tasks.append(task)
        return tasks


class GreedyScheduler:
    @staticmethod
    def get_route_distance(start_edge, end_edge):
        """利用 SUMO 计算两个边之间的实际道路行驶距离"""
        if start_edge == end_edge:
            return 0.0
        try:
            route = traci.simulation.findRoute(start_edge, end_edge, vType="bicycle_type")
            if len(route.edges) > 0:
                return route.length
        except Exception:
            pass
        return float('inf')  # 无法到达时返回无限大

    @staticmethod
    def schedule_for_drone(drone, unassigned_tasks):
        """
        贪心调度算法：
        1. 寻找离仓库最近的任务1
        2. 寻找离任务1最近，且满足无人机剩余载重的任务2
        最多带2个任务之后返回仓库
        """
        if not unassigned_tasks:
            return []

        assigned_tasks = []
        current_capacity = drone.carrying_capacity
        current_edge = WAREHOUSE_ROUTE_ID  # 默认无人机从仓库出发

        # 第一步：筛选无人机能拿得动的所有任务
        valid_tasks = [t for t in unassigned_tasks if t.weight <= current_capacity]
        if not valid_tasks:
            return []

        # 寻找离仓库最近的 任务1
        task1 = min(valid_tasks, key=lambda t: GreedyScheduler.get_route_distance(current_edge, t.get_destination()))
        
        assigned_tasks.append(task1)
        unassigned_tasks.remove(task1)
        current_capacity -= task1.get_weight()
        current_edge = task1.get_destination()

        # 第二步：寻找离 任务1 最近且满足剩余载重的 任务2
        valid_tasks_for_two = [t for t in unassigned_tasks if t.weight <= current_capacity]
        if valid_tasks_for_two:
            task2 = min(valid_tasks_for_two, key=lambda t: GreedyScheduler.get_route_distance(current_edge, t.get_destination()))
            assigned_tasks.append(task2)
            unassigned_tasks.remove(task2)

        # 无论分配了几个任务，最后都加一个返回仓库的任务，以便下次重新装货
        return_task = Task(task_id=f"return_{drone.drone_id[:5]}", weight=0, destination=WAREHOUSE_ROUTE_ID)
        assigned_tasks.append(return_task)

        return assigned_tasks