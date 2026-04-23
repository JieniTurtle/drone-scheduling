import random

from config.settings import get_shared_config


_TASK_CFG = get_shared_config().get("task", {})
WAREHOUSE_ROUTE_ID = _TASK_CFG.get("warehouse_route_id", "1364970737#0")
CHARGER_ROUTE_ID = _TASK_CFG.get("charger_route_id", "125465016")
_warehouse_pos = _TASK_CFG.get("warehouse_pos", [357600.574872369, 3462308.772003661])
WAREHOUSE_POS = (float(_warehouse_pos[0]), float(_warehouse_pos[1]))

class Task:
    def __init__(self, task_id, weight, source, destination, deadline=None, priority=1, generation_time=None):
        """
        初始化任务对象
        
        :param task_id: 任务唯一标识符
        :param weight: 物体的重量 (单位: kg)
        :param source: 起始地点
        :param destination: 目标地点
        :param deadline: 截止时间 (可选)
        :param priority: 优先级 (默认为1，数值越大优先级越高)
        :param generation_time: 任务生成的模拟时刻 (step)
        """
        self.task_id = task_id
        self.weight = weight
        self.source = source
        self.destination = destination
        self.deadline = deadline
        self.priority = priority
        self.status = "pending"  # 任务状态: pending, assigned, in_progress, completed, failed
        self.generation_time = generation_time  # 任务生成时刻 (step)
    
    def get_weight(self):
        """获取物体重量"""
        return self.weight
    
    def get_source(self):
        """获取起始地点"""
        return self.source
    
    def get_destination(self):
        """获取目标地点"""
        return self.destination
    
    def get_deadline(self):
        """获取截止时间"""
        return self.deadline
    
    def get_priority(self):
        """获取优先级"""
        return self.priority
    
    def get_route(self):
        """获取任务的完整路线：起始点 -> 终点"""
        return [self.source, self.destination]
    
    def update_status(self, status):
        """更新任务状态"""
        valid_statuses = ["pending", "assigned", "in_progress", "completed", "failed"]
        if status in valid_statuses:
            self.status = status
        else:
            raise ValueError(f"Invalid status: {status}. Valid statuses are: {valid_statuses}")
    
    def get_generation_time(self):
        """获取任务生成时刻"""
        return self.generation_time
    
    def __str__(self):
        """返回任务的字符串表示"""
        return f"Task(ID: {self.task_id}, Weight: {self.weight}kg, Source: {self.source}, Destination: {self.destination}, Deadline: {self.deadline}, GenerationTime: {self.generation_time}, Priority: {self.priority}, Status: {self.status})"
    
    def __repr__(self):
        """返回任务的详细表示"""
        return self.__str__()


def load_destinations_from_file(file_path):
    """从文件读取配送目的地坐标列表"""
    destinations = []
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                parts = line.split(',')
                if len(parts) != 2:
                    continue
                x, y = float(parts[0]), float(parts[1])
                destinations.append((x, y))
    except FileNotFoundError:
        print(f"Warning: {file_path} not found. No destinations loaded.")
    return destinations


class TaskGenerator:
    """任务生成器，统一封装任务生成逻辑。"""

    def __init__(self, possible_destinations=None, file_path=None):
        cfg = get_shared_config().get("task", {})
        if file_path is None:
            file_path = cfg.get("clicked_positions_file", "clicked_positions.txt")

        if possible_destinations is None:
            possible_destinations = load_destinations_from_file(file_path)

        self.destinations = possible_destinations
        self.task_counter = 0
        self.weight_min = int(cfg.get("weight_min", 1))
        self.weight_max = int(cfg.get("weight_max", 4))
        self.priority_min = int(cfg.get("priority_min", 1))
        self.priority_max = int(cfg.get("priority_max", 5))
        self.deadline_offset_min = int(cfg.get("deadline_offset_min", 200))
        self.deadline_offset_max = int(cfg.get("deadline_offset_max", 500))

    def generate_random_tasks(self, num_tasks=5, current_time=0):
        """随机生成任务，每个任务包含起点、终点、截止时间与优先级。"""
        if len(self.destinations) < 2:
            print("Warning: Need at least 2 destinations for task generation.")
            return []

        current_time = 0 if current_time is None else current_time
        tasks = []
        for _ in range(max(0, int(num_tasks))):
            self.task_counter += 1
            weight = random.randint(self.weight_min, self.weight_max)
            source, destination = random.sample(self.destinations, 2)

            deadline_offset = random.randint(self.deadline_offset_min, self.deadline_offset_max)
            deadline = current_time + deadline_offset
            priority = random.randint(self.priority_min, self.priority_max)

            tasks.append(
                Task(
                    task_id=f"task_{self.task_counter}",
                    weight=weight,
                    source=source,
                    destination=destination,
                    deadline=deadline,
                    priority=priority,
                    generation_time=current_time,
                )
            )
        return tasks