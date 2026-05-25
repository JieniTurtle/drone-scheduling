import math
import random

from config.settings import get_shared_config


_TASK_CFG = get_shared_config().get("task", {})
WAREHOUSE_ROUTE_ID = _TASK_CFG.get("warehouse_route_id", "1364970737#0")
CHARGER_ROUTE_ID = _TASK_CFG.get("charger_route_id", "125465016")
_warehouse_pos = _TASK_CFG.get("warehouse_pos", [357600.574872369, 3462308.772003661])
WAREHOUSE_POS = (float(_warehouse_pos[0]), float(_warehouse_pos[1]))

_TASK_GEN_CFG = get_shared_config().get("task_generation", {})
TASK_GEN_MODE = _TASK_GEN_CFG.get("mode", "constant")
CONSTANT_CFG = _TASK_GEN_CFG.get("constant", {})
REALISTIC_CFG = _TASK_GEN_CFG.get("realistic", {})


# =============================================================================
# 任务生成模式说明
# =============================================================================
#
# 一、Constant 模式（恒定速率模式）
#    - 配置项: config/simulation.json 中的 "constant" 字段
#    - 行为: 维持任务池中任务数量恒定，每当任务被分配/完成后立即补充新任务
#
# 二、Realistic 模式（真实场景模式）
#    - 配置项: config/simulation.json 中的 "realistic" 字段
#    - 行为: 模拟真实世界的订单到达模式，任务生成有高峰期和低谷期
#    - 核心参数:
#      * total_tasks: 总任务数量上限
#      * cycle_length: 周期长度（默认 500 steps，对应 50 秒真实时间）
#      * peak_steps_per_task: 高峰期间隔（默认 20 步 ≈ 2 秒/任务）
#      * off_peak_steps_per_task: 低谷期间隔（默认 60 步 ≈ 6 秒/任务）
#      * peak_probability: 高峰窗口内使用高峰间隔的概率（默认 80%）
#
#    - 周期结构（500 steps = 50 秒）:
#      [0, 125): 低谷期   [125, 250): 高峰窗口   [250, 500): 低谷期
#
#    - 高峰窗口内:
#      - 80% 概率使用高峰间隔（20 步 ≈ 2 秒/任务）
#      - 20% 概率仍使用低谷间隔（60 步 ≈ 6 秒/任务）
#    - 所有间隔均有 ±50% 随机波动
#
# 三、热点区域功能（Hotspot）
#    - 配置项: config/simulation.json 中的 "realistic.hotspot" 字段
#    - 启用条件: hotspot.enabled = true
#    - 核心参数:
#      * probability: 本次任务使用热点区域的概率（默认 40%）
#      * radius: 热点区域半径，单位为米（默认 100 米）
#      * centers: 热点中心点坐标列表（UTM 坐标）
#
#    - 任务生成策略（概率分布）:
#      | 场景                     | 概率 |
#      | 起点在热点，终点在普通区域 | 70% |
#      | 起点和终点都在热点区域     | 20% |
#      | 起点在普通区域，终点在热点 | 10% |
#
#    - 位置生成: 在热点圆形区域内使用极坐标随机生成
#
# =============================================================================

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


class ConstantModeGenerator:
    """恒定数量模式生成器：保持任务池有足够任务"""

    def __init__(self, task_gen):
        self._gen = task_gen
        self._max_unassigned = int(CONSTANT_CFG.get("max_unassigned_tasks", 5))
        self._initial_count = int(CONSTANT_CFG.get("initial_task_count", 8))

    @property
    def initial_task_count(self):
        return self._initial_count

    @property
    def max_unassigned_tasks(self):
        return self._max_unassigned

    def should_generate(self, current_time, tasks_generated, unassigned_count):
        """判断是否需要生成任务"""
        return unassigned_count < self._max_unassigned

    def get_generation_count(self, current_time, tasks_generated, unassigned_count):
        """获取需要生成的任务数量"""
        if unassigned_count < self._max_unassigned:
            return self._max_unassigned - unassigned_count
        return 0

    def is_exhausted(self):
        """恒定模式不会耗尽"""
        return False


class RealisticModeGenerator:
    """真实场景模式生成器：模拟高峰期/低谷期，总任务数固定，支持热点区域"""

    def __init__(self, task_gen):
        self._gen = task_gen
        self._total_tasks = int(REALISTIC_CFG.get("total_tasks", 200))
        self._initial_count = int(REALISTIC_CFG.get("initial_task_count", 10))
        self._peak_steps = int(REALISTIC_CFG.get("peak_steps_per_task", 3))
        self._offpeak_steps = int(REALISTIC_CFG.get("off_peak_steps_per_task", 12))
        self._peak_prob = float(REALISTIC_CFG.get("peak_probability", 0.3))
        self._cycle_length = int(REALISTIC_CFG.get("cycle_length", 100))
        self._next_task_step = 0
        self._tasks_generated = 0

        # 热点区域配置
        hotspot_cfg = REALISTIC_CFG.get("hotspot", {})
        self._hotspot_enabled = bool(hotspot_cfg.get("enabled", True))
        self._hotspot_prob = float(hotspot_cfg.get("probability", 0.7))
        self._hotspot_radius = float(hotspot_cfg.get("radius", 200))
        self._hotspot_centers = hotspot_cfg.get("centers", [])

    @property
    def initial_task_count(self):
        return self._initial_count

    @property
    def total_tasks(self):
        return self._total_tasks

    @property
    def has_hotspot(self):
        """检查是否配置了热点区域"""
        return self._hotspot_enabled and len(self._hotspot_centers) > 0

    def reset(self):
        """重置生成器状态"""
        self._next_task_step = 0
        self._tasks_generated = 0

    def should_generate(self, current_time, unassigned_count):
        """判断是否需要生成任务"""
        if self._tasks_generated >= self._total_tasks:
            return False
        if current_time < self._next_task_step:
            return False
        return True

    def get_generation_count(self, current_time, unassigned_count):
        """获取需要生成的任务数量（真实场景模式每次只生成1个）"""
        if not self.should_generate(current_time, unassigned_count):
            return 0
        return 1

    def is_exhausted(self):
        """检查是否已经生成完所有任务"""
        return self._tasks_generated >= self._total_tasks

    def use_hotspot(self):
        """判断本次任务是否使用热点区域"""
        if not self.has_hotspot:
            return False
        return random.random() < self._hotspot_prob

    def generate_hotspot_position(self):
        """在热点区域内生成一个随机位置"""
        if not self.has_hotspot:
            return None

        center = random.choice(self._hotspot_centers)
        # 在圆形区域内随机生成点（使用极坐标）
        angle = random.uniform(0, 2 * 3.141592653589793)
        r = random.uniform(0, self._hotspot_radius)

        x = center[0] + r * random.uniform(0.8, 1.2) * math.cos(angle)
        y = center[1] + r * random.uniform(0.8, 1.2) * math.sin(angle)

        return (x, y)

    def _update_next_step(self, current_time, seed_offset=None):
        """更新下一个任务生成的步骤"""
        if seed_offset is not None:
            random.seed(seed_offset)

        cycle_position = current_time % self._cycle_length
        peak_start = self._cycle_length // 4
        peak_end = peak_start + self._cycle_length // 4

        if peak_start <= cycle_position < peak_end:
            if random.random() < self._peak_prob:
                interval = self._peak_steps
            else:
                interval = self._offpeak_steps
        else:
            interval = self._offpeak_steps

        variance = int(interval * 0.5)
        interval = max(1, interval + random.randint(-variance, variance))
        self._next_task_step = current_time + interval


class TaskGenerator:
    """任务生成器，统一封装任务生成逻辑。

    支持两种模式：
    1. constant: 恒定数量模式，保持任务池有足够任务
    2. realistic: 真实场景模式，模拟高峰期/低谷期，总任务数固定
    """

    MODE_CONSTANT = "constant"
    MODE_REALISTIC = "realistic"

    def __init__(self, possible_destinations=None, file_path=None, mode=None):
        """
        初始化任务生成器

        :param possible_destinations: 配送目的地列表
        :param file_path: 目的地文件路径
        :param mode: 生成模式，可选 "constant" 或 "realistic"
        """
        task_cfg = get_shared_config().get("task", {})
        if file_path is None:
            file_path = task_cfg.get("clicked_positions_file", "clicked_positions.txt")

        if possible_destinations is None:
            possible_destinations = load_destinations_from_file(file_path)

        self.destinations = possible_destinations
        self.task_counter = 0
        self.weight_min = int(task_cfg.get("weight_min", 1))
        self.weight_max = int(task_cfg.get("weight_max", 4))
        self.priority_min = int(task_cfg.get("priority_min", 1))
        self.priority_max = int(task_cfg.get("priority_max", 5))
        self.deadline_offset_min = int(task_cfg.get("deadline_offset_min", 200))
        self.deadline_offset_max = int(task_cfg.get("deadline_offset_max", 500))
        self._same_source_prob = float(
            _TASK_GEN_CFG.get("same_source_prob", 0.0)
        )

        # 模式相关状态
        self._mode = mode if mode else TASK_GEN_MODE
        self._tasks_generated = 0
        self._tasks_exhausted = False
        self._seed_offset = 0

        # 初始化模式生成器
        self._constant_gen = ConstantModeGenerator(self)
        self._realistic_gen = RealisticModeGenerator(self)
        self._realistic_gen._tasks_generated = 0

        # 任务池
        self._unassigned_tasks = []

    @property
    def mode(self):
        """获取当前模式"""
        return self._mode

    @property
    def initial_task_count(self):
        """获取初始任务数量"""
        if self._mode == self.MODE_REALISTIC:
            return self._realistic_gen.initial_task_count
        return self._constant_gen.initial_task_count

    @property
    def max_unassigned_tasks(self):
        """获取最大未分配任务数"""
        return self._constant_gen.max_unassigned_tasks

    @property
    def total_tasks_generated(self):
        """获取已生成的任务总数"""
        return self._tasks_generated

    @property
    def is_exhausted(self):
        """检查是否已生成完所有任务（仅真实场景模式有效）"""
        return self._tasks_exhausted

    @property
    def unassigned_tasks(self):
        """获取当前未分配任务列表"""
        return self._unassigned_tasks

    def set_seed(self, seed):
        """设置随机种子偏移量，用于复现"""
        self._seed_offset = seed if seed else 0

    def reset(self):
        """重置生成器状态"""
        self.task_counter = 0
        self._tasks_generated = 0
        self._tasks_exhausted = False
        self._unassigned_tasks = []
        self._realistic_gen.reset()

    def _get_mode_generator(self):
        """获取当前模式的生成器"""
        if self._mode == self.MODE_REALISTIC:
            return self._realistic_gen
        return self._constant_gen

    def generate_initial_tasks(self, current_time=0):
        """生成初始任务列表"""
        gen = self._get_mode_generator()
        initial_count = gen.initial_task_count
        new_tasks = self._create_tasks(initial_count, current_time)
        self._unassigned_tasks.extend(new_tasks)
        self._tasks_generated += len(new_tasks)
        self._realistic_gen._tasks_generated = self._tasks_generated

        if self._mode == self.MODE_REALISTIC:
            self._realistic_gen._update_next_step(
                current_time,
                self._seed_offset + current_time if self._seed_offset else None
            )

        return new_tasks

    def step(self, current_time):
        """
        执行一步任务生成逻辑（仅真实场景模式需要调用此方法）

        :param current_time: 当前模拟时间
        :return: 新生成的任务列表
        """
        new_tasks = []

        if self._mode == self.MODE_REALISTIC:
            if self._tasks_exhausted:
                return new_tasks

            gen = self._realistic_gen
            if gen.should_generate(current_time, len(self._unassigned_tasks)):
                tasks_to_gen = gen.get_generation_count(current_time, len(self._unassigned_tasks))
                new_tasks = self._create_tasks(tasks_to_gen, current_time)
                self._unassigned_tasks.extend(new_tasks)
                self._tasks_generated += len(new_tasks)
                gen._tasks_generated = self._tasks_generated
                gen._update_next_step(
                    current_time,
                    self._seed_offset + current_time + self._tasks_generated if self._seed_offset else None
                )

                if self._tasks_generated >= gen.total_tasks:
                    self._tasks_exhausted = True

        else:
            gen = self._constant_gen
            if gen.should_generate(current_time, self._tasks_generated, len(self._unassigned_tasks)):
                count = gen.get_generation_count(current_time, self._tasks_generated, len(self._unassigned_tasks))
                new_tasks = self._create_tasks(count, current_time)
                self._unassigned_tasks.extend(new_tasks)
                self._tasks_generated += len(new_tasks)

        return new_tasks

    def _create_tasks(self, num_tasks, current_time):
        """创建指定数量的任务"""
        if len(self.destinations) < 2:
            print("Warning: Need at least 2 destinations for task generation.")
            return []

        tasks = []
        for _ in range(max(0, int(num_tasks))):
            self.task_counter += 1
            weight = random.randint(self.weight_min, self.weight_max)

            # 根据模式生成起点和终点
            source, destination = self._generate_source_destination(current_time)

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

    def _generate_source_destination(self, current_time):
        """
        根据模式生成起点和终点

        Realistic模式支持热点区域地理聚集性
        当 same_source_prob > 0 时，有一定概率复用已有未分配任务的起点，
        模拟同一取货点产生多个订单的场景。
        """
        # 同源复用：从现有未分配任务中随机选取一个 source
        if (self._same_source_prob > 0
                and len(self._unassigned_tasks) > 0
                and random.random() < self._same_source_prob):
            existing_task = random.choice(self._unassigned_tasks)
            source = existing_task.get_source()
            # 选择一个不同的目的地
            destination = random.choice(self.destinations)
            retries = 0
            while destination == source and retries < 10:
                destination = random.choice(self.destinations)
                retries += 1
            return (source, destination)

        # Realistic 模式使用热点区域
        if self._mode == self.MODE_REALISTIC:
            gen = self._realistic_gen
            if gen.has_hotspot and gen.use_hotspot():
                # 使用热点区域生成起点和终点
                return self._generate_hotspot_task()
            else:
                # 使用普通随机选择
                return random.sample(self.destinations, 2)

        # Constant 模式使用普通随机选择
        return random.sample(self.destinations, 2)

    def _generate_hotspot_task(self):
        """
        在热点区域内生成起点和终点

        策略：
        1. 选择一个热点中心
        2. 70%概率：起点在热点区域，终点在普通区域
        3. 20%概率：起点和终点都在热点区域
        4. 10%概率：起点在普通区域，终点在热点区域
        """
        gen = self._realistic_gen
        strategy = random.random()

        if strategy < 0.7:
            # 热点 -> 普通
            source = gen.generate_hotspot_position()
            destination = random.choice(self.destinations)
        elif strategy < 0.9:
            # 热点 -> 热点
            source = gen.generate_hotspot_position()
            destination = gen.generate_hotspot_position()
        else:
            # 普通 -> 热点
            source = random.choice(self.destinations)
            destination = gen.generate_hotspot_position()

        return (source, destination)

    def generate_random_tasks(self, num_tasks=5, current_time=0):
        """直接生成随机任务（兼容旧接口）"""
        return self._create_tasks(num_tasks, current_time)
