import math
import random
from charging_station import DEFAULT_CHARGING_STATION


# ==================== 电池系统常量 ====================
BATTERY_CAPACITY = 15000.0  # 电池最大容量 (Wh)，足够大，确保20%电量能完成任意订单
BATTERY_CONSUMPTION_BASE = 0.5  # 飞行基础功耗 (Wh/m)
BATTERY_LOAD_PENALTY_FACTOR = 0.3  # 载重惩罚系数（满载时额外消耗比例）
BATTERY_LOW_THRESHOLD = 0.2  # 低电量阈值，低于此值无法接单

# ==================== 订单生成密集点 ====================
# 从 clicked_positions.txt 中选取的点
ORDER_HOTSPOTS = [
    (358919.252551, 3463875.611049),
    (359006.728578, 3464942.818570),
    (358639.329267, 3462878.384349),
    (357545.878938, 3463569.444957),
    (359365.380286, 3463070.831606),
    (357983.023875, 3462464.936166),
    (359505.341928, 3464487.943233),
]


# ==================== 无人机类 ====================
class Drone:
    def __init__(self, x, y, drone_id="drone_0", carrying_capacity=5,
                 battery_capacity=BATTERY_CAPACITY):
        # 位置信息
        self.x = x
        self.y = y
        self.drone_id = drone_id
        self.home_position = (x, y)  # 记录出发点位置（用于返回装货）
        
        # 载重信息
        self.carrying_capacity = carrying_capacity
        self.current_load = 0  # 当前载重
        
        # 任务信息
        self.tasks = []
        self.scheduled_position = []
        self.executing_task_id = None  # 当前正在执行的已分配任务ID
        self.is_free = True # 表示无人机是否可以接单
        
        # ==================== 电量系统 ====================
        self.battery_capacity = battery_capacity  # 电池最大容量 (Wh)
        self.current_battery = battery_capacity  # 当前电量 (Wh)，初始满电
        self.is_charging = False  # 是否正在充电
        self.charging_station_id = None  # 当前所在充电站ID
        
    # ==================== 电量相关方法 ====================
    
    def get_battery_level(self):
        """获取电量百分比 (0.0 ~ 1.0)"""
        return self.current_battery / self.battery_capacity
    
    def is_low_battery(self, threshold=BATTERY_LOW_THRESHOLD):
        """检查是否低电量"""
        return self.get_battery_level() < threshold
    
    def consume_battery(self, distance):
        """
        消耗电量
        
        消耗公式：
            总消耗 = 基础消耗 × (1 + 载重惩罚)
            其中 载重惩罚 = (current_load / carrying_capacity) × LOAD_PENALTY_FACTOR
        """
        # 基础飞行消耗
        base_consumption = distance * BATTERY_CONSUMPTION_BASE
        
        # 载重影响
        load_factor = (self.current_load / self.carrying_capacity) * BATTERY_LOAD_PENALTY_FACTOR
        total_consumption = base_consumption * (1 + load_factor)
        
        # 更新电量（不能低于0）
        self.current_battery = max(0, self.current_battery - total_consumption)
        
        return total_consumption
    
    def start_charging(self, station_id):
        """开始充电"""
        self.is_charging = True
        self.charging_station_id = station_id
        self.is_free = False  # 充电中不可接单
    
    def stop_charging(self):
        """停止充电"""
        self.is_charging = False
        self.charging_station_id = None
    
    def charge(self, charging_power, time_step=0.1):
        """
        充电
        
        Args:
            charging_power: 充电功率 (Wh/step)
            time_step: 时间步长
        """
        if not self.is_charging:
            return
        
        # 充入电量
        charged = charging_power * time_step
        self.current_battery = min(self.battery_capacity, self.current_battery + charged)
        
        # 充满后停止充电
        if self.current_battery >= self.battery_capacity:
            self.current_battery = self.battery_capacity
            self.stop_charging()
            self.is_free = True  # 充满后可重新接单
    
    def reset_battery(self):
        """重置电量为满电（回到仓库时调用）"""
        self.current_battery = self.battery_capacity

    def schedule_route(self, position, task_id=None):
        self.scheduled_position = position
        self.executing_task_id = task_id
        # 只有执行调度器分配的正式任务时才标记为忙碌
        # 自动飞往充电站/待机不算正式任务，is_free保持True
        if task_id is not None:
            self.is_free = False

    def append_route(self, position):
        """追加航点，不覆盖现有航点"""
        if self.scheduled_position:
            self.scheduled_position.extend(position)
        else:
            self.scheduled_position = list(position)
        self.is_free = False

    def return_to_base(self, base_position):
        """返回出发点装货"""
        self.scheduled_position = [base_position]
        self.current_load = 0
        self.is_free = False

    def add_load(self, weight):
        """装载货物"""
        self.current_load += weight

    def get_remaining_capacity(self):
        """获取剩余载重"""
        return self.carrying_capacity - self.current_load

    def update(self, time_step=0.1):
        v = 200  # 速度，值越大移动越快
        max_distance = v * time_step
        
        # 如果正在充电，不移动，只充电
        if self.is_charging:
            self.charge(DEFAULT_CHARGING_STATION.charging_power, time_step)
            return
        
        if self.scheduled_position:
            # Get the next target position (支持新旧两种格式)
            target = self.scheduled_position[0]
            if len(target) >= 3:
                target_x, target_y, _ = target  # 新格式: (x, y, type)
            else:
                target_x, target_y = target  # 旧格式: (x, y)
            
            # Calculate direction vector
            dx = target_x - self.x
            dy = target_y - self.y
            
            # Calculate distance to target
            distance = (dx**2 + dy**2)**0.5
            
            if distance <= max_distance:
                # If we're close enough to target, move directly to it
                self.x = target_x
                self.y = target_y
                # 到达目标，消耗电量
                self.consume_battery(distance)
                # Remove this target from schedule as we've reached it
                self.scheduled_position.pop(0)
                if not self.scheduled_position:
                    self.is_free = True
                    self.current_load = 0  # 任务完成，卸货
                    self.executing_task_id = None  # 清除执行中任务ID
                    # 任务完成后：根据电量决定去向
                    if self.is_low_battery():
                        # 电量不足，检查是否已在充电站
                        drone_pos = (self.x, self.y)
                        station_pos = DEFAULT_CHARGING_STATION.get_position()
                        if drone_pos == station_pos:
                            # 已在充电站，开始充电
                            self.start_charging(DEFAULT_CHARGING_STATION.station_id)
                        else:
                            # 不在充电站，飞往充电站
                            self.is_free = False
                            self.schedule_route([station_pos])
                    # else: 电量充足，原地待机，等待调度器分配新任务
            else:
                # Move a step towards the target
                # Normalize the direction and multiply by time_step
                actual_distance = max_distance
                self.x += (dx / distance) * actual_distance
                self.y += (dy / distance) * actual_distance
                # 飞行消耗电量
                self.consume_battery(actual_distance)

    def get_position(self):
        return (self.x, self.y)