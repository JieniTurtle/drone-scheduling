class Drone:
    def __init__(self, x, y, drone_id="drone_0", carrying_capacity=5):
        self.x = x
        self.y = y
        self.drone_id = drone_id
        self.carrying_capacity = carrying_capacity
        self.current_load = 0  # 当前载重
        self.tasks = []
        self.scheduled_position = []
        self.is_free = True
        self.home_position = (x, y)  # 记录出发点位置（用于返回装货）

    def schedule_route(self, position):
        self.scheduled_position = position
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
                # Remove this target from schedule as we've reached it
                self.scheduled_position.pop(0)
                if not self.scheduled_position:
                    self.is_free = True
                    self.current_load = 0  # 任务完成，卸货
            else:
                # Move a step towards the target
                # Normalize the direction and multiply by time_step
                self.x += (dx / distance) * max_distance
                self.y += (dy / distance) * max_distance

    def get_position(self):
        return (self.x, self.y)