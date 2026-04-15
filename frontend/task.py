WAREHOUSE_ROUTE_ID = "1364970737#0"
CHARGER_ROUTE_ID = "125465016"

class Task:
    def __init__(self, task_id, weight, destination):
        """
        初始化任务对象
        
        :param task_id: 任务唯一标识符
        :param weight: 物体的重量 (单位: kg)
        :param destination: 目标地点 (可以是坐标、地址或区域)
        :param priority: 任务优先级 (默认为1，数值越大优先级越高)
        :param source: 起始地点 (可选参数)
        """
        self.task_id = task_id
        self.weight = weight
        self.destination = destination
        self.status = "pending"  # 任务状态: pending, assigned, in_progress, completed, failed
    
    def set_destination(self, new_destination):
        """更新目标地点"""
        self.destination = new_destination
    
    def get_weight(self):
        """获取物体重量"""
        return self.weight
    
    def get_destination(self):
        """获取目标地点"""
        return self.destination
    
    def update_status(self, status):
        """更新任务状态"""
        valid_statuses = ["pending", "assigned", "in_progress", "completed", "failed"]
        if status in valid_statuses:
            self.status = status
        else:
            raise ValueError(f"Invalid status: {status}. Valid statuses are: {valid_statuses}")
    
    def __str__(self):
        """返回任务的字符串表示"""
        return f"Task(ID: {self.task_id}, Weight: {self.weight}kg, Destination: {self.destination}, Status: {self.status})"
    
    def __repr__(self):
        """返回任务的详细表示"""
        return self.__str__()