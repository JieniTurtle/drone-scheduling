import traci
from task import *
import uuid
from math import sqrt


traci.start(["sumo", "-c", "data/siping/siping.sumocfg"])

class Drone:
    def __init__(self):
        self.drone_id = str(uuid.uuid4())  
        self.battery_level = 100  # 电池电量百分比
        self.carrying_capacity = 5  # 最大载重 (单位: kg)
        # self.current_load = 0  # 当前载重 (单位: kg)
        self.current_task = None  # 当前任务

        self.is_free = True  # 无人机是否空闲
        self.task_list = []  # 任务列表
        self.current_task_index = 0  # 当前正在执行的任务索引
        self.veh_id = None  # 车辆ID
        self.completed_tasks = []  # 已完成的任务列表
        self.arrived_at_first_task = False  # 是否已到达第一个任务点

        self.last_known_position = None  # 保存最后已知位置
        self.current_edge = None  # 保存当前边

    def assign_task_list(self, task_list : list[Task]):
        """分配任务给无人机"""
        start_edge = WAREHOUSE_ROUTE_ID
        
        # 决定起始边：如果是第一次，则从仓库开始；否则从上次的位置开始
        if self.current_edge is None:
            current_edge = start_edge
        else:
            current_edge = self.current_edge
            print(f"从之前的位置继续: {current_edge}")
        
        full_route = [current_edge]  # 初始化完整路线，包含起始边
        self.task_list = task_list  # 保存任务列表
        
        for task in task_list:
            if task.get_weight() > self.carrying_capacity:
                raise ValueError("Task weight exceeds drone's carrying capacity.")
            task.update_status("assigned")

            target_edge = task.get_destination()
            
            try:
                # 获取从当前边到目标边的路径
                route_result = traci.simulation.findRoute(current_edge, target_edge, vType="bicycle_type")
                route_edges = route_result.edges
                
                if len(route_edges) > 0:
                    # 跳过第一个边（因为它与当前边重复），然后添加剩余的边
                    if route_edges[0] == current_edge:
                        full_route.extend(route_edges[1:])  # 跳过第一个边避免重复
                    else:
                        full_route.extend(route_edges)  # 如果不重复就全部添加
                else:
                    # 如果找不到路径，直接添加目标边
                    full_route.append(target_edge)
                
                # 更新当前边为本次任务的目标边，用于下一次路径计算
                current_edge = target_edge
                
            except Exception as e:
                print(f"路径计算错误: {e}")
                # 如果出错，直接添加目标边
                full_route.append(target_edge)
                current_edge = target_edge

        # 确保最终路线中没有重复的相邻边
        final_route = []
        for edge in full_route:
            if not final_route or final_route[-1] != edge:
                final_route.append(edge)

        # 注册路线
        route_id = f"route_{uuid.uuid4()}"
        print(f"注册路线 {route_id}: {final_route}")
        traci.route.add(route_id, final_route)

        # 如果已有车辆，先删除它
        if self.veh_id and self.veh_id in traci.vehicle.getIDList():
            try:
                traci.vehicle.remove(self.veh_id)
            except traci.exceptions.TraCIException:
                pass  # 车辆可能已被删除

        # 创建新的车辆
        self.veh_id = f"bike_{self.drone_id}_{int(traci.simulation.getTime())}"
        traci.vehicle.add(
            vehID=self.veh_id,
            routeID=route_id,
            typeID="bicycle_type",
            depart=int(traci.simulation.getTime()),
            departLane='first',
            departPos='base',
            departSpeed=0
        )

        self.is_free = False  # 无人机现在有任务了，不再空闲
        self.current_task_index = 0  # 重置任务索引

    def update(self):
        if self.is_free:
            return False
        # print("task_list:", self.task_list, "is free: ", self.is_free)
        """更新无人机状态，检查是否到达目标点"""
        try:
            pos = traci.vehicle.getPosition(self.veh_id)
            
            # 获取当前任务的目标位置
            current_task_dest = self.get_current_task_destination()
            if not current_task_dest:
                return False  # 没有任务可执行
            
            lane_shape = traci.lane.getShape(f"{current_task_dest}_0")
            current_task_target_pos = lane_shape[0]  # 使用目标边的第一个点作为目标位置
            distance_to_current_task = self.calculate_distance(pos, current_task_target_pos)
            
            # 设定一个阈值，当距离小于该值时认为到达目标
            arrival_threshold = 3.0  # 10米内算到达
            
            if distance_to_current_task <= arrival_threshold:
                # 记录完成的任务
                if self.current_task_index < len(self.task_list):
                    completed_task = self.task_list[self.current_task_index]
                    completed_task.update_status("completed")
                    self.completed_tasks.append(completed_task)
                
                self.current_task_index += 1
                
                if self.current_task_index >= len(self.task_list):
                    # 所有任务完成，保存当前位置并删除车辆
                    self.last_known_position = pos
                    try:
                        # 获取当前所在的边
                        self.current_edge = traci.vehicle.getRoadID(self.veh_id)
                    except traci.exceptions.TraCIException:
                        # 如果无法获取边ID，使用最近的边
                        self.current_edge = traci.simulation.convertRoad(pos[0], pos[1], isGeo=False)
                    
                    # 删除车辆
                    traci.vehicle.remove(self.veh_id)
                    self.veh_id = None
                    self.is_free = True
                    print(f"无人机完成所有任务，位置保存为: {self.current_edge}")
                else:
                    # 到达中间目标点，继续下一个任务
                    print(f"无人机 {self.drone_id} 已到达任务 {self.current_task_index-1} 的目标点！")
                
                return True  # 返回True表示到达了任务点
            
            return False  # 返回False表示尚未到达任务点
            
        except traci.exceptions.TraCIException as e:
            # 如果获取位置失败，可能是车辆已经被删除
            if self.veh_id not in traci.vehicle.getIDList():
                self.is_free = True
            print(f"获取车辆信息时出错: {e}")
            return False

    def calculate_distance(self, pos1, pos2):
        """计算两个坐标点之间的欧几里得距离"""
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        return sqrt(dx*dx + dy*dy)

    def get_current_position(self):
        """获取无人机当前位置"""
        try:
            if self.veh_id and self.veh_id in traci.vehicle.getIDList():
                return traci.vehicle.getPosition(self.veh_id)
            elif self.last_known_position:
                return self.last_known_position
        except traci.exceptions.TraCIException:
            return None

    def get_vehicle_id(self):
        """获取车辆ID"""
        return self.veh_id
    
    def get_current_task_destination(self):
        """获取当前任务的目标位置"""
        if self.task_list and self.current_task_index < len(self.task_list):
            return self.task_list[self.current_task_index].get_destination()
        else:
            return None