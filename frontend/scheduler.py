import math


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
        1. 从观察空间中获取无人机位置、空闲状态、电量信息和未分配任务信息
        2. 为每个空闲且电量充足（>=20%）的无人机分配一个离当前位置最近的任务
        返回: {drone_index: [assigned_task_id]} 字典（每个无人机最多分配一个任务）
        """
        drone_positions = observation['drone_positions']
        unassigned_tasks_info = list(observation['unassigned_tasks'])
        drone_free_masks = observation['drone_free_masks']

        if not unassigned_tasks_info:
            return {}

        assignments = {}

        # 为每个空闲且电量充足的无人机分配任务
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
            'drone_positions': [list(d.get_position()) for d in drones],
            'drone_free_masks': [[d.is_free for _ in unassigned_tasks_info] for d in drones],
            'unassigned_tasks': unassigned_tasks_info,
        }

        task_id_map = {task.task_id: task for task in unassigned_tasks}
        id_assignments = GreedyScheduler.schedule_for_drone(observation)

        for drone_idx, task_ids in id_assignments.items():
            if drone_idx >= len(drones):
                continue
            drone = drones[drone_idx]
            assigned_tasks = [task_id_map[tid] for tid in task_ids if tid in task_id_map]
            assignments[drone] = assigned_tasks

            if assigned_tasks:
                route = [drone.get_position()] + [t.get_destination() for t in assigned_tasks]
                drone.schedule_route(route, assigned_tasks[0].task_id)
        
        return assignments


def greedy_action_from_observation(observation):
    """Callable greedy policy entry for both frontend and backend usage."""
    return GreedyScheduler.schedule_for_drone(observation)
