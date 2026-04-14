from environment import Environment
from drone import Drone
from task import WAREHOUSE_POS
from map_drawer import OptimizedMapViewer
from scheduler import TaskGenerator, GreedyScheduler

if __name__ == "__main__":
    env = Environment('data/map/part_of_yangpu.osm')

    # 创建多架无人机（从仓库位置出发）
    NUM_DRONES = 3  # 可根据需要调整无人机数量
    env.drones = [
        Drone(WAREHOUSE_POS[0], WAREHOUSE_POS[1], drone_id=f"drone_{i}")
        for i in range(NUM_DRONES)
    ]

    # 从点击文件读取配送点并生成任务
    generator = TaskGenerator(file_path='clicked_positions.txt')
    
    # 任务池：持续生成新任务
    MAX_UNASSIGNED_TASKS = 20  # 任务池中最大未分配任务数
    unassigned_tasks = []
    
    # 初始生成一批任务
    new_tasks = generator.generate_random_tasks(num_tasks=8)
    unassigned_tasks.extend(new_tasks)
    print(f"Generated {len(new_tasks)} tasks:")
    for task in new_tasks:
        print(f"  {task}")
    
    # 初始调度：所有无人机从仓库出发分配任务
    for drone in env.drones:
        if unassigned_tasks:
            assigned = GreedyScheduler.schedule_for_drone(drone, unassigned_tasks)
            if assigned:
                # 构建无人机的完整路线：当前位置 -> 任务1起始点 -> 任务1终点 -> 任务2起始点 -> 任务2终点 -> ...
                route = [drone.get_position()]
                for task in assigned:
                    print(f"Drone {drone.drone_id} assigned: {task}")
                    drone.add_load(task.weight)
                    # 添加任务的起始点和终点到路线
                    route.append(task.get_source())
                    route.append(task.get_destination())
                drone.schedule_route(route)

    viewer = OptimizedMapViewer('data/map/part_of_yangpu.osm')
    frame_count = 0
    while True:
        env.update()
        frame_count += 1
        
        # 每60帧（约1秒）输出一次状态
        if frame_count % 60 == 0:
            free_drones = sum(1 for d in env.drones if d.is_free)
            busy_drones = sum(1 for d in env.drones if not d.is_free)
            print(f"[Status] Unassigned tasks: {len(unassigned_tasks)}, Free drones: {free_drones}, Busy drones: {busy_drones}")
        
        # 持续生成新任务，保持任务池有足够任务
        if len(unassigned_tasks) < MAX_UNASSIGNED_TASKS:
            new_tasks = generator.generate_random_tasks(num_tasks=5)
            unassigned_tasks.extend(new_tasks)
        
        # 多轮调度：检查空闲无人机并继续分配剩余任务
        for drone in env.drones:
            if drone.is_free:  # 无人机空闲
                if unassigned_tasks:
                    # 分配新任务
                    assigned = GreedyScheduler.schedule_for_drone(drone, unassigned_tasks)
                    if assigned:
                        # 构建无人机的完整路线
                        route = [drone.get_position()]
                        for task in assigned:
                            print(f"Drone {drone.drone_id} assigned: {task}")
                            drone.add_load(task.weight)
                            route.append(task.get_source())
                            route.append(task.get_destination())
                        drone.schedule_route(route)
        
        viewer.render(env.drones)