from environment import Environment
from drone import Drone
from task import WAREHOUSE_POS
from map_drawer import OptimizedMapViewer
from scheduler import TaskGenerator, GreedyScheduler
from task_logger import TaskLogger

if __name__ == "__main__":
    env = Environment('data/map/part_of_yangpu.osm')

    # 创建多架无人机（从仓库位置出发）
    NUM_DRONES = 3  # 可根据需要调整无人机数量
    env.drones = [
        Drone(WAREHOUSE_POS[0], WAREHOUSE_POS[1], drone_id=f"drone_{i}")
        for i in range(NUM_DRONES)
    ]

    # 初始化任务日志记录器
    logger = TaskLogger(file_path='task_log.jsonl')

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
                for task in assigned:
                    task.mark_assigned()
                    print(f"Drone {drone.drone_id} assigned: {task}")
                    drone.add_load(task.weight)
                drone.pending_tasks.extend(assigned)
                # 使用绕行路径规划
                route = env.plan_route_for_tasks(drone, assigned)
                drone.schedule_route(route)

    viewer = OptimizedMapViewer('data/map/part_of_yangpu.osm')
    frame_count = 0
    running = True
    while running:
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
        
        # 收集各无人机已完成的任务并写入日志
        for drone in env.drones:
            while drone.completed_tasks:
                done_task = drone.completed_tasks.pop(0)
                latency = done_task.get_latency_info()
                print(
                    f"[Done] {done_task.task_id} | "
                    f"wait={latency['wait_time']:.2f}s  "
                    f"exec={latency['execution_time']:.2f}s  "
                    f"total={latency['total_time']:.2f}s"
                )
                logger.log_task(done_task)

        # 多轮调度：检查空闲无人机并继续分配剩余任务
        for drone in env.drones:
            if drone.is_free:  # 无人机空闲
                if unassigned_tasks:
                    # 分配新任务
                    assigned = GreedyScheduler.schedule_for_drone(drone, unassigned_tasks)
                    if assigned:
                        for task in assigned:
                            task.mark_assigned()
                            print(f"Drone {drone.drone_id} assigned: {task}")
                            drone.add_load(task.weight)
                        drone.pending_tasks.extend(assigned)
                        # 使用绕行路径规划
                        route = env.plan_route_for_tasks(drone, assigned)
                        drone.schedule_route(route)
        
        # 检查pygame窗口事件，如果窗口被关闭则退出循环
        running = viewer.render(env.drones)