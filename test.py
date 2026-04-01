from map_drawer import PygameVisualizer
from drone import Drone, traci
from task import Task, WAREHOUSE_ROUTE_ID
from scheduler import TaskGenerator, GreedyScheduler

# 可用的目标边缘 ID
destination_route_ids = ["-1364970735", "-1364939699", "-1364682825#0", "-1364682816"]

# 初始化订单生成器和任务池
task_generator = TaskGenerator(destination_route_ids)
unassigned_tasks = task_generator.generate_random_tasks(10) # 初始生成10个任务

print(f"初始化生成了 {len(unassigned_tasks)} 个待分配任务。")

# 初始化无人机
drones = [Drone(), Drone()]
visualizer = PygameVisualizer()

# 运行 2000 个仿真步
for step in range(2000):
    traci.simulationStep()
    
    # 动态订单生成机制：每过 300 步，再随机生成 3 个新任务
    if step > 0 and step % 300 == 0:
        new_tasks = task_generator.generate_random_tasks(3)
        unassigned_tasks.extend(new_tasks)
        print(f"[系统] 新增 3 个任务，当前待分配任务池剩余: {len(unassigned_tasks)} 个")

    for drone in drones:
        drone.update()
        
        # 调度逻辑：如果无人机空闲（通常说明它刚被创建，或者已经回到了仓库），则为其分配任务
        if drone.is_free and len(unassigned_tasks) > 0:
            print(f"\n--- 正在为无人机 {drone.drone_id[:8]} 规划路线 ---")
            
            # 使用贪心算法分配最多2个任务 + 1个回程任务
            scheduled_tasks = GreedyScheduler.schedule_for_drone(drone, unassigned_tasks)
            
            if scheduled_tasks:
                # 打印分配情况（不打印回程任务）
                delivery_tasks = [t for t in scheduled_tasks if t.weight > 0]
                print(f"分配成功! 运送 {len(delivery_tasks)} 个订单:")
                for i, t in enumerate(delivery_tasks):
                    print(f"  -> 任务{i+1}: ID={t.task_id}, 重量={t.weight}kg, 目的地={t.destination}")
                
                # 下达任务
                drone.assign_task_list(scheduled_tasks)
            else:
                print("当前没有满足条件的任务（可能是无人机载重不足）。")

    visualizer.render(drones)

    # 每 100 步打印一次系统状态
    if step % 100 == 0:
        vehicle_ids = traci.vehicle.getIDList()
        print(f"时间: {traci.simulation.getTime():.2f}s, 地图内车辆数: {len(vehicle_ids)}, 剩余未分配任务: {len(unassigned_tasks)}")
        
traci.close()