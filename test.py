from map_drawer import PygameVisualizer
from drone import Drone, traci
from task import *

destination_route_ids = ["-1364970735", "-1364939699", "-1364682825#0", "-1364682816"]

task0 = Task(task_id="task_0", weight=3, destination=destination_route_ids[0])
task1 = Task(task_id="task_1", weight=5, destination=destination_route_ids[1])
go_to_charger_task = Task(task_id="go_to_charger", weight=0, destination=CHARGER_ROUTE_ID)
go_to_warehouse_task = Task(task_id="go_to_warehouse", weight=0, destination=WAREHOUSE_ROUTE_ID)

drones = []
drone1 = Drone()
drone1.assign_task_list([task0, task1, go_to_charger_task, go_to_warehouse_task])
drones.append(drone1)

drone2 = Drone()
drone2.assign_task_list([go_to_charger_task, go_to_warehouse_task])
drones.append(drone2)

visualizer = PygameVisualizer()

# 运行 100 个仿真步
for step in range(1000):
    traci.simulationStep()
    for drone in drones:
        drone.update()

    visualizer.render(drones)

    # 每 10 步打印一次车辆信息
    if step % 10 == 0:
        vehicle_ids = traci.vehicle.getIDList()
        print(f"时间: {traci.simulation.getTime():.2f}s, 车辆数: {len(vehicle_ids)}")

    # 给第一个无人机持续安排任务
    if drones[0].is_free:
        print("所有任务已完成")

        for i in range(100):
            traci.simulationStep()
        drones[0].assign_task_list([task0, task1])
        
traci.close()
