import traci
from map_drawer import PygameVisualizer

# 连接到SUMO仿真
traci.start(["sumo", "-c", "data/siping/siping.sumocfg"])


# 创建路线 - 连接起始边和目标边
# 需要找到连接这两个边的有效路径
route_edges = ["-1364682825#0"]  # 起始边

start_route_edges = ["-1364970735", "-1364939699", "-1364682825#0", "-1364682816"]



for i in range(len(start_route_edges)):
    start_edge = start_route_edges[i]
    target_edge = "1364970737#0"  # 目标边
    # 如果目标边与起始边直接相连，则添加它；否则需要找到路径
    if traci.edge.getIDList():  # 确保网络存在
        # 使用SUMO的路由功能查找路径
        try:
            # 查找从起始边到目标边的路径
            route_edges = traci.simulation.findRoute(start_edge, target_edge).edges
        except:
            # 如果找不到路径，使用简单的两段路线（仅当它们直接相连时有效）
            route_edges = [start_edge, target_edge]

    # 注册路线
    route_id = f"route_{i}"
    traci.route.add(route_id, route_edges)

    # 添加自行车并设置目的地
    veh_id = f"bike_{i}"
    traci.vehicle.add(
        vehID=veh_id,
        routeID=route_id,  # 使用新创建的路线
        typeID="bicycle_type",
        depart=0,
        departLane='first',
        departPos='base',
        departSpeed=0
    )

visualizer = PygameVisualizer()

# 运行 100 个仿真步
for step in range(1000):
    traci.simulationStep()
    visualizer.render()

    # 每 10 步打印一次车辆信息
    if step % 10 == 0:
        vehicle_ids = traci.vehicle.getIDList()
        print(f"时间: {traci.simulation.getTime():.2f}s, 车辆数: {len(vehicle_ids)}")
        
        # for veh_id in vehicle_ids:
        #     try:
        #         pos = traci.vehicle.getPosition(veh_id)
        #         angle = traci.vehicle.getAngle(veh_id)
        #         speed = traci.vehicle.getSpeed(veh_id)
        #         edge = traci.vehicle.getRoadID(veh_id)
        #         destination = traci.vehicle.getRoute(veh_id)[-1]
        #         # traci.edge.get

        #         # edge_id = traci.edge.getIDList()[-1]
        #         num_lanes = traci.edge.getLaneNumber(destination)
        #         lane_id = f"{destination}_0"
        #         shape = traci.lane.getShape(lane_id)[-1]

        #         print(f"  {veh_id}: 边缘 {edge}, 位置 {pos}, 角度: {angle:.2f}°, 速度: {speed:.2f}m/s, 目的地: {shape}")
        #     except traci.exceptions.TraCIException:
        #         print(f"  {veh_id}: 无法获取位置信息")

traci.close()
