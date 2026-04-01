import traci
import pygame
from drone import Drone
from task import Task

# 定义类型常量
DRONE = 1
WAREHOUSE = 2
CHARGER = 3

WAREHOUSE_POS = (1120, 1000)
CHARGER_POS = (1127, 1449)

class PygameVisualizer:
    def __init__(self, width=1200, height=800, fps=30):
        # self.sumo_cfg_path = sumo_cfg_path
        self.width = width
        self.height = height
        self.fps = fps

        # 初始化pygame
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("SUMO Traffic Simulation")
        self.clock = pygame.time.Clock()
        
        # 坐标转换参数（SUMO坐标 -> 屏幕坐标）
        self.world_min_x = float('inf')
        self.world_max_x = float('-inf')
        self.world_min_y = float('inf')
        self.world_max_y = float('-inf')
        self.scale_x = 1
        self.scale_y = 1
        self.offset_x = 0
        self.offset_y = 0
        
        # 缓存
        self.road_surfaces = {}  # 预渲染的道路表面
        self.vehicle_colors = {}

        self.draw_roads()


        self.drone_image = pygame.image.load("assets/drone.png")
        self.drone_image = pygame.transform.scale(self.drone_image, (20, 20))
        self.warehouse_image = pygame.image.load("assets/warehouse.jpeg")
        self.warehouse_image = pygame.transform.scale(self.warehouse_image, (40, 40))
        self.charger_image = pygame.image.load("assets/charger.jpeg")
        self.charger_image = pygame.transform.scale(self.charger_image, (40, 40))

        warehouse_pos_screen = self.world_to_screen(WAREHOUSE_POS[0], WAREHOUSE_POS[1])
        self.background.blit(self.warehouse_image, (warehouse_pos_screen[0] - 20, warehouse_pos_screen[1] - 20))

        charger_pos_screen = self.world_to_screen(CHARGER_POS[0], CHARGER_POS[1])
        self.background.blit(self.charger_image, (charger_pos_screen[0] - 20, charger_pos_screen[1] - 20))

        
    def calculate_transform(self):
        """计算坐标转换参数"""
        # 获取世界坐标范围
        edge_ids = traci.edge.getIDList()
        for edge_id in edge_ids:
            try:
                num_lanes = traci.edge.getLaneNumber(edge_id)
                for i in range(num_lanes):
                    lane_id = f"{edge_id}_{i}"
                    shape = traci.lane.getShape(lane_id)
                    for x, y in shape:
                        self.world_min_x = min(self.world_min_x, x)
                        self.world_max_x = max(self.world_max_x, x)
                        self.world_min_y = min(self.world_min_y, y)
                        self.world_max_y = max(self.world_max_y, y)
            except:
                pass
        

        # 计算世界坐标尺寸
        world_width = self.world_max_x - self.world_min_x
        world_height = self.world_max_y - self.world_min_y
        
        # 计算缩放比例（取较小的比例以保持长宽比）
        scale_x = self.width / world_width
        scale_y = self.height / world_height
        self.scale = min(scale_x, scale_y)  # 使用相同的缩放比例
        
        # 计算偏移量，使地图居中显示
        scaled_world_width = world_width * self.scale
        scaled_world_height = world_height * self.scale
        
        self.offset_x = (self.width - scaled_world_width) / 2 - self.world_min_x * self.scale
        self.offset_y = (self.height - scaled_world_height) / 2 - self.world_min_y * self.scale

    def world_to_screen(self, x, y):
        """世界坐标转屏幕坐标（使用统一缩放比例）"""
        screen_x = x * self.scale + self.offset_x
        screen_y = self.height - y * self.scale - self.offset_y
        return int(screen_x), int(screen_y)
    def draw_roads(self):
        """绘制所有道路（预渲染）"""
        self.calculate_transform()
        
        # 创建背景表面
        background = pygame.Surface((self.width, self.height))
        background.fill((255, 255, 255))  # 白色背景
        
        # 绘制道路
        edge_ids = traci.edge.getIDList()
        for edge_id in edge_ids:
            try:
                num_lanes = traci.edge.getLaneNumber(edge_id)
                for i in range(num_lanes):
                    lane_id = f"{edge_id}_{i}"
                    shape = traci.lane.getShape(lane_id)
                    
                    if len(shape) >= 2:
                        # 转换坐标
                        points = [self.world_to_screen(x, y) for x, y in shape]
                        if len(points) >= 2:
                            # 根据车道数设置不同颜色
                            if num_lanes == 1:
                                color = (100, 100, 100)
                            elif i == 0:
                                color = (150, 150, 150)
                            else:
                                color = (0, 0, 0)
                            
                            pygame.draw.lines(background, color, False, points, 
                                            width=3 if num_lanes == 1 else 2)
            except:
                pass
        
        self.background = background
        
    def draw_vehicles(self):
        """绘制车辆"""
        # 复制背景
        self.screen.blit(self.background, (0, 0))
        
        # 获取所有车辆
        vehicle_ids = traci.vehicle.getIDList()
        
        for vehicle_id in vehicle_ids:
            try:
                pos = traci.vehicle.getPosition(vehicle_id)
                if pos:
                    screen_pos = self.world_to_screen(pos[0], pos[1])
                    self.screen.blit(self.drone_image, (screen_pos[0] - 10, screen_pos[1] - 10))

                    # self.draw_destination(vehicle_id)
            except:
                continue
        
        for drone in self.drones:
            self.draw_next_task_destination(drone)


        # 显示统计信息
        font = pygame.font.Font(None, 36)
        text = font.render(f'Vehicles: {len(vehicle_ids)}', True, (0, 0, 0))
        self.screen.blit(text, (10, 10))
        
        pygame.display.flip()

    def draw_destination(self, veh_id):
        """绘制目的地"""
        current_pos = traci.vehicle.getPosition(veh_id)
        current_pos_screen = self.world_to_screen(current_pos[0], current_pos[1])

        destination_route = traci.vehicle.getRoute(veh_id)[-1]
        lane_id = f"{destination_route}_0"
        destination_pos = traci.lane.getShape(lane_id)[-1]
        destination_pos_screen = self.world_to_screen(destination_pos[0], destination_pos[1])

        self.draw_dashed_line(self.screen, (255, 0, 0), destination_pos_screen, current_pos_screen, 2)

    def draw_next_task_destination(self, drone: Drone):
        if drone.is_free:
            return  # 无任务时不绘制目的地
        """绘制下一次任务的目的地"""
        next_destination = drone.get_current_task_destination()
        # print("Next destination:", next_destination)
        if next_destination:
            try:
                lane_id = f"{next_destination}_0"
                current_pos = drone.get_current_position()
                current_pos_screen = self.world_to_screen(current_pos[0], current_pos[1])
                next_pos = traci.lane.getShape(lane_id)[-1]
                next_pos_screen = self.world_to_screen(next_pos[0], next_pos[1])
                # print("Current pos:", current_pos_screen)
                # print("Next pos:", next_pos_screen)
                # 绘制到下一个任务目的地的虚线
                width = 5
                # pygame.draw.line(self.screen, (0, 255, 0), current_pos_screen, next_pos_screen, width)
                self.draw_dashed_line(self.screen, (255, 0, 0), current_pos_screen, next_pos_screen, width)
            except:
                print("Error:", lane_id)
                pass


    def draw_dashed_line(self, surface, color, start_pos, end_pos, width=1, dash_length=10):
        """绘制虚线（修正版）"""
        x1, y1 = start_pos
        x2, y2 = end_pos
        dl = dash_length

        dx = x2 - x1
        dy = y2 - y1
        distance = (dx*dx + dy*dy) ** 0.5
        if distance == 0:
            return

        # 单位方向向量
        dx_unit = dx / distance
        dy_unit = dy / distance

        # 浮点循环，步长为 2*dash_length
        pos = 0.0
        while pos < distance:
            # 当前段长度（最后一段可能不足）
            seg_len = min(dl, distance - pos)
            start_x = x2 - dx_unit * pos
            start_y = y2 - dy_unit * pos
            end_x = start_x - dx_unit * seg_len
            end_y = start_y - dy_unit * seg_len
            pygame.draw.line(surface, color, (start_x, start_y), (end_x, end_y), width)
            pos += 2 * dl   # 跳过空白段
        

    def render(self, drones:list[Drone]):
        # 防止ubuntu报错无响应
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                traci.close()
                pygame.quit()
                exit()

        self.drones = drones
        # 控制帧率
        self.draw_vehicles()
        self.clock.tick(self.fps)

    def run(self, steps=1000, fps=30):
        """运行仿真"""
        traci.start(["sumo-gui", "-c", self.sumo_cfg_path])
        traci.simulationStep()
        
        # 预渲染道路
        self.draw_roads()
        
        running = True
        step = 0
        
        while running and step < steps:
            # 处理事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
            
            # 执行仿真步
            traci.simulationStep()
            
            # 绘制车辆
            self.draw_vehicles()
            
            # 控制帧率
            self.clock.tick(fps)
            step += 1
        
        traci.close()
        pygame.quit()

# 使用
if __name__ == "__main__":
    viz = PygameVisualizer("data/siping/siping.sumocfg")
    viz.run(steps=1000, fps=30)