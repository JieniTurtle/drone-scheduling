import osmnx as ox
import pygame
import numpy as np
from shapely.geometry import Point, LineString, Polygon, box
from shapely.strtree import STRtree
import math
from collections import defaultdict
from drone import Drone
from tools.osm import load_map_data

class OptimizedMapViewer:
    """优化的地图查看器，使用空间索引提高性能"""
    
    def __init__(self, osm_file_path, screen_size=(1200, 800)):
        pygame.init()
        self.clock = pygame.time.Clock()
        self.screen_size = screen_size
        self.screen = pygame.display.set_mode(screen_size)
        pygame.display.set_caption("OSM Map Viewer - Optimized")
        
        # 颜色方案
        self.COLORS = {
            'BACKGROUND': (255, 255, 255),
            'ROAD_HIGHWAY': (80, 80, 100),
            'ROAD_RESIDENTIAL': (60, 60, 80),
            'ROAD_PRIMARY': (100, 100, 120),
            'BUILDING': (150, 150, 200),
            'BUILDING_HOVER': (80, 80, 100),
            'WATER': (40, 60, 80),
            'GREEN': (40, 70, 40),
            'TEXT': (255, 255, 255),
            'SELECTION': (100, 150, 200, 128),
        }
        
        # 无人机颜色配置
        self.DRONE_COLORS = [
            (255, 80, 80),    # 红色
            (80, 80, 255),    # 蓝色
            (80, 200, 80),    # 绿色
            (255, 165, 0),    # 橙色
            (138, 43, 226),   # 紫色
            (0, 206, 209),    # 深青色
            (255, 105, 180),  # 粉红色
            (255, 215, 0),    # 金色
        ]
        
        # 特殊点颜色
        self.TASK_SOURCE_COLOR = (255, 255, 100)    # 任务起点 - 黄色方块
        self.TASK_DEST_COLOR = (255, 100, 255)     # 任务终点 - 粉色菱形
        self.WAYPOINT_COLOR = (100, 200, 255)      # 绕飞转弯点 - 天蓝色三角
        self.NEXT_TARGET_COLOR = (255, 255, 255)   # 下一个目标 - 白色大圆
        
        # 视图控制
        self.zoom = 1.0
        self.pan_x = 0
        self.pan_y = 0
        self.dragging = False
        self.drag_start = (0, 0)

        self.fps_counter = 0
        self.fps_time = 0
        self.current_fps = 0
        
        # 加载数据
        # self.load_map_data(osm_file_path)
        self.roads_by_type, self.buildings_with_height = load_map_data(osm_file_path)
        
        print(f"buildings_with_height: {self.buildings_with_height[0]}")
        
        # 构建空间索引
        self.build_spatial_index()
        
        # 计算边界
        self.calculate_bounds()

        self.scale = min(self.screen_size[0] / self.map_width, self.screen_size[1] / self.map_height)
        
        # 交互状态
        self.hovered_building = None
        self.selected_building = None
        
        self.font = pygame.font.Font(None, 20)
        self.small_font = pygame.font.Font(None, 16)
       
    def build_spatial_index(self):
        """构建空间索引以加速查询"""
        self.building_geoms = [b['geometry'] for b in self.buildings_with_height]
        if self.building_geoms:
            self.building_index = STRtree(self.building_geoms)
    
    def calculate_bounds(self):
        """计算边界"""
        all_geoms = []
        
        for roads in self.roads_by_type.values():
            all_geoms.extend(roads)
        
        all_geoms.extend(self.building_geoms)
        
        if all_geoms:
            bounds = all_geoms[0].bounds
            min_x, min_y, max_x, max_y = bounds
            
            for geom in all_geoms[1:]:
                b = geom.bounds
                min_x = min(min_x, b[0])
                min_y = min(min_y, b[1])
                max_x = max(max_x, b[2])
                max_y = max(max_y, b[3])
            
            self.min_x, self.min_y, self.max_x, self.max_y = min_x, min_y, max_x, max_y
        else:
            self.min_x, self.min_y, self.max_x, self.max_y = -180, -90, 180, 90
        
        self.map_width = self.max_x - self.min_x
        self.map_height = self.max_y - self.min_y

        print(f"Map bounds: ({self.min_x}, {self.min_y}) to ({self.max_x}, {self.max_y}), size: ({self.map_width}, {self.map_height})")
    
    def world_to_screen(self, x, y):
        """坐标转换"""
        # 使用统一的缩放比例
        screen_x = (x - self.min_x) * self.scale * self.zoom
        screen_y = (self.max_y - y) * self.scale * self.zoom
        screen_x += self.pan_x
        screen_y += self.pan_y
        return int(screen_x), int(screen_y)

    def get_visible_bounds(self):
        """获取当前可见区域的边界"""
        top_left = self.screen_to_world(0, 0)
        bottom_right = self.screen_to_world(self.screen_size[0], self.screen_size[1])
        return (top_left[0], bottom_right[1], bottom_right[0], top_left[1])

    def screen_to_world(self, screen_x, screen_y):
        """屏幕坐标转世界坐标"""
        # 使用统一的缩放比例
        unified_scale = min(self.screen_size[0], self.screen_size[1])
        world_x = (screen_x - self.pan_x) / self.zoom / self.scale  + self.min_x
        world_y = self.max_y - (screen_y - self.pan_y) / self.zoom / self.scale 
        return world_x, world_y
    
    def draw_road(self, road, road_type):
        """绘制道路"""
        if len(road.coords) < 2:
            return
        
        screen_points = []
        for coord in road.coords:
            screen_point = self.world_to_screen(coord[0], coord[1])
            screen_points.append(screen_point)
        
        # 根据道路类型设置样式
        if 'motorway' in road_type or 'primary' in road_type:
            color = self.COLORS['ROAD_PRIMARY']
            width = max(2, int(3 * self.zoom))
        elif 'residential' in road_type or 'living' in road_type:
            color = self.COLORS['ROAD_RESIDENTIAL']
            width = max(1, int(2 * self.zoom))
        else:
            color = self.COLORS['ROAD_HIGHWAY']
            width = max(1, int(1.5 * self.zoom))
        
        if len(screen_points) > 1:
            pygame.draw.lines(self.screen, color, False, screen_points, width)
    
    def draw_building(self, building_data, is_hovered=False, is_selected=False):
        """绘制建筑"""
        geom = building_data['geometry']
        
        if hasattr(geom, 'exterior'):
            screen_points = []
            for coord in geom.exterior.coords:
                screen_point = self.world_to_screen(coord[0], coord[1])
                screen_points.append(screen_point)
            
            if len(screen_points) > 2:
                # 根据状态选择颜色
                if is_selected:
                    color = (150, 200, 100)
                elif is_hovered:
                    color = self.COLORS['BUILDING_HOVER']
                else:
                    # 根据高度设置颜色
                    if building_data['height'] and building_data['height'] > 20:
                        color = (255, 0, 0)  # 红色 - 高建筑物
                    else:
                        color = self.COLORS['BUILDING']
                
                pygame.draw.polygon(self.screen, color, screen_points)
                pygame.draw.polygon(self.screen, (100, 100, 120), screen_points, 1)
                
    
    def find_hovered_building(self, mouse_pos):
        """查找鼠标悬停的建筑"""
        world_x, world_y = self.screen_to_world(mouse_pos[0], mouse_pos[1])
        point = Point(world_x, world_y)
        
        if self.building_geoms:
            # 使用空间索引查找
            indices = self.building_index.query(point)
            for idx in indices:
                if self.building_geoms[idx].contains(point):
                    return idx
        return None
    
    def draw_info_panel(self, drones=[]):
        """绘制信息面板"""
        info_lines = [
            f"Zoom: {self.zoom:.2f}x",
            f"Buildings: {len(self.buildings_with_height)}",
            f"With height: {sum(1 for b in self.buildings_with_height if b['height'])}",
            f"FPS: {self.current_fps}",
            "",
            "Drone Status:",
        ]
        
        # 添加无人机状态
        for drone in drones:
            drone_idx = int(drone.drone_id.split('_')[1]) if '_' in drone.drone_id else 0
            drone_color = self.DRONE_COLORS[drone_idx % len(self.DRONE_COLORS)]
            color_name = f"#{drone_color[0]:02X}{drone_color[1]:02X}{drone_color[2]:02X}"
            status = "IDLE" if drone.is_free else f"BUSY ({len(drone.scheduled_position)} pts)"
            info_lines.append(f"  {drone.drone_id}: {status}")
        
        info_lines.extend([
            "",
            "Controls:",
            "  Drag - Pan",
            "  Scroll - Zoom",
            "  Click - Select",
            "  R - Reset"
        ])
        
        if self.selected_building is not None:
            building = self.buildings_with_height[self.selected_building]
            info_lines.extend([
                "",
                "Selected Building:",
                f"  Height: {building['height'] or 'Unknown'}m"
            ])

            if 'id' in building and building['id'] is not None:
                info_lines.append(f"  ID: {building['id']}")
            
            tags = building['tags']
            if 'name' in tags:
                info_lines.append(f"  Name: {tags['name']}")
        
        panel_x = 10
        panel_y = 10
        
        # 绘制半透明背景
        max_width = 0
        for line in info_lines:
            text_surface = self.font.render(line, True, self.COLORS['TEXT'])
            max_width = max(max_width, text_surface.get_width())
        
        pygame.draw.rect(self.screen, (0, 0, 0, 200), 
                        (panel_x - 5, panel_y - 5, max_width + 10, len(info_lines) * 22 + 10))
        
        for i, line in enumerate(info_lines):
            text_surface = self.font.render(line, True, self.COLORS['TEXT'])
            self.screen.blit(text_surface, (panel_x, panel_y + i * 22))
    
    def handle_events(self):
        """处理事件"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # 左键
                    # 记录左键点击在地图上的坐标并追加保存到文件
                    world_pos = self.screen_to_world(event.pos[0], event.pos[1])
                    self.save_click_position(world_pos)

                    # 检查是否点击了建筑
                    hover_idx = self.find_hovered_building(event.pos)
                    if hover_idx is not None:
                        self.selected_building = hover_idx
                    else:
                        # 开始拖动
                        self.dragging = True
                        self.drag_start = event.pos
                elif event.button == 4:  # 放大
                    # 以鼠标位置为中心缩放
                    mouse_world = self.screen_to_world(event.pos[0], event.pos[1])
                    self.zoom *= 1.1
                    self.zoom = min(self.zoom, 10.0)
                    # 调整平移使鼠标位置保持不变
                    new_mouse_screen = self.world_to_screen(mouse_world[0], mouse_world[1])
                    self.pan_x += event.pos[0] - new_mouse_screen[0]
                    self.pan_y += event.pos[1] - new_mouse_screen[1]
                elif event.button == 5:  # 缩小
                    mouse_world = self.screen_to_world(event.pos[0], event.pos[1])
                    self.zoom /= 1.1
                    self.zoom = max(self.zoom, 0.05)
                    new_mouse_screen = self.world_to_screen(mouse_world[0], mouse_world[1])
                    self.pan_x += event.pos[0] - new_mouse_screen[0]
                    self.pan_y += event.pos[1] - new_mouse_screen[1]
            
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.dragging = False
            
            elif event.type == pygame.MOUSEMOTION:
                if self.dragging:
                    dx = event.pos[0] - self.drag_start[0]
                    dy = event.pos[1] - self.drag_start[1]
                    self.pan_x += dx
                    self.pan_y += dy
                    self.drag_start = event.pos
                else:
                    # 更新悬停的建筑
                    hover_idx = self.find_hovered_building(event.pos)
                    self.hovered_building = hover_idx
            
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    self.zoom = 1.0
                    self.pan_x = 0
                    self.pan_y = 0
                    self.selected_building = None
        
        return True

    def save_click_position(self, world_pos):
        """将地图坐标追加写入文件。"""
        try:
            with open('clicked_positions.txt', 'a', encoding='utf-8') as f:
                f.write(f"{world_pos[0]:.6f},{world_pos[1]:.6f}\n")
        except Exception as e:
            print(f"Failed to save click position: {e}")
    
    def draw(self, drones=[]):
        """主绘制函数"""
        self.screen.fill(self.COLORS['BACKGROUND'])

        # 获取可见区域
        visible_bounds = self.get_visible_bounds()
        
        # 绘制道路
        for road_type, roads in self.roads_by_type.items():
            for road in roads:
                if road.bounds[2] >= visible_bounds[0] and road.bounds[0] <= visible_bounds[2] and \
                   road.bounds[3] >= visible_bounds[1] and road.bounds[1] <= visible_bounds[3]:
                    self.draw_road(road, road_type)
        
        # 绘制建筑
        for idx, building in enumerate(self.buildings_with_height):
            geom = building['geometry']
            if geom.bounds[2] >= visible_bounds[0] and geom.bounds[0] <= visible_bounds[2] and \
               geom.bounds[3] >= visible_bounds[1] and geom.bounds[1] <= visible_bounds[3]:
                is_hovered = (self.hovered_building == idx)
                is_selected = (self.selected_building == idx)
                self.draw_building(building, is_hovered, is_selected)

        # 绘制所有无人机的路线（先画路线，再画无人机）
        for drone in drones:
            drone_idx = int(drone.drone_id.split('_')[1]) if '_' in drone.drone_id else 0
            drone_color = self.DRONE_COLORS[drone_idx % len(self.DRONE_COLORS)]
            
            if drone.scheduled_position:
                self.draw_drone_route(drone, drone_color)
        
        # 绘制无人机和任务点
        for drone in drones:
            drone_idx = int(drone.drone_id.split('_')[1]) if '_' in drone.drone_id else 0
            drone_color = self.DRONE_COLORS[drone_idx % len(self.DRONE_COLORS)]
            self.draw_drone(drone, drone_color)
        
        # 绘制图例
        self.draw_legend()
        
        # 绘制UI
        self.draw_info_panel(drones)
        
        pygame.display.flip()
    
    def draw_drone_route(self, drone, drone_color):
        """绘制无人机的路线，区分不同类型的航点"""
        route = drone.scheduled_position
        
        # 检查路线格式：是否包含类型信息 (x, y, type)
        has_types = len(route) > 0 and len(route[0]) >= 3
        
        # 绘制路线线段（半透明）
        for i in range(len(route) - 1):
            if has_types:
                start_screen = self.world_to_screen(route[i][0], route[i][1])
                end_screen = self.world_to_screen(route[i+1][0], route[i+1][1])
            else:
                start_screen = self.world_to_screen(route[i][0], route[i][1])
                end_screen = self.world_to_screen(route[i+1][0], route[i+1][1])
            
            # 绘制带箭头的线段
            self.draw_arrow_line(start_screen, end_screen, drone_color, width=3, alpha=180)
        
        # 根据类型绘制航点标记
        for i, point in enumerate(route):
            if has_types:
                x, y, point_type = point
                screen_pos = self.world_to_screen(x, y)
                
                if point_type == 'source':
                    # 任务起点 - 黄色方块
                    square_size = max(5, int(4 * self.zoom))
                    rect = pygame.Rect(screen_pos[0] - square_size, screen_pos[1] - square_size, 
                                       square_size * 2, square_size * 2)
                    pygame.draw.rect(self.screen, self.TASK_SOURCE_COLOR, rect)
                    pygame.draw.rect(self.screen, (100, 100, 0), rect, 1)
                elif point_type == 'dest':
                    # 任务终点 - 粉色菱形
                    self.draw_diamond(screen_pos, self.TASK_DEST_COLOR, max(6, int(5 * self.zoom)))
                elif point_type == 'waypoint':
                    # 绕飞航点 - 天蓝色三角形
                    self.draw_triangle(screen_pos, self.WAYPOINT_COLOR, size=max(4, int(3 * self.zoom)))
            else:
                # 旧格式（无类型信息）：第一个是 source，最后一个是 dest，中间是 waypoint
                screen_pos = self.world_to_screen(point[0], point[1])
                if i == 0:
                    square_size = max(5, int(4 * self.zoom))
                    rect = pygame.Rect(screen_pos[0] - square_size, screen_pos[1] - square_size, 
                                       square_size * 2, square_size * 2)
                    pygame.draw.rect(self.screen, self.TASK_SOURCE_COLOR, rect)
                    pygame.draw.rect(self.screen, (100, 100, 0), rect, 1)
                elif i == len(route) - 1:
                    self.draw_diamond(screen_pos, self.TASK_DEST_COLOR, max(6, int(5 * self.zoom)))
                elif len(route) > 2:
                    self.draw_triangle(screen_pos, self.WAYPOINT_COLOR, size=max(4, int(3 * self.zoom)))
    
    def draw_arrow_line(self, start, end, color, width=2, alpha=255):
        """绘制带箭头的线段"""
        # 创建带透明度的表面
        pygame.draw.line(self.screen, color, start, end, width)
        
        # 计算箭头方向
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = (dx*dx + dy*dy)**0.5
        if length < 20:
            return
        
        # 箭头大小
        arrow_size = max(5, int(4 * self.zoom))
        
        # 计算垂直方向
        nx = -dy / length
        ny = dx / length
        
        # 箭头位置（在终点附近）
        ax = end[0] - dx * 0.2
        ay = end[1] - dy * 0.2
        
        # 绘制箭头
        arrow_points = [
            (end[0], end[1]),
            (int(ax - nx * arrow_size), int(ay - ny * arrow_size)),
            (int(ax + nx * arrow_size), int(ay + ny * arrow_size))
        ]
        pygame.draw.polygon(self.screen, color, arrow_points)
    
    def draw_triangle(self, center, color, size=5):
        """绘制三角形（绕飞点标记）"""
        points = [
            (center[0], center[1] - size),
            (center[0] - size * 0.866, center[1] + size * 0.5),
            (center[0] + size * 0.866, center[1] + size * 0.5)
        ]
        pygame.draw.polygon(self.screen, color, points)
        pygame.draw.polygon(self.screen, (50, 50, 50), points, 1)
    
    def draw_diamond(self, center, color, size=6):
        """绘制菱形（终点标记）"""
        points = [
            (center[0], center[1] - size),
            (center[0] + size, center[1]),
            (center[0], center[1] + size),
            (center[0] - size, center[1])
        ]
        pygame.draw.polygon(self.screen, color, points)
        pygame.draw.polygon(self.screen, (50, 50, 50), points, 1)
    
    def draw_drone(self, drone, drone_color):
        """绘制无人机"""
        x, y = drone.get_position()
        screen_pos = self.world_to_screen(x, y)
        
        # 绘制无人机
        drone_size = max(8, int(6 * self.zoom))
        
        if drone.is_free:
            # 空闲状态：绿色实心圆形
            pygame.draw.circle(self.screen, (50, 200, 50), screen_pos, drone_size + 2)
            pygame.draw.circle(self.screen, (100, 255, 100), screen_pos, drone_size)
        else:
            # 执行任务：对应颜色的圆形，带白色边框
            pygame.draw.circle(self.screen, drone_color, screen_pos, drone_size + 3)
            pygame.draw.circle(self.screen, (255, 255, 255), screen_pos, drone_size + 1)
            pygame.draw.circle(self.screen, drone_color, screen_pos, drone_size)
        
        # 绘制无人机ID
        id_text = self.small_font.render(drone.drone_id, True, (0, 0, 0))
        self.screen.blit(id_text, (screen_pos[0] + 10, screen_pos[1] - 8))
    
    def draw_legend(self):
        """绘制图例"""
        legend_items = [
            ("Drone (IDLE)", (50, 200, 50)),
            ("Drone (BUSY)", (255, 80, 80)),
            ("Task Source", self.TASK_SOURCE_COLOR),
            ("Task Dest", self.TASK_DEST_COLOR),
            ("Waypoint", self.WAYPOINT_COLOR),
        ]
        
        # 图例背景
        legend_x = self.screen_size[0] - 180
        legend_y = 10
        item_height = 22
        bg_height = len(legend_items) * item_height + 15
        
        pygame.draw.rect(self.screen, (220, 220, 220), 
                        (legend_x - 10, legend_y - 5, 170, bg_height))
        pygame.draw.rect(self.screen, (100, 100, 100), 
                        (legend_x - 10, legend_y - 5, 170, bg_height), 1)
        
        for i, (text, color) in enumerate(legend_items):
            y = legend_y + i * item_height
            if "Drone" in text:
                pygame.draw.circle(self.screen, color, (legend_x, y + 6), 5)
            elif "Source" in text:
                pygame.draw.rect(self.screen, color, (legend_x - 5, y + 1, 10, 10))
            elif "Dest" in text:
                self.draw_diamond((legend_x, y + 6), color, 5)
            elif "Waypoint" in text:
                self.draw_triangle((legend_x, y + 6), color, 5)
            
            label = self.small_font.render(text, True, (0, 0, 0))
            self.screen.blit(label, (legend_x + 12, y))
    
    def render(self, drones):
        running = self.handle_events()
        self.draw(drones)

        # FPS calculation
        current_time = pygame.time.get_ticks()
        self.fps_counter += 1
        
        if current_time - self.fps_time > 1000:  # Update every second
            self.current_fps = self.fps_counter
            self.fps_counter = 0
            self.fps_time = current_time
            print(f"FPS: {self.current_fps}")
        
        self.clock.tick(60)


    def run(self):
        """主循环"""
        # clock = pygame.time.Clock()
        running = True
        
        while running:
            running = self.handle_events()
            self.draw()
            self.clock.tick(60)
        
        pygame.quit()

# 使用示例
if __name__ == "__main__":
    osm_file_path = "data/map/map.osm"
    
    try:
        pygame.init()
        viewer = OptimizedMapViewer(osm_file_path)
        viewer.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()