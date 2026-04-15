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
        # print(f"buildings_with_height: {self.buildings_with_height[0]}")
        
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
    
    def draw_info_panel(self):
        """绘制信息面板"""
        info_lines = [
            f"Zoom: {self.zoom:.2f}x",
            f"Buildings: {len(self.buildings_with_height)}",
            f"With height: {sum(1 for b in self.buildings_with_height if b['height'])}",
            f"FPS: {self.current_fps}",
            "",
            "Controls:",
            "  Drag - Pan",
            "  Scroll - Zoom",
            "  Click - Select building",
            "  R - Reset"
        ]
        
        if self.selected_building is not None:
            building = self.buildings_with_height[self.selected_building]
            info_lines.extend([
                "",
                "Selected Building:",
                f"  Height: {building['height'] or 'Unknown'}m"
            ])
            
            # 添加更多标签信息
            tags = building['tags']
            if 'name' in tags:
                info_lines.append(f"  Name: {tags['name']}")
            if 'building' in tags:
                info_lines.append(f"  Type: {tags['building']}")
        
        panel_x = 10
        panel_y = 10
        
        # 绘制半透明背景
        max_width = 0
        for line in info_lines:
            text_surface = self.font.render(line, True, self.COLORS['TEXT'])
            max_width = max(max_width, text_surface.get_width())
        
        pygame.draw.rect(self.screen, (0, 0, 0, 200), 
                        (panel_x - 5, panel_y - 5, max_width + 10, len(info_lines) * 25 + 10))
        
        for i, line in enumerate(info_lines):
            text_surface = self.font.render(line, True, self.COLORS['TEXT'])
            self.screen.blit(text_surface, (panel_x, panel_y + i * 25))
    
    def handle_events(self):
        """处理事件"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # 左键
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
    
    def draw(self, drones=[]):
        """主绘制函数"""
        self.screen.fill(self.COLORS['BACKGROUND'])
        
            

        # 获取可见区域
        visible_bounds = self.get_visible_bounds()
        
        # 绘制道路（简化：绘制所有，因为道路通常不多）
        for road_type, roads in self.roads_by_type.items():
            for road in roads:
                # 简单的视锥剔除
                if road.bounds[2] >= visible_bounds[0] and road.bounds[0] <= visible_bounds[2] and \
                   road.bounds[3] >= visible_bounds[1] and road.bounds[1] <= visible_bounds[3]:
                    self.draw_road(road, road_type)
        
        # 绘制建筑
        for idx, building in enumerate(self.buildings_with_height):
            geom = building['geometry']
            # 视锥剔除
            if geom.bounds[2] >= visible_bounds[0] and geom.bounds[0] <= visible_bounds[2] and \
               geom.bounds[3] >= visible_bounds[1] and geom.bounds[1] <= visible_bounds[3]:
                is_hovered = (self.hovered_building == idx)
                is_selected = (self.selected_building == idx)
                self.draw_building(building, is_hovered, is_selected)

        
        for drone in drones:
            if drone.is_free == False:
                x, y = drone.get_position()
                screen_pos = self.world_to_screen(x, y)
                pygame.draw.circle(self.screen, (0, 0, 255), screen_pos, max(5, int(3 * self.zoom)))
                print("Drone position: ({}, {})".format(x, y))

                # Draw the route to the next target points if available
                if drone.scheduled_position:  # Check if there are scheduled positions
                    route_points = [screen_pos]  # Start from current drone position
                    
                    # Add the next target positions to the route
                    for target_pos in drone.scheduled_position:
                        target_screen_pos = self.world_to_screen(target_pos[0], target_pos[1])
                        route_points.append(target_screen_pos)
                    
                    # Draw the route line
                    if len(route_points) > 1:
                        pygame.draw.lines(self.screen, (100, 100, 255), False, route_points, 2)  # Light blue route line
                        
                        # Optionally, highlight the next target point
                        if len(drone.scheduled_position) > 0:
                            next_target = drone.scheduled_position[0]
                            next_target_screen = self.world_to_screen(next_target[0], next_target[1])
                            pygame.draw.circle(self.screen, (255, 255, 0), next_target_screen, max(3, int(2 * self.zoom)))  # Yellow circle for next target
        
        # 绘制UI
        self.draw_info_panel()
        
        pygame.display.flip()
    
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