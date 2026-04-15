import osmnx as ox
import numpy as np
import heapq
from shapely.geometry import Point, LineString
import math
from drone import Drone
from task import Task
# from test import OptimizedMapViewer
from tools.osm import load_map_data, get_building_location_by_name, get_global_bounds

class Environment:
    def __init__(self, osm_file_path):
        _, buildings_with_height = load_map_data(osm_file_path)

        self.global_bounds = get_global_bounds(buildings_with_height)

        self.high_buildings = [b for b in buildings_with_height if b['height'] is not None and b['height'] > 20]

        print(get_building_location_by_name( self.high_buildings, "衷和楼"))
        print(f"加载了 {len(self.high_buildings)} 个具有高度信息的建筑物")

    def get_global_bounds(self):
        return self.global_bounds
    
    def get_high_buildings(self):
        return self.high_buildings

    def update(self):
        for drone in self.drones:
            drone.update()

    
    def assign_task(self, drone, task):
        """
        Assigns a single task to a drone, planning a route that avoids buildings.
        """
        start_pos = drone.get_position()
        end_pos = task.get_destination()
        
        # Plan route avoiding buildings
        planned_route = self.plan_route_around_buildings(start_pos, end_pos)
        print(f"Planned route for {task.task_id}: {planned_route}")
        # Schedule the route to the drone
        drone.schedule_route(planned_route)
    
    def plan_route_for_tasks(self, drone, tasks):
        """
        为无人机规划多个任务的路线，确保绕过建筑物。
        返回完整的绕行路线，每个点格式为 (x, y, type)。
        type: 'source' = 任务起点, 'dest' = 任务终点, 'waypoint' = 绕飞航点
        """
        if not tasks:
            return []
        
        full_route = []  # 格式: [(x, y, type), ...]
        current_pos = drone.get_position()
        
        for task in tasks:
            # 规划从当前位置到任务起始点的路径
            source = task.get_source()
            dest = task.get_destination()
            
            # 第一步：当前位置 → 起始点（绕行）
            source_route = self.plan_route_around_buildings(current_pos, source)
            for point in source_route:
                # 最后一个点是 source，其他是 waypoint
                if len(source_route) > 1:
                    if point == source_route[-1]:
                        full_route.append((point[0], point[1], 'source'))
                    else:
                        full_route.append((point[0], point[1], 'waypoint'))
                else:
                    full_route.append((point[0], point[1], 'source'))
            
            # 第二步：起始点 → 终点（绕行）
            dest_route = self.plan_route_around_buildings(source, dest)
            for point in dest_route:
                if len(dest_route) > 1:
                    if point == dest_route[-1]:
                        full_route.append((point[0], point[1], 'dest'))
                    else:
                        full_route.append((point[0], point[1], 'waypoint'))
                else:
                    full_route.append((point[0], point[1], 'dest'))
            
            current_pos = dest
        
        print(f"Full planned route for drone {drone.drone_id}: {full_route}")
        
        # 验证整个路线是否与建筑物相交
        self.verify_route_with_types(full_route)
        
        return full_route
    
    def verify_route_with_types(self, route):
        """验证路线是否与建筑物相交"""
        for i in range(len(route) - 1):
            if not self.is_path_clear(route[i][:2], route[i+1][:2]):
                print(f"WARNING: Route segment {route[i][:2]} -> {route[i+1][:2]} intersects with building!")
                return False
        return True
    
    def verify_route(self, route):
        """验证路线是否与建筑物相交"""
        for i in range(len(route) - 1):
            if not self.is_path_clear(route[i], route[i+1]):
                print(f"WARNING: Route segment {route[i]} -> {route[i+1]} intersects with building!")
                return False
        return True

    def plan_route_around_buildings(self, start_pos, end_pos):
        """
        Plans a route from start_pos to end_pos avoiding buildings using A* algorithm.
        """
        # Check if direct path is possible (stricter check)
        if self.is_path_clear(start_pos, end_pos):
            return [end_pos]
        
        # Try A* pathfinding
        path = self.a_star_pathfinding(start_pos, end_pos)
        
        if path and len(path) > 1:
            # Return path excluding starting position
            return path[1:]
        
        # A* failed, try to find waypoints around buildings manually
        waypoints = self.find_waypoints_around_buildings(start_pos, end_pos)
        if waypoints:
            return waypoints
        
        # Last resort: return direct destination with warning
        print(f"Warning: Could not find obstacle-free path from {start_pos} to {end_pos}")
        return [end_pos]
    
    def find_waypoints_around_buildings(self, start_pos, end_pos):
        """
        当A*算法失败时，尝试手动找到绕过建筑物的航点。
        """
        # 找到所有与直线相交的建筑物
        intersecting_buildings = []
        direct_line = LineString([start_pos, end_pos])
        
        for building in self.high_buildings:
            if building['geometry'].intersects(direct_line):
                intersecting_buildings.append(building)
        
        if not intersecting_buildings:
            return [end_pos]
        
        # 对每个相交的建筑物，找到绕行的航点
        waypoints = []
        for building in intersecting_buildings:
            # 获取建筑物的边界框
            bounds = building['geometry'].bounds  # (minx, miny, maxx, maxy)
            
            # 计算从起点到终点的方向
            dx = end_pos[0] - start_pos[0]
            dy = end_pos[1] - start_pos[1]
            
            # 找到建筑物在方向上的哪一侧，然后选择绕行的角点
            center_x = (bounds[0] + bounds[2]) / 2
            center_y = (bounds[1] + bounds[3]) / 2
            
            # 计算偏移量（绕开建筑物）
            offset = 50  # 绕行距离
            
            # 选择绕行方向
            if abs(dx) > abs(dy):
                # 主要水平移动，垂直绕行
                if start_pos[0] < center_x:
                    # 从建筑下方绕行
                    waypoint = (center_x - offset, bounds[3] + offset)
                else:
                    # 从建筑上方绕行
                    waypoint = (center_x + offset, bounds[1] - offset)
            else:
                # 主要垂直移动，水平绕行
                if start_pos[1] < center_y:
                    # 从建筑左侧绕行
                    waypoint = (bounds[2] + offset, center_y - offset)
                else:
                    # 从建筑右侧绕行
                    waypoint = (bounds[0] - offset, center_y + offset)
            
            # 确保航点不在建筑物内部
            waypoint = self.adjust_waypoint_outside_buildings(waypoint)
            waypoints.append(waypoint)
        
        # 添加终点
        waypoints.append(end_pos)
        
        # 验证路线是否通畅
        full_route = [start_pos] + waypoints
        if self.verify_route(full_route):
            return waypoints
        
        return [end_pos]
    
    def adjust_waypoint_outside_buildings(self, waypoint):
        """调整航点，确保它在所有建筑物外部"""
        point = Point(waypoint)
        min_distance = 10  # 最小安全距离
        
        for building in self.high_buildings:
            geom = building['geometry']
            dist = geom.exterior.distance(point)
            if dist < min_distance:
                # 需要将航点移出建筑物
                # 简单地增加偏移量
                bounds = geom.bounds
                center_x = (bounds[0] + bounds[2]) / 2
                center_y = (bounds[1] + bounds[3]) / 2
                
                # 向远离建筑物的方向移动
                dx = waypoint[0] - center_x
                dy = waypoint[1] - center_y
                dist = (dx*dx + dy*dy)**0.5
                if dist > 0:
                    waypoint = (
                        waypoint[0] + (dx / dist) * (min_distance - dist + 10),
                        waypoint[1] + (dy / dist) * (min_distance - dist + 10)
                    )
        
        return waypoint

    def a_star_pathfinding(self, start, goal):
        """
        Implements A* pathfinding algorithm considering building obstacles.
        Uses visibility graph approach.
        """
        # Round positions to avoid floating point precision issues
        def round_pos(pos, decimals=2):
            return (round(pos[0], decimals), round(pos[1], decimals))
        
        start = round_pos(start)
        goal = round_pos(goal)
        
        # Get all relevant points (start, goal, and building corner points)
        all_points = [start, goal]
        
        # Add building corner points to our visibility graph
        for building in self.high_buildings:
            geom = building['geometry']
            if hasattr(geom, 'exterior'):  # Polygon
                for coord in list(geom.exterior.coords)[:-1]:
                    rounded_coord = round_pos(coord)
                    if rounded_coord != start and rounded_coord != goal:
                        all_points.append(rounded_coord)
            elif hasattr(geom, 'coords'):  # Point or LineString
                for coord in geom.coords:
                    rounded_coord = round_pos(coord)
                    if rounded_coord != start and rounded_coord != goal:
                        all_points.append(rounded_coord)
        
        # Remove duplicates
        all_points = list(set(all_points))
        
        # Create dictionaries with rounded keys for consistent lookup
        point_to_key = {p: p for p in all_points}
        
        # Implement A* algorithm
        open_set = [(0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        open_set_set = {start}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            open_set_set.remove(current)
            
            if self.heuristic(current, goal) < 1:  # Close enough to goal
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            for neighbor in all_points:
                if neighbor == current:
                    continue
                    
                # Check if path from current to neighbor intersects any buildings
                if self.is_path_clear(current, neighbor):
                    tentative_g = g_score.get(current, float('inf')) + self.heuristic(current, neighbor)
                    
                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                        
                        if neighbor not in open_set_set:
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))
                            open_set_set.add(neighbor)
        
        # No path found
        return None

    def heuristic(self, pos1, pos2):
        """
        Calculates Euclidean distance between two points.
        """
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)

    def is_path_clear(self, pos1, pos2):
        """
        Checks if the path between two positions intersects any buildings.
        """
        line = LineString([pos1, pos2])
        
        for building in self.high_buildings:
            if building['geometry'].intersects(line):
                # Check if intersection is significant (not just touching corners)
                if building['geometry'].intersection(line).length > 0.1:  # Small threshold to avoid floating point errors
                    return False
        
        return True

if __name__ == "__main__":
    env = Environment('data/map/map.osm')

    env.drones = [Drone(357600.574872369, 3462308.772003661)]

    env.assign_task(env.drones[0], Task(1, 1, (357400.574872369, 3462308.772003661)))

    viewer = OptimizedMapViewer('data/map/map.osm')
    for i in range(100):
        viewer.render(env.drones)
    # print(env.buildings_with_height[0]['geometry'])
    while True:
        env.update()
        viewer.render(env.drones)