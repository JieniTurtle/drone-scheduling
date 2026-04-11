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
        Assigns a task to a drone, planning a route that avoids buildings.
        """
        start_pos = drone.get_position()
        end_pos = task.destination
        
        # Plan route avoiding buildings
        planned_route = self.plan_route_around_buildings(start_pos, end_pos)
        print("Planned route:", planned_route)
        # Schedule the route to the drone
        drone.schedule_route(planned_route)

    def plan_route_around_buildings(self, start_pos, end_pos):
        """
        Plans a route from start_pos to end_pos avoiding buildings using A* algorithm.
        """
        # Convert start and end positions to Points for geometric operations
        start_point = Point(start_pos)
        end_point = Point(end_pos)
        
        # Check if direct path is possible
        direct_path = LineString([start_pos, end_pos])
        collision = False
        
        for building in self.high_buildings:
            if building['geometry'].intersects(direct_path):
                collision = True
                break
        
        if not collision:
            # Direct path is clear
            return [end_pos]
        
        # Need to find path around buildings using A* algorithm
        path = self.a_star_pathfinding(start_pos, end_pos)
        
        if path:
            # Return the path excluding the starting position
            return path[1:]  # Exclude start position since drone is already there
        else:
            # If no path found, return direct destination (fallback)
            print("Warning: No obstacle-free path found, attempting direct route")
            return [end_pos]

    def a_star_pathfinding(self, start, goal):
        """
        Implements A* pathfinding algorithm considering building obstacles.
        """
        # Create a simplified grid-based representation for pathfinding
        # We'll use a visibility graph approach connecting start, goal, and building corners
        
        # Get all relevant points (start, goal, and building corner points)
        all_points = [start, goal]
        
        # Add building corner points to our visibility graph
        for building in self.high_buildings:
            geom = building['geometry']
            if hasattr(geom, 'exterior'):  # Polygon
                for coord in list(geom.exterior.coords)[:-1]:  # Exclude last point as it duplicates first
                    all_points.append(coord)
            elif hasattr(geom, 'coords'):  # Point or LineString
                for coord in geom.coords:
                    all_points.append(coord)
        
        # Remove duplicates
        all_points = list(set(all_points))
        
        # Implement A* algorithm
        open_set = [(0, start)]
        came_from = {}
        g_score = {point: float('inf') for point in all_points}
        g_score[start] = 0
        f_score = {point: float('inf') for point in all_points}
        f_score[start] = self.heuristic(start, goal)
        
        open_set_hash = {start}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            open_set_hash.remove(current)
            
            if current == goal:
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
                    tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                    
                    if tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                        
                        if neighbor not in open_set_hash:
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))
                            open_set_hash.add(neighbor)
        
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