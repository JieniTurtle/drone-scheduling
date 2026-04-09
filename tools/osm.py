import osmnx as ox
import pygame
import numpy as np
from shapely.geometry import Point, LineString, Polygon, box
from shapely.strtree import STRtree
import math
from collections import defaultdict
from drone import Drone    

def load_map_data(osm_file_path):
    """加载地图数据"""
    print("Loading map data...")
    # 读取路网 relations
    graph = ox.graph_from_xml(osm_file_path)
    # 将经纬度坐标投影为米制坐标
    graph = ox.project_graph(graph)

    # 读取建筑 - 使用新的 API，并投影到相同 CRS
    buildings = ox.features_from_xml(osm_file_path, tags={'building': True})
    if hasattr(buildings, 'to_crs') and graph.graph.get('crs') is not None:
        buildings = buildings.to_crs(graph.graph['crs'])
    
    # 提取道路并分类
    roads_by_type = defaultdict(list)
    for u, v, data in graph.edges(data=True):
        road_type = data.get('highway', 'residential')
        
        if isinstance(road_type, list):  # 修复：处理可能的列表类型
            road_type = road_type[0] if road_type else 'residential'
        
        if 'geometry' in data:
            geom = data['geometry']
        else:
            node1 = graph.nodes[u]
            node2 = graph.nodes[v]
            geom = LineString([(node1['x'], node1['y']), (node2['x'], node2['y'])])
        
        roads_by_type[road_type].append(geom)
    
    # 提取建筑高度信息
    buildings_with_height = []
    for idx, building in buildings.iterrows():
        height = building.get('height', None)
        levels = building.get('building:levels', None)
        
        if height:
            try:
                height_val = float(height.split()[0]) if isinstance(height, str) else float(height)
            except:
                height_val = None
        elif levels:
            try:
                height_val = float(levels) * 3.0  # 按每层3米估算
            except:
                height_val = None
        else:
            height_val = None
        
        buildings_with_height.append({
            'geometry': building.geometry,
            'height': height_val,
            'tags': building
        })
    
    ubidings_with_height = buildings_with_height
    print(f"Loaded: {len(roads_by_type)} road types, {len(ubidings_with_height)} buildings")
    print(f"Buildings with height info: {sum(1 for b in ubidings_with_height if b['height'])}")
    return roads_by_type, ubidings_with_height


def get_building_location_by_name(buildings_with_height, name, exact=True):
    """根据建筑名查找建筑位置。

    :param buildings_with_height: load_map_data 返回的建筑列表
    :param name: 建筑名
    :param exact: 是否完全匹配，False 则使用包含匹配
    :return: 如果找到，返回一个坐标元组 (x, y) 或坐标元组列表；未找到返回 None
    """
    if not name:
        return None

    query = name.strip().lower()
    matches = []
    for building in buildings_with_height:
        tags = building.get('tags', {})
        if tags is None:
            continue

        # 常见建筑名字段
        for field in ('name', 'building:name', 'addr:housename'):
            value = tags.get(field)
            if not value or not isinstance(value, str):
                continue

            value_norm = value.strip().lower()
            if exact:
                if value_norm == query:
                    matches.append(building)
                    break
            else:
                if query in value_norm:
                    matches.append(building)
                    break

    if not matches:
        return None

    locations = []
    for building in matches:
        geom = building.get('geometry')
        if geom is None:
            continue
        centroid = geom.centroid
        locations.append((centroid.x, centroid.y))

    if not locations:
        return None
    return locations[0] if len(locations) == 1 else locations

    