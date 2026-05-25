import math
from config.config_loder import get_shared_config


class ChargingStation:
    """Simple charging station model."""

    def __init__(self, station_id, x, y, charging_power=50):
        self.station_id = station_id
        self.x = x
        self.y = y
        self.charging_power = charging_power

    def get_position(self):
        return (self.x, self.y)


def _distance(pos1, pos2):
    return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)


def build_default_charging_stations():
    """从配置文件构建充电站列表，支持新旧两种配置格式"""
    cfg = get_shared_config()

    # 新格式: "charging_stations" 数组
    stations_array = cfg.get("charging_stations", None)
    if stations_array:
        stations = []
        for s in stations_array:
            stations.append(ChargingStation(
                station_id=int(s.get("station_id", 0)),
                x=float(s.get("x", 356000.0)),
                y=float(s.get("y", 3463000.0)),
                charging_power=float(s.get("charging_power", 50)),
            ))
        return stations

    # 旧格式兼容: "charging_station" 单站
    old_cfg = cfg.get("charging_station", {})
    if old_cfg:
        return [ChargingStation(
            station_id=int(old_cfg.get("station_id", 0)),
            x=float(old_cfg.get("x", 356000.0)),
            y=float(old_cfg.get("y", 3463000.0)),
            charging_power=float(old_cfg.get("charging_power", 50)),
        )]

    # 兜底默认
    return [ChargingStation(station_id=0, x=356000.0, y=3463000.0, charging_power=50)]


def find_nearest_station(stations, position):
    """给定位置，找到最近的充电站"""
    if not stations:
        return None
    return min(stations, key=lambda s: _distance(position, s.get_position()))


DEFAULT_CHARGING_STATIONS = build_default_charging_stations()
# 向后兼容：DEFAULT_CHARGING_STATION 始终指向第一个充电站
DEFAULT_CHARGING_STATION = DEFAULT_CHARGING_STATIONS[0] if DEFAULT_CHARGING_STATIONS else None
