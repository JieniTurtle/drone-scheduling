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


def build_default_charging_station():
    cfg = get_shared_config().get("charging_station", {})
    return ChargingStation(
        station_id=int(cfg.get("station_id", 0)),
        x=float(cfg.get("x", 356000.0)),
        y=float(cfg.get("y", 3463000.0)),
        charging_power=float(cfg.get("charging_power", 50)),
    )


DEFAULT_CHARGING_STATION = build_default_charging_station()
