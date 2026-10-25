from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class WaypointNed:
    north_m: float
    east_m: float
    down_m: float


@dataclass(frozen=True)
class GpsDeniedMission:
    waypoints: tuple[WaypointNed,...]
    position_error_m_max: float


def load_gps_denied_mission(path: str|Path) -> GpsDeniedMission:
    payload=json.loads(Path(path).read_text(encoding="utf-8"))
    if payload.get("frame")!="NED" or payload.get("mode")!="external_vision":
        raise ValueError("mission must use NED external-vision mode")
    waypoints=tuple(WaypointNed(float(x["north_m"]),float(x["east_m"]),float(x["down_m"]))
                    for x in payload["waypoints"])
    tolerance=float(payload["acceptance"]["position_error_m_max"])
    if not waypoints or tolerance<=0:
        raise ValueError("mission requires waypoints and a positive tolerance")
    return GpsDeniedMission(waypoints,tolerance)


def ned_distance(position, waypoint: WaypointNed) -> float:
    return math.sqrt(
        (float(position[0])-waypoint.north_m)**2+
        (float(position[1])-waypoint.east_m)**2+
        (float(position[2])-waypoint.down_m)**2
    )
