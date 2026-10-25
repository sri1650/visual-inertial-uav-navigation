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


@dataclass(frozen=True)
class EstimatorLossPolicy:
    degraded_max_duration_ms: int
    lost_action: str
    external_vision_allowed_after_loss: bool
    estimator_timeout_ms: int


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


def load_estimator_loss_policy(path: str|Path) -> EstimatorLossPolicy:
    payload=json.loads(Path(path).read_text(encoding="utf-8"))["external_vision"]
    policy=EstimatorLossPolicy(
        degraded_max_duration_ms=int(payload["degraded"]["max_duration_ms"]),
        lost_action=str(payload["lost"]["action"]),
        external_vision_allowed_after_loss=bool(payload["lost"]["external_vision_allowed"]),
        estimator_timeout_ms=int(payload["watchdog"]["estimator_timeout_ms"]),
    )
    if policy.degraded_max_duration_ms<=0 or policy.estimator_timeout_ms<=0:
        raise ValueError("failsafe timeouts must be positive")
    if policy.lost_action!="request_position_hold_or_land" or policy.external_vision_allowed_after_loss:
        raise ValueError("estimator-loss policy must reject external vision and request hold or land")
    return policy
