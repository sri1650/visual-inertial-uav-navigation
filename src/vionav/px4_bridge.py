from __future__ import annotations
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ExternalVisionSample:
    timestamp_us: int
    position_ned_m: tuple[float,float,float]
    velocity_ned_m_s: tuple[float,float,float]
    position_variance_m2: tuple[float,float,float]
    velocity_variance_m2_s2: tuple[float,float,float]
    reset_counter: int
    quality: float


def enu_to_ned(v):
    x_east,y_north,z_up=(float(v[0]),float(v[1]),float(v[2]))
    return (y_north,x_east,-z_up)


def ned_to_enu(v):
    x_north,y_east,z_down=(float(v[0]),float(v[1]),float(v[2]))
    return (y_east,x_north,-z_down)


def make_external_vision_sample(
    *,
    timestamp_ns: int,
    position_enu_m,
    velocity_enu_m_s,
    position_variance_m2=(.01,.01,.02),
    velocity_variance_m2_s2=(.04,.04,.06),
    reset_counter: int=0,
    quality: float=1.0,
) -> ExternalVisionSample:
    if timestamp_ns < 0:
        raise ValueError("timestamp_ns cannot be negative")
    if not 0.0 <= quality <= 1.0:
        raise ValueError("quality must be within [0,1]")
    if reset_counter < 0:
        raise ValueError("reset_counter cannot be negative")
    vals=(*position_enu_m,*velocity_enu_m_s,*position_variance_m2,*velocity_variance_m2_s2)
    if not all(math.isfinite(float(x)) for x in vals):
        raise ValueError("external vision sample contains non-finite values")
    if any(float(x)<0 for x in (*position_variance_m2,*velocity_variance_m2_s2)):
        raise ValueError("variances cannot be negative")
    return ExternalVisionSample(
        timestamp_us=timestamp_ns//1000,
        position_ned_m=enu_to_ned(position_enu_m),
        velocity_ned_m_s=enu_to_ned(velocity_enu_m_s),
        position_variance_m2=tuple(float(x) for x in position_variance_m2),
        velocity_variance_m2_s2=tuple(float(x) for x in velocity_variance_m2_s2),
        reset_counter=reset_counter,
        quality=quality,
    )
