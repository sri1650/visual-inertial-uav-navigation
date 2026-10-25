from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Any


@dataclass(frozen=True)
class ExternalVisionSample:
    timestamp_us: int
    position_ned_m: tuple[float,float,float]
    velocity_ned_m_s: tuple[float,float,float]
    position_variance_m2: tuple[float,float,float]
    velocity_variance_m2_s2: tuple[float,float,float]
    reset_counter: int
    quality: float
    quaternion_ned_frd_wxyz: tuple[float,float,float,float]=(1.0,0.0,0.0,0.0)
    angular_velocity_frd_rad_s: tuple[float,float,float]=(0.0,0.0,0.0)
    orientation_variance_rad2: tuple[float,float,float]=(.01,.01,.02)
    angular_velocity_variance_rad2_s2: tuple[float,float,float]=(.04,.04,.04)


def enu_to_ned(v):
    x_east,y_north,z_up=(float(v[0]),float(v[1]),float(v[2]))
    return (y_north,x_east,-z_up)


def ned_to_enu(v):
    x_north,y_east,z_down=(float(v[0]),float(v[1]),float(v[2]))
    return (y_east,x_north,-z_down)


def enu_variance_to_ned(v):
    east,north,up=(float(v[0]),float(v[1]),float(v[2]))
    return (north,east,up)


def flu_to_frd(v):
    x_forward,y_left,z_up=(float(v[0]),float(v[1]),float(v[2]))
    return (x_forward,-y_left,-z_up)


def _quaternion_multiply(a, b):
    aw,ax,ay,az=(float(x) for x in a)
    bw,bx,by,bz=(float(x) for x in b)
    return (
        aw*bw-ax*bx-ay*by-az*bz,
        aw*bx+ax*bw+ay*bz-az*by,
        aw*by-ax*bz+ay*bw+az*bx,
        aw*bz+ax*by-ay*bx+az*bw,
    )


def enu_flu_to_ned_frd_quaternion(q):
    values=tuple(float(x) for x in q)
    if len(values)!=4 or not all(math.isfinite(x) for x in values):
        raise ValueError("quaternion must contain four finite values")
    norm=math.sqrt(sum(x*x for x in values))
    if norm<1e-9:
        raise ValueError("quaternion norm is too small")
    q_enu_flu=tuple(x/norm for x in values)
    root_half=math.sqrt(0.5)
    q_ned_enu=(0.0,root_half,root_half,0.0)
    q_flu_frd=(0.0,1.0,0.0,0.0)
    converted=_quaternion_multiply(_quaternion_multiply(q_ned_enu,q_enu_flu),q_flu_frd)
    converted_norm=math.sqrt(sum(x*x for x in converted))
    result=tuple(x/converted_norm for x in converted)
    return tuple(-x for x in result) if result[0]<0 else result


def make_external_vision_sample(
    *,
    timestamp_ns: int,
    position_enu_m,
    velocity_enu_m_s,
    position_variance_m2=(.01,.01,.02),
    velocity_variance_m2_s2=(.04,.04,.06),
    orientation_variance_rad2=(.01,.01,.02),
    angular_velocity_variance_rad2_s2=(.04,.04,.04),
    orientation_enu_flu_wxyz=(1.0,0.0,0.0,0.0),
    angular_velocity_flu_rad_s=(0.0,0.0,0.0),
    reset_counter: int=0,
    quality: float=1.0,
) -> ExternalVisionSample:
    if timestamp_ns < 0:
        raise ValueError("timestamp_ns cannot be negative")
    if not 0.0 <= quality <= 1.0:
        raise ValueError("quality must be within [0,1]")
    if reset_counter < 0:
        raise ValueError("reset_counter cannot be negative")
    vals=(*position_enu_m,*velocity_enu_m_s,*position_variance_m2,*velocity_variance_m2_s2,
          *orientation_variance_rad2,*angular_velocity_variance_rad2_s2,*angular_velocity_flu_rad_s)
    if not all(math.isfinite(float(x)) for x in vals):
        raise ValueError("external vision sample contains non-finite values")
    if any(float(x)<0 for x in (*position_variance_m2,*velocity_variance_m2_s2,
                                *orientation_variance_rad2,*angular_velocity_variance_rad2_s2)):
        raise ValueError("variances cannot be negative")
    return ExternalVisionSample(
        timestamp_us=timestamp_ns//1000,
        position_ned_m=enu_to_ned(position_enu_m),
        velocity_ned_m_s=enu_to_ned(velocity_enu_m_s),
        position_variance_m2=enu_variance_to_ned(position_variance_m2),
        velocity_variance_m2_s2=enu_variance_to_ned(velocity_variance_m2_s2),
        reset_counter=reset_counter,
        quality=quality,
        quaternion_ned_frd_wxyz=enu_flu_to_ned_frd_quaternion(orientation_enu_flu_wxyz),
        angular_velocity_frd_rad_s=flu_to_frd(angular_velocity_flu_rad_s),
        orientation_variance_rad2=tuple(float(x) for x in orientation_variance_rad2),
        angular_velocity_variance_rad2_s2=tuple(float(x) for x in angular_velocity_variance_rad2_s2),
    )


def _upper_triangle_covariance(linear_variance, angular_variance):
    covariance=[0.0]*21
    for index,value in zip((0,6,11),linear_variance):
        covariance[index]=float(value)
    for index,value in zip((15,18,20),angular_variance):
        covariance[index]=float(value)
    return covariance


class MavlinkExternalVisionPublisher:
    """Quality-gated MAVLink 2 ODOMETRY publisher for PX4 external vision."""

    def __init__(
        self,
        connection: Any,
        *,
        min_quality: float=0.2,
        max_age_us: int=200_000,
        max_future_us: int=20_000,
    ) -> None:
        if not 0.0<=min_quality<=1.0:
            raise ValueError("min_quality must be within [0,1]")
        if max_age_us<0 or max_future_us<0:
            raise ValueError("timestamp limits cannot be negative")
        self.connection=connection
        self.min_quality=min_quality
        self.max_age_us=max_age_us
        self.max_future_us=max_future_us

    @classmethod
    def connect(
        cls,
        endpoint: str="udpin:127.0.0.1:14540",
        *,
        heartbeat_timeout_s: float=10.0,
        source_system: int=42,
        source_component: int=191,
        **kwargs,
    ) -> "MavlinkExternalVisionPublisher":
        from pymavlink import mavutil
        connection=mavutil.mavlink_connection(
            endpoint,
            source_system=source_system,
            source_component=source_component,
            dialect="common",
        )
        if connection.wait_heartbeat(timeout=heartbeat_timeout_s) is None:
            connection.close()
            raise TimeoutError(f"no PX4 heartbeat received on {endpoint}")
        return cls(connection,**kwargs)

    def send(self, sample: ExternalVisionSample, *, now_us: int|None=None) -> bool:
        if sample.quality<self.min_quality:
            return False
        if now_us is not None:
            age_us=now_us-sample.timestamp_us
            if age_us>self.max_age_us or age_us < -self.max_future_us:
                return False

        from pymavlink.dialects.v20 import common as mavlink2
        pose_covariance=_upper_triangle_covariance(
            sample.position_variance_m2,
            sample.orientation_variance_rad2,
        )
        velocity_covariance=_upper_triangle_covariance(
            sample.velocity_variance_m2_s2,
            sample.angular_velocity_variance_rad2_s2,
        )
        self.connection.mav.odometry_send(
            sample.timestamp_us,
            mavlink2.MAV_FRAME_LOCAL_NED,
            mavlink2.MAV_FRAME_LOCAL_NED,
            *sample.position_ned_m,
            sample.quaternion_ned_frd_wxyz,
            *sample.velocity_ned_m_s,
            *sample.angular_velocity_frd_rad_s,
            pose_covariance,
            velocity_covariance,
            sample.reset_counter,
            mavlink2.MAV_ESTIMATOR_TYPE_VIO,
            int(round(sample.quality*100.0)),
        )
        return True

    def close(self) -> None:
        self.connection.close()
