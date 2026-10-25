#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import struct
import threading
import time

from pymavlink import mavutil

from vionav.mission import load_gps_denied_mission,ned_distance
from vionav.px4_bridge import MavlinkExternalVisionPublisher,make_external_vision_sample


@dataclass(frozen=True)
class GroundTruth:
    timestamp_ns: int
    position_enu_m: tuple[float,float,float]
    velocity_enu_m_s: tuple[float,float,float]
    orientation_enu_flu_wxyz: tuple[float,float,float,float]


class GazeboGroundTruth:
    def __init__(self, topic: str, model_name: str) -> None:
        try:
            from gz.msgs10.pose_v_pb2 import Pose_V
            from gz.transport13 import Node
        except ImportError as exc:
            raise RuntimeError("Gazebo Python bindings are not available; activate PX4's locked Gazebo environment") from exc
        self._lock=threading.Lock()
        self._latest: GroundTruth|None=None
        self._previous_position: tuple[float,float,float]|None=None
        self._previous_sim_s: float|None=None
        self._node=Node()

        def callback(message) -> None:
            pose=next((item for item in message.pose if item.name==model_name),None)
            if pose is None:
                return
            position=(pose.position.x,pose.position.y,pose.position.z)
            sim_s=float(message.header.stamp.sec)+float(message.header.stamp.nsec)*1e-9
            velocity=(0.0,0.0,0.0)
            with self._lock:
                if self._previous_position is not None and self._previous_sim_s is not None:
                    dt=sim_s-self._previous_sim_s
                    if dt>1e-6:
                        velocity=tuple((position[i]-self._previous_position[i])/dt for i in range(3))
                self._previous_position=position
                self._previous_sim_s=sim_s
                self._latest=GroundTruth(
                    time.monotonic_ns(),position,velocity,
                    (pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z),
                )

        self._node.subscribe(Pose_V,topic,callback)

    def latest(self) -> GroundTruth|None:
        with self._lock:
            return self._latest


class MissionRuntime:
    def __init__(self,connection,ground_truth: GazeboGroundTruth,rate_hz: float) -> None:
        self.connection=connection
        self.ground_truth=ground_truth
        self.publisher=MavlinkExternalVisionPublisher(connection,min_quality=.2,max_age_us=250_000)
        self.period_s=1.0/rate_hz
        self.last_heartbeat_s=0.0
        self.latest_local_position=None
        self.latest_heartbeat=None
        self.status_texts=[]
        self.parameter_values={}
        self.target=None
        self.vision_enabled=True
        self.messages_sent=0

    def tick(self) -> None:
        started=time.monotonic()
        truth=self.ground_truth.latest()
        if truth is not None and self.vision_enabled:
            sample=make_external_vision_sample(
                timestamp_ns=truth.timestamp_ns,
                position_enu_m=truth.position_enu_m,
                velocity_enu_m_s=truth.velocity_enu_m_s,
                orientation_enu_flu_wxyz=truth.orientation_enu_flu_wxyz,
                quality=.95,
            )
            if self.publisher.send(sample,now_us=time.monotonic_ns()//1_000):
                self.messages_sent+=1
        now=time.monotonic()
        if now-self.last_heartbeat_s>=1.0:
            self.connection.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self.last_heartbeat_s=now
        if self.target is not None:
            self._send_setpoint(*self.target)
        while True:
            message=self.connection.recv_match(blocking=False)
            if message is None:
                break
            if message.get_type()=="LOCAL_POSITION_NED":
                self.latest_local_position=(message.x,message.y,message.z)
            elif message.get_type()=="HEARTBEAT" and message.get_srcSystem()==self.connection.target_system:
                self.latest_heartbeat=message
            elif message.get_type()=="STATUSTEXT":
                status_text=message.text.decode() if isinstance(message.text,bytes) else message.text
                self.status_texts.append((time.monotonic(),status_text.rstrip("\x00")))
            elif message.get_type()=="PARAM_VALUE":
                param_id=message.param_id.decode() if isinstance(message.param_id,bytes) else message.param_id
                if message.param_type==mavutil.mavlink.MAV_PARAM_TYPE_INT32:
                    value=struct.unpack("<i",struct.pack("<f",message.param_value))[0]
                else:
                    value=message.param_value
                self.parameter_values[param_id.rstrip("\x00")]=value
        time.sleep(max(0.0,self.period_s-(time.monotonic()-started)))

    def _send_setpoint(self,north,east,down,yaw) -> None:
        ignore_velocity=(1<<3)|(1<<4)|(1<<5)
        ignore_acceleration=(1<<6)|(1<<7)|(1<<8)
        ignore_yaw_rate=1<<11
        self.connection.mav.set_position_target_local_ned_send(
            int(time.monotonic()*1_000)&0xFFFFFFFF,
            self.connection.target_system,self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            ignore_velocity|ignore_acceleration|ignore_yaw_rate,
            north,east,down,0,0,0,0,0,0,yaw,0,
        )

    def set_parameter(self,name: str,value: int,timeout_s: float=8.0) -> None:
        encoded=name.encode("ascii")
        wire_value=struct.unpack("<f",struct.pack("<i",value))[0]
        deadline=time.monotonic()+timeout_s
        next_send=0.0
        while time.monotonic()<deadline:
            if time.monotonic()>=next_send:
                self.connection.mav.param_set_send(
                    self.connection.target_system,self.connection.target_component,
                    encoded,wire_value,mavutil.mavlink.MAV_PARAM_TYPE_INT32,
                )
                next_send=time.monotonic()+1.0
            self.tick()
            if name in self.parameter_values and int(round(self.parameter_values[name]))==value:
                return
        raise TimeoutError(f"PX4 did not confirm {name}={value}")

    def wait(self,duration_s: float) -> None:
        deadline=time.monotonic()+duration_s
        while time.monotonic()<deadline:
            self.tick()

    def armed(self) -> bool:
        return bool(self.latest_heartbeat and self.latest_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def command_mode(self,mode: str) -> None:
        mapping=self.connection.mode_mapping()
        if not mapping or mode not in mapping:
            raise RuntimeError(f"PX4 mode {mode} is unavailable")
        self.connection.set_mode(mode)

    def command_arm(self) -> None:
        self.connection.mav.command_long_send(
            self.connection.target_system,self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0,
        )


def main() -> int:
    parser=argparse.ArgumentParser(description="Run the retained GPS-denied PX4 SITL mission")
    parser.add_argument("--mission",default="simulation/px4/gps_denied_mission.json")
    parser.add_argument("--endpoint",default="udpin:127.0.0.1:14540")
    parser.add_argument("--gz-topic",default="/world/default/dynamic_pose/info")
    parser.add_argument("--model",default="x500_0")
    parser.add_argument("--rate-hz",type=float,default=30.0)
    parser.add_argument("--waypoint-timeout",type=float,default=35.0)
    parser.add_argument("--output",type=Path,default=Path("simulation/px4/evidence/m6-t03-mission.json"))
    args=parser.parse_args()
    mission=load_gps_denied_mission(args.mission)
    truth=GazeboGroundTruth(args.gz_topic,args.model)
    connection=mavutil.mavlink_connection(
        args.endpoint,
        source_system=255,
        source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
        dialect="common",
    )
    if connection.wait_heartbeat(timeout=10) is None:
        raise TimeoutError("no PX4 heartbeat received")
    runtime=MissionRuntime(connection,truth,args.rate_hz)
    results=[]
    mission_started=time.monotonic()
    try:
        deadline=time.monotonic()+5.0
        while truth.latest() is None and time.monotonic()<deadline:
            runtime.tick()
        if truth.latest() is None:
            raise TimeoutError("no Gazebo ground-truth pose received")

        runtime.set_parameter("EKF2_EV_CTRL",15)
        runtime.set_parameter("EKF2_GPS_CTRL",0)
        runtime.wait(5.0)

        first=mission.waypoints[0]
        runtime.target=(first.north_m,first.east_m,first.down_m,math.pi/2)
        runtime.wait(2.0)
        runtime.command_mode("OFFBOARD")
        runtime.wait(1.0)
        runtime.command_arm()
        arm_deadline=time.monotonic()+10.0
        while not runtime.armed() and time.monotonic()<arm_deadline:
            runtime.tick()
        if not runtime.armed():
            raise RuntimeError("PX4 did not arm in Offboard mode")

        for index,waypoint in enumerate(mission.waypoints):
            runtime.target=(waypoint.north_m,waypoint.east_m,waypoint.down_m,math.pi/2)
            deadline=time.monotonic()+args.waypoint_timeout
            stable_since=None
            best_error=float("inf")
            while time.monotonic()<deadline:
                runtime.tick()
                latest=truth.latest()
                if latest is None:
                    continue
                position_ned=(latest.position_enu_m[1],latest.position_enu_m[0],-latest.position_enu_m[2])
                error=ned_distance(position_ned,waypoint)
                best_error=min(best_error,error)
                if error<=mission.position_error_m_max:
                    stable_since=stable_since or time.monotonic()
                    if time.monotonic()-stable_since>=1.0:
                        results.append({"index":index,"target_ned_m":[waypoint.north_m,waypoint.east_m,waypoint.down_m],"error_m":error})
                        break
                else:
                    stable_since=None
            else:
                raise TimeoutError(f"waypoint {index} timed out; best error {best_error:.3f} m")

        runtime.command_mode("LAND")
        land_deadline=time.monotonic()+30.0
        while runtime.armed() and time.monotonic()<land_deadline:
            runtime.tick()
        completed=len(results)==len(mission.waypoints)
        evidence={
            "status":"pass" if completed else "fail",
            "gps_fusion_parameter":0,
            "external_vision_parameter":15,
            "waypoints":results,
            "messages_sent":runtime.messages_sent,
            "duration_s":round(time.monotonic()-mission_started,3),
            "landed_disarmed":not runtime.armed(),
            "simulation_only":True,
        }
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(evidence,indent=2)+"\n",encoding="utf-8")
        print(json.dumps(evidence,indent=2))
        return 0 if completed and not runtime.armed() else 2
    finally:
        connection.close()


if __name__=="__main__":
    raise SystemExit(main())
