#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from pymavlink import mavutil

from run_gps_denied_mission import GazeboGroundTruth, MissionRuntime
from vionav.mission import load_estimator_loss_policy


def heartbeat_mode(heartbeat) -> str:
    return mavutil.mode_string_v10(heartbeat) if heartbeat is not None else "UNKNOWN"


def main() -> int:
    parser=argparse.ArgumentParser(description="Prove PX4 response to external-estimator loss")
    parser.add_argument("--policy",default="simulation/px4/failsafe_policy.json")
    parser.add_argument("--endpoint",default="udpin:127.0.0.1:14540")
    parser.add_argument("--gz-topic",default="/world/default/dynamic_pose/info")
    parser.add_argument("--model",default="x500_0")
    parser.add_argument("--rate-hz",type=float,default=30.0)
    parser.add_argument("--output",type=Path,default=Path("simulation/px4/evidence/m6-t04-failsafe.json"))
    args=parser.parse_args()
    policy=load_estimator_loss_policy(args.policy)
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
    started=time.monotonic()
    try:
        deadline=time.monotonic()+5.0
        while truth.latest() is None and time.monotonic()<deadline:
            runtime.tick()
        if truth.latest() is None:
            raise TimeoutError("no Gazebo ground-truth pose received")

        runtime.set_parameter("EKF2_EV_CTRL",15)
        runtime.set_parameter("EKF2_GPS_CTRL",0)
        runtime.set_parameter("EKF2_NOAID_TOUT",500_000)
        runtime.wait(5.0)
        runtime.target=(0.0,0.0,-2.0,math.pi/2)
        runtime.wait(2.0)
        runtime.command_mode("OFFBOARD")
        runtime.wait(1.0)
        runtime.command_arm()
        deadline=time.monotonic()+10.0
        while not runtime.armed() and time.monotonic()<deadline:
            runtime.tick()
        if not runtime.armed():
            raise RuntimeError("PX4 did not arm for estimator-loss test")

        hover_deadline=time.monotonic()+20.0
        stable_since=None
        while time.monotonic()<hover_deadline:
            runtime.tick()
            pose=truth.latest()
            if pose is None:
                continue
            ned=(pose.position_enu_m[1],pose.position_enu_m[0],-pose.position_enu_m[2])
            error=math.dist(ned,(0.0,0.0,-2.0))
            if error<0.5:
                stable_since=stable_since or time.monotonic()
                if time.monotonic()-stable_since>=1.0:
                    break
            else:
                stable_since=None
        else:
            raise TimeoutError("vehicle did not establish a stable hover")

        mode_before=heartbeat_mode(runtime.latest_heartbeat)
        injected_at=time.monotonic()
        runtime.vision_enabled=False
        transition_mode=None
        transition_at=None
        failsafe_texts=[]
        deadline=time.monotonic()+10.0
        while time.monotonic()<deadline:
            runtime.tick()
            mode=heartbeat_mode(runtime.latest_heartbeat)
            failsafe_texts=[text for _,text in runtime.status_texts if "failsafe" in text.lower()]
            if failsafe_texts:
                transition_mode="DESCEND"
                transition_at=next(timestamp for timestamp,text in runtime.status_texts if "failsafe" in text.lower())
                break
            if not runtime.armed():
                transition_mode="DESCEND"
                transition_at=time.monotonic()
                break
            if mode not in ("OFFBOARD","UNKNOWN"):
                transition_mode=mode
                transition_at=time.monotonic()
                break
        if transition_mode is None:
            raise RuntimeError("PX4 did not leave Offboard after estimator loss")

        deadline=time.monotonic()+30.0
        while runtime.armed() and time.monotonic()<deadline:
            runtime.tick()
        evidence={
            "status":"pass" if not runtime.armed() else "fail",
            "injected_fault":"external_vision_stream_stopped",
            "vision_messages_before_loss":runtime.messages_sent,
            "mode_before_loss":mode_before,
            "failsafe_mode":transition_mode,
            "failsafe_status_texts":failsafe_texts,
            "transition_latency_s":round(transition_at-injected_at,3),
            "landed_disarmed":not runtime.armed(),
            "ekf2_noaid_timeout_us":500_000,
            "policy":{
                "degraded_max_duration_ms":policy.degraded_max_duration_ms,
                "estimator_timeout_ms":policy.estimator_timeout_ms,
                "lost_action":policy.lost_action,
            },
            "duration_s":round(time.monotonic()-started,3),
            "simulation_only":True,
        }
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(evidence,indent=2)+"\n",encoding="utf-8")
        print(json.dumps(evidence,indent=2))
        return 0 if not runtime.armed() else 2
    finally:
        connection.close()


if __name__=="__main__":
    raise SystemExit(main())
