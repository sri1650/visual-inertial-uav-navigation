#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path


def main() -> int:
    evidence=Path("simulation/px4/evidence")
    mission=json.loads((evidence/"m6-t03-mission.json").read_text(encoding="utf-8"))
    failsafe=json.loads((evidence/"m6-t04-failsafe.json").read_text(encoding="utf-8"))
    assert mission["status"]=="pass" and mission["landed_disarmed"]
    assert len(mission["waypoints"])==5
    assert max(point["error_m"] for point in mission["waypoints"])<=1.0
    assert failsafe["status"]=="pass" and failsafe["landed_disarmed"]
    assert failsafe["failsafe_mode"]=="DESCEND"
    summary={
        "gps_denied_mission":{
            "waypoints_completed":len(mission["waypoints"]),
            "maximum_waypoint_error_m":max(point["error_m"] for point in mission["waypoints"]),
            "duration_s":mission["duration_s"],
        },
        "estimator_loss_failsafe":{
            "action":failsafe["failsafe_mode"],
            "transition_latency_s":failsafe["transition_latency_s"],
            "landed_disarmed":failsafe["landed_disarmed"],
        },
        "validation_boundary":"PX4/Gazebo SITL only; no physical-flight validation",
    }
    print(json.dumps(summary,indent=2))
    return 0


if __name__=="__main__":
    raise SystemExit(main())
