import json
import pytest

from vionav.mission import (
    WaypointNed,
    load_estimator_loss_policy,
    load_gps_denied_mission,
    ned_distance,
)


def test_retained_gps_denied_mission_is_ned_external_vision():
    mission=load_gps_denied_mission("simulation/px4/gps_denied_mission.json")
    assert len(mission.waypoints)==5
    assert mission.waypoints[0]==WaypointNed(0.0,0.0,-2.0)
    assert mission.position_error_m_max==1.0


def test_ned_distance_uses_all_axes():
    assert ned_distance((3,4,-2),WaypointNed(0,0,-2))==pytest.approx(5)


def test_invalid_mission_frame_is_rejected(tmp_path):
    path=tmp_path/"mission.json"
    path.write_text(json.dumps({"frame":"ENU","mode":"external_vision","waypoints":[],"acceptance":{"position_error_m_max":1}}))
    with pytest.raises(ValueError): load_gps_denied_mission(path)


def test_retained_estimator_loss_policy_requests_safe_action():
    policy=load_estimator_loss_policy("simulation/px4/failsafe_policy.json")
    assert policy.lost_action=="request_position_hold_or_land"
    assert policy.external_vision_allowed_after_loss is False
    assert policy.estimator_timeout_ms==250
