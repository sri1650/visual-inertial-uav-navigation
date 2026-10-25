import pytest
from vionav.px4_bridge import *


class FakeMav:
    def __init__(self): self.calls=[]
    def odometry_send(self,*args): self.calls.append(args)


class FakeConnection:
    def __init__(self): self.mav=FakeMav(); self.closed=False
    def close(self): self.closed=True

def test_enu_ned_round_trip():
    v=(1.2,-3.4,5.6)
    assert ned_to_enu(enu_to_ned(v)) == pytest.approx(v)

def test_external_vision_conversion():
    s=make_external_vision_sample(
        timestamp_ns=1_234_567_890,
        position_enu_m=(2,3,4),
        velocity_enu_m_s=(.5,.2,-.1),
        position_variance_m2=(1,2,3),
        quality=.8,
        reset_counter=2,
    )
    assert s.timestamp_us==1_234_567
    assert s.position_ned_m==(3.0,2.0,-4.0)
    assert s.velocity_ned_m_s==pytest.approx((.2,.5,.1))
    assert s.position_variance_m2==(2.0,1.0,3.0)
    assert s.reset_counter==2

def test_invalid_quality_rejected():
    with pytest.raises(ValueError):
        make_external_vision_sample(timestamp_ns=0,position_enu_m=(0,0,0),velocity_enu_m_s=(0,0,0),quality=1.2)


def test_attitude_and_angular_rate_frame_conversion():
    s=make_external_vision_sample(
        timestamp_ns=1_000_000,
        position_enu_m=(0,0,0),
        velocity_enu_m_s=(0,0,0),
        orientation_enu_flu_wxyz=(1,0,0,0),
        angular_velocity_flu_rad_s=(1,2,3),
    )
    root_half=2**-0.5
    assert s.quaternion_ned_frd_wxyz==pytest.approx((root_half,0,0,root_half))
    assert s.angular_velocity_frd_rad_s==(1.0,-2.0,-3.0)


def test_mavlink_odometry_message_contains_frames_covariance_and_health():
    connection=FakeConnection()
    publisher=MavlinkExternalVisionPublisher(connection,min_quality=.5)
    sample=make_external_vision_sample(
        timestamp_ns=2_000_000,
        position_enu_m=(2,3,4),
        velocity_enu_m_s=(.5,.2,-.1),
        reset_counter=7,
        quality=.8,
    )
    assert publisher.send(sample,now_us=2_100)
    args=connection.mav.calls[0]
    assert args[0]==2_000
    assert args[1:3]==(1,1)  # MAV_FRAME_LOCAL_NED
    assert args[3:6]==(3.0,2.0,-4.0)
    assert args[13][0:12:6]==pytest.approx((.01,.01))
    assert args[13][11]==pytest.approx(.02)
    assert args[-3:]==(7,3,80)  # reset counter, VIO estimator, quality


def test_publisher_rejects_low_quality_stale_and_future_samples():
    connection=FakeConnection()
    publisher=MavlinkExternalVisionPublisher(connection,min_quality=.5,max_age_us=100,max_future_us=10)
    low=make_external_vision_sample(timestamp_ns=1_000_000,position_enu_m=(0,0,0),velocity_enu_m_s=(0,0,0),quality=.49)
    good=make_external_vision_sample(timestamp_ns=1_000_000,position_enu_m=(0,0,0),velocity_enu_m_s=(0,0,0),quality=.8)
    assert not publisher.send(low,now_us=1_000)
    assert not publisher.send(good,now_us=1_101)
    assert not publisher.send(good,now_us=989)
    assert not connection.mav.calls
