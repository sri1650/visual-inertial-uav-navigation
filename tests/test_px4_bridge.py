import pytest
from vionav.px4_bridge import *

def test_enu_ned_round_trip():
    v=(1.2,-3.4,5.6)
    assert ned_to_enu(enu_to_ned(v)) == pytest.approx(v)

def test_external_vision_conversion():
    s=make_external_vision_sample(
        timestamp_ns=1_234_567_890,
        position_enu_m=(2,3,4),
        velocity_enu_m_s=(.5,.2,-.1),
        quality=.8,
        reset_counter=2,
    )
    assert s.timestamp_us==1_234_567
    assert s.position_ned_m==(3.0,2.0,-4.0)
    assert s.velocity_ned_m_s==pytest.approx((.2,.5,.1))
    assert s.reset_counter==2

def test_invalid_quality_rejected():
    with pytest.raises(ValueError):
        make_external_vision_sample(timestamp_ns=0,position_enu_m=(0,0,0),velocity_enu_m_s=(0,0,0),quality=1.2)
