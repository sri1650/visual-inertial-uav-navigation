from dataclasses import dataclass
import pytest
from vionav.fault_injection import shift_timestamps, drop_every_n, drop_burst

@dataclass(frozen=True)
class S:
    timestamp_ns:int

def test_shift():
    x=[S(1),S(2)]
    assert [s.timestamp_ns for s in shift_timestamps(x,5)] == [6,7]

def test_drop_every_n():
    x=[S(i) for i in range(6)]
    assert [s.timestamp_ns for s in drop_every_n(x,3)] == [0,1,3,4]

def test_drop_burst():
    x=[S(i) for i in range(6)]
    assert [s.timestamp_ns for s in drop_burst(x,2,2)] == [0,1,4,5]

def test_reject_n():
    with pytest.raises(ValueError):
        drop_every_n([S(1)],1)
