import math
import pytest
from vionav.metrics import ate_rmse

def test_zero_error():
    assert ate_rmse([(0,0,0)], [(0,0,0)]) == 0.0

def test_known_error():
    assert ate_rmse([(1,0,0),(0,2,0)], [(0,0,0),(0,0,0)]) == pytest.approx(math.sqrt(2.5))

def test_reject_length_mismatch():
    with pytest.raises(ValueError):
        ate_rmse([(0,0,0)], [])

def test_reject_empty():
    with pytest.raises(ValueError):
        ate_rmse([], [])
