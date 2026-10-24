import pytest
from vionav.metrics import rpe_translation_rmse

def test_rpe_zero_for_matching_trajectory():
    t=[(0,0,0),(1,0,0),(2,0,0)]
    assert rpe_translation_rmse(t,t) == 0.0

def test_rpe_known_error():
    est=[(0,0,0),(2,0,0),(4,0,0)]
    gt=[(0,0,0),(1,0,0),(2,0,0)]
    assert rpe_translation_rmse(est,gt) == pytest.approx(1.0)
