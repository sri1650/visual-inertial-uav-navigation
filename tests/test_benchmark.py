import numpy as np
import pytest
from vionav.benchmark import align_se3, trajectory_metrics, offset_sensitivity_curve

def test_se3_alignment_removes_rigid_transform():
    gt=np.array([[0,0,0],[1,0,0],[1,1,0],[2,1,0]],dtype=float)
    R=np.array([[0,-1,0],[1,0,0],[0,0,1]],dtype=float)
    t=np.array([3,4,2],dtype=float)
    est=(R.T @ (gt-t).T).T
    m=trajectory_metrics(est,gt,align=True)
    assert m.ate_rmse_m == pytest.approx(0.0,abs=1e-9)
    assert m.rpe_translation_rmse_m == pytest.approx(0.0,abs=1e-9)

def test_metrics_detect_shape_error():
    gt=[[0,0,0],[1,0,0],[2,0,0]]
    est=[[0,0,0],[1.2,0,0],[2.4,0,0]]
    m=trajectory_metrics(est,gt,align=False)
    assert m.ate_rmse_m > 0
    assert m.rpe_translation_rmse_m > 0

def test_offset_curve_minimum_near_zero():
    ts=[0,10,20,30,40]
    curve=offset_sensitivity_curve(ts,ts,[-10,0,10])
    best=min(curve,key=lambda x:x["mean_abs_residual_ns"])
    assert best["offset_ns"] == 0
