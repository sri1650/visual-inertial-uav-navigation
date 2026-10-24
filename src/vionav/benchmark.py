from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Sequence
import numpy as np


@dataclass(frozen=True)
class TrajectoryMetrics:
    ate_rmse_m: float
    rpe_translation_rmse_m: float
    samples: int


def _as_array(points: Sequence[Sequence[float]]) -> np.ndarray:
    a=np.asarray(points,dtype=float)
    if a.ndim != 2 or a.shape[1] != 3:
        raise ValueError("trajectory must be Nx3")
    if len(a) < 2:
        raise ValueError("trajectory too short")
    return a


def align_se3(estimated, ground_truth):
    """Rigidly align estimated positions to ground truth without scale."""
    x=_as_array(estimated)
    y=_as_array(ground_truth)
    if len(x)!=len(y):
        raise ValueError("trajectory lengths differ")

    mx=x.mean(axis=0)
    my=y.mean(axis=0)
    xc=x-mx
    yc=y-my
    H=xc.T @ yc / len(x)
    U,_,Vt=np.linalg.svd(H)
    R=Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1,:] *= -1
        R=Vt.T @ U.T
    t=my - R @ mx
    aligned=(R @ x.T).T + t
    return aligned, R, t


def trajectory_metrics(estimated, ground_truth, *, align=True, delta=1):
    est=_as_array(estimated)
    gt=_as_array(ground_truth)
    if len(est)!=len(gt):
        raise ValueError("trajectory lengths differ")
    if align:
        est,_,_=align_se3(est,gt)

    residual=est-gt
    ate=float(np.sqrt(np.mean(np.sum(residual*residual,axis=1))))

    if delta <= 0 or len(est) <= delta:
        raise ValueError("invalid RPE delta")
    de=est[delta:]-est[:-delta]
    dg=gt[delta:]-gt[:-delta]
    dr=de-dg
    rpe=float(np.sqrt(np.mean(np.sum(dr*dr,axis=1))))
    return TrajectoryMetrics(ate,rpe,len(est))


def offset_sensitivity_curve(
    reference_times_ns: Sequence[int],
    estimate_times_ns: Sequence[int],
    offsets_ns: Sequence[int],
):
    """Timestamp association quality proxy for deterministic offset experiments.

    Returns mean nearest-neighbour timestamp residual for each injected offset.
    """
    if not reference_times_ns or not estimate_times_ns:
        raise ValueError("timestamps cannot be empty")
    ref=np.asarray(reference_times_ns,dtype=np.int64)
    est=np.asarray(estimate_times_ns,dtype=np.int64)
    out=[]
    for off in offsets_ns:
        shifted=est+int(off)
        residuals=[]
        for t in ref:
            idx=int(np.searchsorted(shifted,t))
            choices=[]
            if idx < len(shifted): choices.append(abs(int(shifted[idx]-t)))
            if idx > 0: choices.append(abs(int(shifted[idx-1]-t)))
            residuals.append(min(choices))
        out.append({"offset_ns":int(off),"mean_abs_residual_ns":float(np.mean(residuals))})
    return out
