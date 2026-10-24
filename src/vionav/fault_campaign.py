from __future__ import annotations
from dataclasses import dataclass
from .quality import QualityInputs,QualityState,assess_quality


@dataclass(frozen=True)
class FaultCaseResult:
    name:str
    state:QualityState
    score:float
    reasons:tuple[str,...]


def retained_fault_cases():
    return [
        ("nominal",QualityInputs(120,.9,1,1,2)),
        ("camera_delay_15ms",QualityInputs(110,.85,2,15,2)),
        ("camera_delay_40ms",QualityInputs(110,.85,2,40,2)),
        ("frame_loss",QualityInputs(35,.55,8,4,2)),
        ("severe_frame_loss",QualityInputs(8,.25,30,4,2)),
        ("imu_gap_50ms",QualityInputs(100,.8,5,2,50)),
        ("imu_gap_150ms",QualityInputs(100,.8,5,2,150)),
        ("blur_low_geometry",QualityInputs(40,.3,15,3,2)),
    ]


def run_retained_fault_campaign():
    out=[]
    for name,inputs in retained_fault_cases():
        q=assess_quality(inputs)
        out.append(FaultCaseResult(name,q.state,q.score,q.reasons))
    return out
