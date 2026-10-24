from __future__ import annotations
from dataclasses import dataclass
import time
import cv2
import numpy as np

from .vision import PreprocessConfig,preprocess_grayscale,detect_features


@dataclass(frozen=True)
class FrameStats:
    feature_count:int
    elapsed_ms:float


def process_frame(image:np.ndarray, *, max_features:int=500) -> FrameStats:
    start=time.perf_counter_ns()
    gray=preprocess_grayscale(image,PreprocessConfig(clahe_clip_limit=2.0))
    pts=detect_features(gray,max_corners=max_features)
    elapsed=(time.perf_counter_ns()-start)/1e6
    return FrameStats(len(pts),elapsed)


def synthetic_video_frame(index:int, *, width:int=640,height:int=480) -> np.ndarray:
    rng=np.random.default_rng(index)
    image=np.zeros((height,width),dtype=np.uint8)
    for _ in range(150):
        x=int(rng.integers(5,width-5)); y=int(rng.integers(5,height-5))
        cv2.circle(image,(x,y),int(rng.integers(1,4)),int(rng.integers(80,255)),-1)
    cv2.putText(image,f"{index:04d}",(20,40),cv2.FONT_HERSHEY_SIMPLEX,1,255,2)
    return image


def benchmark_synthetic_frames(count:int=50):
    if count<=0: raise ValueError("count must be positive")
    rows=[process_frame(synthetic_video_frame(i)) for i in range(count)]
    lat=sorted(x.elapsed_ms for x in rows)
    def pct(p):
        k=(len(lat)-1)*p; lo=int(k); hi=min(lo+1,len(lat)-1); f=k-lo
        return lat[lo]*(1-f)+lat[hi]*f
    return {
        "frames":count,
        "mean_ms":float(sum(lat)/len(lat)),
        "p50_ms":float(pct(.5)),
        "p95_ms":float(pct(.95)),
        "max_ms":float(max(lat)),
        "mean_features":float(sum(x.feature_count for x in rows)/len(rows)),
    }
