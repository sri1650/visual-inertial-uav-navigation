from vionav.video_pipeline import *

def test_synthetic_frame_pipeline():
    s=process_frame(synthetic_video_frame(1, width=320,height=240))
    assert s.feature_count>20
    assert s.elapsed_ms>=0

def test_latency_benchmark_shape():
    r=benchmark_synthetic_frames(5)
    assert r["frames"]==5
    assert r["p95_ms"]>=r["p50_ms"]
    assert r["mean_features"]>0
